# Navegación y misión (ida, víctimas y vuelta)

Cómo llega el robot a la línea amarilla del fondo en un recinto que no conoce y, después, cómo rescata a las víctimas y vuelve. Las secciones de la máquina de estados
y del mapa describen la navegación (ida y vuelta); la sección «Misión completa» explica la secuencia. Para el resumen de módulos ver
[ARQUITECTURA.md](ARQUITECTURA.md); los números citados aquí están en [CONFIGURACION.md](CONFIGURACION.md).

## Idea general

Hay tres capas que cooperan, de la más "inteligente" a la más reactiva:

| Capa | Módulo | Qué aporta | Depende de la luz |
|---|---|---|---|
| Memoria + plan | `OccupancyGrid` + `PathPlanner` | Recuerda paredes y callejones; calcula el camino más corto conocido | No (solo IR + pose) |
| Anticipación | `FloorSegmenter` + `SphericalMapper` | Huecos libres a varios metros (frontal); paredes alrededor del robot a ~1,5 m (esférica) | Sí (se desactivan con poca luz) |
| Reacción | IR en `GoalNavigator` + `SafetyGuard` | Rodea lo que aparece delante, evita roces, frena en emergencia | No |

Si la capa de arriba falla (cámara oscura, plan no disponible), la de abajo sigue funcionando. Por eso el robot completó la ida en los
10 escenarios aunque dos son casi a oscuras (02 y 03) y varios tienen niebla (04–09).

## Dirección de la meta

El navegador trabaja con un único dato: `goalRel`, el ángulo (rad) entre hacia dónde mira el robot y hacia dónde debe ir.
- **Con mapa:** el rumbo hacia el punto de mira del plan A\* (`lookaheadYaw`, a ≤ 0,8 m sobre la ruta).
- **Meta puntual** (víctimas, tramos de búsqueda): `setGoalPoint` planifica hasta celdas a ≤ r del punto; sin plan, rumbo recto al punto.
- **Sin mapa (`nomap`):** recto hacia +Z (`yaw = π`).

El progreso se mide como longitud restante del plan (con mapa) o `z` restante (sin mapa).

## Máquina de estados (`GoalNavigator`)

```
        ┌─────────┐  |goalRel| < 17°   ┌────────────┐   obstáculo delante / paso estrecho
 inicio │  ALIGN  │ ────────────────►  │ GO_TO_GOAL │ ─────────────────────────────┐
 ──────►│ (pivot) │ ◄───────────────── │            │                              ▼
        └─────────┘   |goalRel| > 60°  └────────────┘                       ┌─────────────┐
             ▲                                ▲                              │ FOLLOW_WALL │
             │            progreso + meta libre│                              │   _L / _R   │
             │                                └──────────────────────────────┤             │
             │ plan pide ir atrás (callejón)                                 └──┬───────┬──┘
             └───────────────────────────────────────────────────────────────────┘       │ cuña / retroceso excesivo
                                                                                          ▼
                                                              ┌─────────────┐   ┌─────────────┐
                                                              │ REVERSE_OUT │──►│ TURN_AROUND │──► FOLLOW_WALL (otro lado)
                                                              └─────────────┘   └─────────────┘
        Cualquier estado ── robot no se mueve ──► RECOVER ──► FOLLOW_WALL o ALIGN
```

### ALIGN
Gira en el sitio hasta que la meta queda a menos de ~17° (0,3 rad), y entonces pasa a `GO_TO_GOAL`.
- Sentido por defecto: el giro más corto. Si el giro es grande (> 90°) y la trasera del lado que barrería está pegada a algo (< 12 cm), gira al revés.
- El sentido se fija al entrar y no cambia aunque el plan oscile; si en 4 s no consigue girar (encajado), prueba el contrario una vez.
- Usa `safePivot` (ver más abajo).

### GO_TO_GOAL
Avanza hacia la meta. En cada paso:
1. **¿Obstáculo?** Si ds0/ds15 < 18 cm, ds1/ds14 < 15 cm, o hay obstáculos a < 10 cm a ambos lados a la vez (paso demasiado estrecho),
   se pasa a `FOLLOW_WALL` (cuenta como un obstáculo rodeado).
2. **Rumbo y velocidad.**
   - *Sin cámara válida:* rumbo = meta, velocidad 4,0 rad/s (≈ 0,33 m/s).
   - *Con cámara válida (y meta a < 26°):* prueba rumbos de ±20° cada 2° y mide con `corridorClearance` el espacio libre en un pasillo de
     ±0,30 m. Elige el rumbo más cercano a la meta (y al anterior, para no oscilar) con al menos 2 m libres; si ninguno llega, va al más
     despejado. La velocidad sube de 2,5 a 8,0 rad/s (≈ 0,66 m/s) según el espacio libre (de 0,8 m a 2,5 m).
   - Si la cámara ve un hueco desviado más de 4° de la meta, **recuerda el lado** para usarlo si luego choca.
   - Con cualquier IR delantero detectando (ds0, 1, 2, 13, 14, 15) la velocidad baja a 2,5 rad/s pase lo que pase.
3. **Repulsión lateral:** si ds2/ds3 o ds12/ds13 detectan algo a < 14 cm, se añade un giro proporcional (ganancia 15) para apartarse.
4. Giro = `4,0 · error de rumbo` (limitado a ±2,0) + repulsión.

### FOLLOW_WALL_L / FOLLOW_WALL_R
Rodea el obstáculo con los IR (algoritmo tipo **Bug**). `_L`: la pared queda a la izquierda; `_R`: a la derecha.

**Elección de lado** (`startFollowing`), por orden de prioridad:
1. Si hay plan, el lado por el que el plan rodea el obstáculo.
2. Si la cámara vio un hueco en los últimos 2 s, ese lado.
3. El lado con el obstáculo más lejano (diferencia > 3 cm).
4. Cerca de una pared lateral del recinto (|x| > 2,5): hacia el centro.
5. El lado hacia el que está la meta.

**Pasos de cada ciclo** (en este orden):
0. *Engaging:* gira en el sitio hasta dejar el obstáculo de lado.
1. *Esquina interior* (obstáculo delante < 18 cm, diagonal < 12 cm): gira alejándose de la pared; si la pared lateral está muy cerca y hay sitio atrás, retrocede antes.
1b. *Cuña:* el paso se estrecha por el lado contrario a la pared mientras se sigue la pared → `REVERSE_OUT`.
2. *Soltar la pared:* si avanzó ≥ 25 cm hacia la meta respecto al choque (o, con plan, lleva > 1,5 s rodeando), la meta no queda "detrás" de la pared
   y la dirección de la meta está libre (IR a ±45° y, con cámara, ≥ 0,7 m) → vuelve a `GO_TO_GOAL`/`ALIGN`.
2b. *Callejón (con plan):* si el plan pide ir hacia atrás (> 135°) durante 1 s, deja la pared, cuenta un cambio de lado y gira (`ALIGN`) para seguir el plan.
3. *Regresión (sin plan):* si se aleja más de 1,5 m de la meta, o lleva más de 35 s rodeando, `TURN_AROUND` para rodear por el otro lado (una vez por obstáculo).
4. *Pared perdida* (esquina exterior): 0,4 s recto y luego arco hacia la pared.
5. *Seguimiento:* control proporcional para mantener 15 cm de la pared (ganancia 15) y paralelismo con ds3/ds4 (ganancia 12), a 3,0 rad/s (más lento si está a < 9 cm).

### REVERSE_OUT y TURN_AROUND
`REVERSE_OUT` retrocede hasta 0,8 s (o hasta que el IR trasero ve algo a < 10 cm) para no encajarse en una cuña. `TURN_AROUND` da una media vuelta
(~150°) girando **hacia** la pared (así la parte trasera, que sobresale, barre hacia fuera) y vuelve a `FOLLOW_WALL` con el lado cambiado.

### RECOVER (detector de atasco)
Cada 1,5 s se compara lo que se ordenó con lo que pasó: se ordenó avanzar y el robot se movió < 3 cm, o se ordenó girar y el yaw cambió < 0,05 rad.
Entonces: retrocede 0,6 s (o gira si atrás no hay sitio), gira 0,7 s hacia el lado con más espacio y reanuda (`FOLLOW_WALL` si estaba rodeando, `ALIGN` si no).
Se cuenta en `stuck_recoveries`. En los 10 mundos fue 0.

### `safePivot`: giro sin golpear con la trasera
El Pioneer II sobresale por detrás del eje, así que al girar en el sitio la cola barre hacia el lado contrario. `safePivot` comprueba los IR traseros de ese lado
(< 9 cm) y, si están pegados, hace un escape corto (retroceder, avanzar o girar muy despacio) durante 0,6 s. Tras tres escapes seguidos sin cambiar de orientación, fuerza
un giro lento de 1 s para no entrar en bucle.

## Mapa y plan

- **Cada paso** `RescueMission` (o `OutboundRun`) llama a `OccupancyGrid::update` con la pose y los 16 IR (ver [ARQUITECTURA.md](ARQUITECTURA.md), sección 5).
- **Replanificación** (`replanIfNeeded`): cada 0,3 s, y además cuando el mapa cambia (una celda pasa a ocupada/libre), como máximo cada 0,1 s. En los resultados,
  el número de replanificaciones (`replans`) va de ~190 a ~500 según el mundo (ida en Webots, 10 mundos).
- Al tocar un obstáculo se replanifica **de inmediato** para decidir por qué lado rodearlo.
- Si no existe ningún camino (`plan=NO` en consola), el navegador vuelve temporalmente al comportamiento sin mapa (recto hacia +Z).

- **Cámara esférica:** cada 2 pasos `SphericalMapper` añade evidencia de paredes a 0,5–1,6 m en 360°; se descarta con el suelo más oscuro de 25/255.
- **Vuelta:** `setPreferVisited(true)` hace que las celdas recorridas cuesten menos que las no recorridas (0,06 m extra), así el A* tiende a repetir el camino ya andado.

## Misión completa (`RescueMission`)

```
IDA ──z ≥ 8,2──► BUSCAR ──víctima confirmada──► IR_A_VICTIMA ──≤ 1,25 m──► ACERCAR ──≤ 0,85 m──► GIRO_360 ──► BUSCAR ... ──2 rescatadas / sin tiempo / sin víctimas──► VUELTA ──z ≤ −8,2──► FIN
```

- **Detección:** cada paso se alimentan las observaciones de la cámara frontal (base del bloque verde) y de la esférica (píxel verde más cercano por dirección) a un
  `VictimTracker`. Una víctima se confirma con 5 imágenes; las observaciones cercanas (< 1,7 m) corrigen las lejanas. Detalle en [ARQUITECTURA.md](ARQUITECTURA.md), sección 4.
- **Búsqueda:** si no hay víctima confirmada, barre el pasillo del fondo con tres tramos (a x = ±3,2 y x = 0, z = 8,9); cada tramo caduca a los 30 s.
- **Aproximación:** A* hasta 1,15 m y avance directo hasta 0,85 m del centro de la víctima (retrocede si ya está a < 0,7 m); se abandona a los 45 s (ir) o 15 s (acercar).
- **Giro y mensaje:** giro a la izquierda hasta acumular 6,40 rad (límite 8 s; si no se alcanza, la víctima **no** cuenta como rescatada y se marca fallida) e impresión de `[VICTIMA n] Víctima encontrada ... a d m del robot` y `... VICTIMA RESCATADA ...`.
  Después la víctima se marca como obstáculo circular (0,3 m) en el mapa y no se vuelve a contar.
- **Tiempo:** se deja de buscar si `tiempo + 0,55 × tiempo de ida + 12 s + 12 s de margen` supera 240 s; entonces se vuelve con las víctimas rescatadas. También se comprueba en `ACERCAR`: si aún está a ≥ 1 m de la víctima, se abandona y se vuelve (una vez a < 1 m se termina el giro, ~2,5 s).
- **Vuelta:** `setGoalLine(−8,6)` con preferencia por lo recorrido; termina en z ≤ −8,2 (supuesto: simétrico a la ida; la línea amarilla de salida está en z = −8).

## Cámara en la navegación

La cámara es una ayuda, no un requisito. Se usa para: (1) elegir el hueco más cercano a la meta antes de llegar al obstáculo, (2) ir más rápido cuando hay
mucho espacio libre, (3) comprobar que la dirección de la meta está libre al soltar la pared. Si el suelo se ve más oscuro de 12/255 el perfil no es válido y esas
tres funciones se omiten (en el mundo s03 la cámara fue válida solo el 85 % del tiempo).

## Cómo depurar una ejecución

En el modo `navigate` la consola imprime cada segundo:

```
[NAV] t=57.0 FOLLOW_WALL_L pos=(0.31, 2.40) yaw=170 restante=6.1 libre_cam=1.9 luz=88 plan=ok ocupadas=412 F=0.15
```

| Campo | Significado |
|---|---|
| `t` | segundos desde el inicio de la prueba |
| estado | `ALIGN`, `GO_TO_GOAL`, `FOLLOW_WALL_L/R`, `REVERSE_OUT`, `TURN_AROUND`, `RECOVER` |
| `pos`, `yaw` | pose (m, grados) |
| `restante` | longitud del plan (con mapa) o distancia en z hasta la meta |
| `libre_cam` | espacio libre visto por la cámara en el rumbo elegido (m) |
| `luz` | brillo del suelo en la cámara (0–255) |
| `plan` / `ocupadas` | si hay plan válido / celdas ocupadas en el mapa |
| `F` | distancia del sector frontal (`--` si no detecta nada) |

En el modo `mission` la línea es `[MISION] t=… <fase> pos=(x, z) yaw=… victimas vistas=… rescatadas=… plan=ok restante=…`, donde `<fase>` es `IDA`, `BUSCAR`,
`IR_A_VICTIMA`, `ACERCAR`, `GIRO_360` o `VUELTA`. Además se imprimen los eventos (`[MISION] Víctima confirmada…`, `[VICTIMA n] …`) y la lista de candidatas al terminar la ida.
El visor `tools/live_map.py` marca las víctimas (círculo = vista, cruz = rescatada).

Para comparar enfoques, ejecutar el mismo mundo con `nocam`, `nomap` o `nosphere` y comparar las filas del CSV.
