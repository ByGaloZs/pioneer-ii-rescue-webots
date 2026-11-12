# Resultados

## Prueba de ida (paso 2): 10 de 10 mundos superados

Configuración: **con cámara, con mapa + A\***, valores por defecto de `RobotConfig.h`, un intento por mundo, límite de 240 s simulados.
Fuente: `results/outbound_results.csv`.

| Mundo | Inicio (x, z) | Éxito | Tiempo (s) | Recorrido (m) | Líneas | Obstáculos | Cambios de lado | Atascos | Frenos | Casi-contactos | Cámara válida | Replanif. |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| s01 | (3,00, −9,00) | sí | 55,1 | 21,09 | 8/8 | 2 | 0 | 0 | 0 | 22 | 100 % | 206 |
| s02 | (0,00, −9,05) | sí | 99,7 | 30,32 | 8/8 | 5 | 0 | 0 | 0 | 66 | 100 % | 389 |
| s03 | (0,00, −9,05) | sí | 181,6 | 53,35 | 8/8 | 13 | 1 | 0 | 0 | 93 | 85 % | 708 |
| s04 | (0,00, −9,05) | sí | 58,6 | 23,14 | 8/8 | 1 | 0 | 0 | 0 | 22 | 100 % | 219 |
| s05 | (0,00, −9,05) | sí | 107,1 | 38,38 | 8/8 | 4 | 0 | 0 | 0 | 48 | 100 % | 429 |
| s06 | (0,00, −9,05) | sí | 130,8 | 34,65 | 8/8 | 12 | 0 | 0 | 0 | 95 | 100 % | 496 |
| s07 | (0,00, −9,05) | sí | 158,9 | 44,48 | 8/8 | 15 | 0 | 0 | 0 | 108 | 100 % | 641 |
| s08 | (0,00, −9,05) | sí | 120,8 | 37,26 | 8/8 | 6 | 0 | 0 | 0 | 89 | 100 % | 482 |
| s09 | (0,00, −9,05) | sí | 56,6 | 21,33 | 8/8 | 3 | 0 | 0 | 0 | 19 | 100 % | 211 |
| s10 | (0,00, −9,05) | sí | 68,1 | 22,41 | 8/8 | 4 | 0 | 0 | 0 | 14 | 100 % | 256 |

Resumen: 10/10 éxitos · tiempo medio 103,7 s (mín. 55,1 s en s01, máx. 181,6 s en s03) · recorrido medio 32,6 m · 0 atascos · 0 frenos de emergencia ·
8/8 líneas cruzadas en todos.

### Lectura de los resultados

- **Tiempo y recorrido crecen con el número de obstáculos rodeados**, no con la longitud del recinto: s04, s09, s01 y s10 (1–4 obstáculos) tardan 55–68 s; s03, s07 y s06
  (12–15 obstáculos) tardan 131–182 s. s03 es además el único mundo con un cambio de lado (media vuelta) y con la cámara válida solo el 85 % del tiempo (poca luz).
- **Margen respecto al límite:** el peor caso (s03, 181,6 s) deja 58 s de margen sobre los 240 s. Si se añaden mundos más densos, este es el primer sitio donde se
  agotaría el tiempo.
- **Recorrido eficiente:** el mundo más corto (s09) recorre 21,3 m para avanzar 17,3 m en z (de −9,05 a 8,2), es decir, un 24 % más que la línea recta.
- **Replanificaciones:** entre 206 y 708. Siguen el número de obstáculos porque cada pared nueva en el mapa dispara un replan.
- **Casi-contactos** (14–108): cuenta episodios en que algún IR marcó < 3 cm **o está saturado**. Como `min_ir_m` es 0,104 m en todas las pruebas (ningún IR midió menos de 10 cm),
  estos conteos reflejan la bandera de saturación y no un roce confirmado. Para saber si hubo colisiones hay que revisar la simulación visualmente.
- **Frenos de emergencia = 0:** el freno a 6 cm nunca llegó a activarse; las maniobras con IR (repulsión lateral, seguimiento de pared a 15 cm) bastaron.

### Qué no cubren estos datos

- Solo hay filas con `camera = cam` y `map = map`. Falta la comparación con `nocam` y `nomap` (Bug puro).
- Un único intento por mundo; no se ha medido la variación entre ejecuciones ni otras posiciones de salida (solo s01 arrancó en x = 3,00).
- El tiempo es simulado (Webots), no tiempo real.

## Cómo reproducir

1. Abrir el mundo `worlds/final_scenarioNN.wbt`, compilar y pulsar Play (sin `controllerArgs`, valen los valores por defecto).
2. Esperar la línea `[RESULT] IDA COMPLETADA`.
3. La fila nueva aparece al final de `results/outbound_results.csv`.

Variantes para la comparación: añadir `nocam` o `nomap` en `controllerArgs` (un argumento por línea) y, opcionalmente, `tag=s03-nocam` para distinguir las filas.

## Columnas de `outbound_results.csv`

| Columna | Significado |
|---|---|
| `when` | Fecha y hora de la ejecución. El controlador escribe `AAAA-MM-DD hh:mm:ss`; si abres el CSV en una hoja de cálculo y lo guardas, puede reformatearlo |
| `tag` | Mundo (`s01`…`s10`, detectado automáticamente) o el valor de `tag=` |
| `camera` | `cam` o `nocam` |
| `start_x`, `start_z`, `start_yaw_deg` | Pose inicial (el yaw 180° = mirando a +Z) |
| `success` / `time_s` | 1 si cruzó la línea final (z ≥ 8,2) / tiempo simulado desde el arranque |
| `max_z` / `lines` | Máximo z alcanzado / líneas cruzadas de 8 (7 negras de z = −6 a 6 y la amarilla en z = 8) |
| `path_m` | Distancia recorrida según los encoders |
| `obstacles` | Veces que se empezó a rodear un obstáculo (`FOLLOW_WALL`) |
| `side_switches` | Medias vueltas para rodear por el otro lado o por plan (callejones) |
| `stuck_recoveries` | Veces que se activó `RECOVER` por atasco |
| `emergency_stops` | Episodios del `SafetyGuard` (IR delantero < 6 cm con orden de avance) |
| `near_contacts` | Episodios con algún IR < 3 cm o saturado (indicador de proximidad, no choque confirmado) |
| `min_ir_m` | Distancia IR mínima vista en toda la prueba (−1 si no detectó nada) |
| `map` | `map` o `nomap` |
| `cam_valid_pct` | % de fotogramas en que la cámara fue utilizable (0 si no se usó) |
| `replans` | Número de veces que se ejecutó el A\* |

Si cambian las columnas en `OutboundRun.cpp`, el CSV anterior se renombra a `outbound_results_old.csv` y se empieza uno nuevo.

## Mapas ASCII (`results/map_<mundo>_<cam|nocam>.txt`)

Dibujo de 100 × 200 caracteres (10 cm por carácter), con **arriba = +z** (víctimas) e **izquierda = −x**.

| Símbolo | Significado |
|---|---|
| `#` | Celda ocupada (pared exterior fija u obstáculo visto por los IR) |
| `.` | Libre, vista por algún rayo IR |
| `o` | Celda por la que pasó el robot |
| `*` | Último plan A\* calculado |
| ` ` | Desconocido |

Útil para el informe: muestra qué obstáculos llegó a ver el robot y por dónde pasó. Si el archivo se llama `map_s??_...`, el controlador no pudo
detectar el mundo; usar `tag=sNN` en `controllerArgs`. Actualmente `results/` contiene un único mapa de ese tipo (`map_s??_cam.txt`), de una ejecución anterior
sin etiqueta; vuelve a ejecutar cada mundo con `tag=sNN` si quieres un mapa por escenario.

## Pendiente para la tabla final

| Prueba | Estado |
|---|---|
| Ida con cámara + mapa, 10 mundos | Hecho |
| Ida sin cámara (`nocam`), 10 mundos | Pendiente |
| Ida sin mapa (`nomap`), 10 mundos | Pendiente (opcional, para justificar el A\*) |
| Vuelta a la salida | Pendiente (paso 3) |
| Víctimas: detección, aproximación y giro de 360° | Pendiente (paso 4) |
