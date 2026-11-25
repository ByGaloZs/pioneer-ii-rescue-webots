# Resultados

Todos los tiempos son **segundos simulados** y todas las cifras salen de ejecuciones en **Webots**.
Las carpetas `results/` y `logs/` no se versionan: los CSV, mapas y consolas se generan al ejecutar y son los de la copia de trabajo de quien ejecuta.
Las tablas de la misión completa (secciones 3 y 4) salen de las cuatro tandas del 24-11-2026, guardadas en `results/Tanda A`…`Tanda D` (CSV y mapas) y
`logs/Tanda A`…`Tanda D` (consola de cada ejecución). Las de la ida (secciones 1 y 2) son **resúmenes copiados de ejecuciones anteriores** y no se pueden
verificar con los archivos actuales.

## Qué está validado

| Prueba | Estado |
|---|---|
| Ida con mapa + esférica + cámara, 10 mundos | **10/10** (tabla 1, histórica: modo `navigate`, ya retirado) |
| Ida sin esférica (solo IR + cámara), 10 mundos | Hecha antes de la esférica (tabla 2, histórica) |
| **Misión completa** con el controlador de entrega, 10 mundos | **10/10 misiones completas**, **20/20 víctimas**, todas en ≤ 240 s (tabla 3) |
| Comparación `nocam` / `nosphere` / `nomap`, 10 mundos | Hecha (sección 4) |

## 1. Ida en Webots (histórica)

Configuración: cámara frontal + mapa + A\* + cámara esférica en el mapa, valores por defecto, un intento por mundo, modo `navigate`.
Origen: `results/outbound_results.csv` (filas `sphere`). El modo `navigate` se ha retirado del controlador (la misión completa ya mide la ida,
columna `outbound_s`), así que esta tabla se conserva como referencia y no se puede repetir. En s01 el robot salía entonces de (3, −9).

| Mundo | Inicio (x, z) | Éxito | Tiempo (s) | Recorrido (m) | Líneas | Obstáculos | Cambios de lado | Casi-contactos | Esférica válida | Replanif. |
|---|---|---|---|---|---|---|---|---|---|---|
| s01 | (3,00, −9,00) | sí | 33,2 | 19,49 | 8/8 | 0 | 0 | 2 | 100 % | 187 |
| s02 | (0,00, −9,05) | sí | 67,4 | 23,48 | 8/8 | 1 | 0 | 12 | 100 % | 347 |
| s03 | (0,00, −9,05) | sí | 100,3 | 31,94 | 8/8 | 4 | 1 | 38 | 87 % | 500 |
| s04 | (0,00, −9,05) | sí | 40,9 | 20,63 | 8/8 | 0 | 0 | 5 | 100 % | 228 |
| s05 | (0,00, −9,05) | sí | 65,4 | 35,42 | 8/8 | 0 | 0 | 0 | 100 % | 355 |
| s06 | (0,00, −9,05) | sí | 63,5 | 26,84 | 8/8 | 1 | 0 | 14 | 100 % | 341 |
| s07 | (0,00, −9,05) | sí | 54,9 | 27,05 | 8/8 | 0 | 0 | 19 | 100 % | 295 |
| s08 | (0,00, −9,05) | sí | 49,7 | 26,25 | 8/8 | 0 | 0 | 0 | 100 % | 282 |
| s09 | (0,00, −9,05) | sí | 39,6 | 20,40 | 8/8 | 0 | 0 | 2 | 100 % | 203 |
| s10 | (0,00, −9,05) | sí | 65,5 | 32,62 | 8/8 | 0 | 0 | 4 | 100 % | 346 |

Resumen: 10/10 · tiempo medio 58,0 s (mín. 33,2 s en s01, máx. 100,3 s en s03) · recorrido medio 26,4 m · 0 atascos · 0 frenos de emergencia.
Control con `nosphere` (mismos valores, sin la esférica): s03 181,6 s y s07 158,9 s, es decir, la esférica reduce esos dos tiempos un 45 % y un 65 %.

## 2. Ida en Webots sin cámara esférica (histórica)

Versión anterior (cámara frontal + mapa solo con IR). Se conserva como referencia para comparar; el archivo CSV original de esa serie se apartó al cambiar las columnas
(`outbound_results_old_<fecha>.csv`) y puede no estar en tu copia, por lo que estas cifras ya no son reproducibles solo con los archivos actuales.

| Mundo | s01 | s02 | s03 | s04 | s05 | s06 | s07 | s08 | s09 | s10 |
|---|---|---|---|---|---|---|---|---|---|---|
| Tiempo (s) | 55,1 | 99,7 | 181,6 | 58,6 | 107,1 | 130,8 | 158,9 | 120,8 | 56,6 | 68,1 |
| Obstáculos rodeados | 2 | 5 | 13 | 1 | 4 | 12 | 15 | 6 | 3 | 4 |

Media 103,7 s. Sin memoria (Bug puro) fallaban los mundos 03, 05, 07 y 09 (callejones sin salida).

## 3. Misión completa en Webots (tanda A, controlador de entrega)

Modo `mission`, `controllerArgs` = `mission nolive tag=NN`, límite 240 s, un intento por mundo, salida original (0; −9,05) mirando a +Z en los 10 mundos,
robot `Pioneer2_Robot`. Tanda del 24-11-2026 con el código final (incluida la regla de aplazamiento de la sección 5). Origen: `results/Tanda A/mission_results.csv`.

**Dos medidas distintas:** *Vuelta a tiempo* (columna `success` del CSV) solo indica que el robot volvió a la salida (z ≤ −8,2) antes del límite, sin mirar las
víctimas; *Víctimas* (columna `victims`) cuenta los giros de rescate. Una **misión completa** exige las dos cosas: vuelta a tiempo y 2/2 víctimas.

| Mundo | Vuelta a tiempo | Víctimas | Total (s) | Ida (s) | Víctima 1 (s) | Víctima 2 (s) | Vuelta (s) | Líneas de vuelta | Recorrido (m) | Casi-contactos |
|---|---|---|---|---|---|---|---|---|---|---|
| s01 | sí | 2/2 | 104,1 | 40,0 | 50,0 | 70,7 | 33,3 | 7 | 44,68 | 0 |
| s02 | sí | 2/2 | 134,0 | 67,4 | 74,5 | 90,4 | 43,5 | 7 | 49,93 | 17 |
| s03 | sí | 2/2 | 199,3 | 99,5 | 107,3 | 129,7 | 69,6 | 7 | 66,37 | 49 |
| s04 | sí | 2/2 | 96,1 | 41,2 | 50,3 | 63,0 | 33,1 | 7 | 41,83 | 7 |
| s05 | sí | 2/2 | 149,6 | 65,4 | 74,6 | 107,2 | 42,3 | 7 | 68,70 | 0 |
| s06 | sí | 2/2 | 137,7 | 63,6 | 72,5 | 103,5 | 34,2 | 7 | 53,83 | 16 |
| s07 | sí | 2/2 | 123,2 | 54,9 | 61,8 | 84,3 | 38,9 | 7 | 53,92 | 19 |
| s08 | sí | 2/2 | 110,6 | 49,7 | 60,3 | 76,0 | 34,6 | 7 | 50,14 | 0 |
| s09 | sí | 2/2 | 122,7 | 39,6 | 59,8 | 87,0 | 35,6 | 7 | 51,30 | 4 |
| s10 | sí | 2/2 | 122,1 | 65,5 | 75,4 | 86,3 | 35,8 | 7 | 54,52 | 22 |

Resumen: **misión completa en 10/10** · **20/20 víctimas** · tiempo total medio **129,9 s** (mín. 96,1 s en s04, máx. 199,3 s en s03, 40,7 s de margen) ·
ida media 58,7 s · vuelta media 40,1 s · 0 frenos de emergencia.

**Verificación de los rescates.** Los rescates de las 40 ejecuciones (tandas A–D) se comprobaron visualmente en Webots: los dos giros de cada misión se hicieron
junto a dos víctimas distintas y a menos de 1 m. La columna `victim1_dist_m`/`victim2_dist_m` es la distancia a la posición **estimada** de la víctima (casi siempre
0,85 m, la distancia de parada), así que no sirve por sí sola para comprobar la regla de 1 m. Los casi-contactos son lecturas IR < 3 cm o saturadas, no choques confirmados.

**Respecto a la tanda del 17-11 (también 10/10 y 20/20):** s01 sale ahora de la posición original (0; −9,05) y no de (3; −9): 104,1 s en vez de 98,5 s.
s10 pasa de 121,9 s a 122,1 s por la regla de aplazamiento. Los otros ocho mundos dan exactamente las mismas cifras.

## 4. Comparación de variantes (tandas A–D)

Cada tanda desactiva una capa con `controllerArgs`, sin recompilar. En B, C y D se usó `timelimit=600` para medir cuánto tarda cada variante aunque pase de
240 s; con el límite ampliado la regla de vuelta deja más tiempo para buscar, así que estas tandas miden el rendimiento **con límite ampliado**. Para el
enunciado cuenta `time_s` ≤ 240.

| Tanda | Variante | `controllerArgs` | Misiones completas | Víctimas | Completas en ≤ 240 s | Ida media (s) | Recorrido medio (m) | Casi-contactos (total) |
|---|---|---|---|---|---|---|---|---|
| A | Completo (entrega) | `mission nolive tag=NN` | 10/10 | 20/20 | 10/10 | 58,7 | 53,5 | 134 |
| B | Sin cámara frontal | `… nocam timelimit=600` | 10/10 | 20/20 | 10/10 | 81,3 | 52,6 | 78 |
| C | Sin cámara esférica | `… nosphere timelimit=600` | 9/10 | 19/20 | 6/10 | 105,0 | 67,8 | 1022 |
| D | Sin mapa (Bug) | `… nomap timelimit=600` | 6/10 | 14/20 | 2/10 | 191,1 \* | 100,5 | 2187 |

\* Media de los 9 mundos en que terminó la ida. En s09 sin mapa la ida no terminó: el CSV pone `outbound_s` = 60,0, que es el valor por defecto
(`_outboundS`) cuando la ida no acaba, no un tiempo real.

Tiempo total (s) por mundo. En *cursiva*, misiones completas por encima de 240 s (solo posibles con `timelimit=600`); con guion, misiones no completas
(entre paréntesis, víctimas rescatadas).

| Mundo | A: completo | B: sin frontal | C: sin esférica | D: sin mapa |
|---|---|---|---|---|
| s01 | 104,1 | 175,5 | 143,5 | 167,0 |
| s02 | 134,0 | 170,0 | 185,5 | *285,4* |
| s03 | 199,3 | 225,1 | *324,4* | — (2/2) |
| s04 | 96,1 | 144,2 | 126,9 | *265,6* |
| s05 | 149,6 | 211,9 | *312,7* | — (0/2) |
| s06 | 137,7 | 169,2 | 207,3 | 199,9 |
| s07 | 123,2 | 223,8 | *271,9* | — (0/2) |
| s08 | 110,6 | 171,8 | 228,3 | *541,9* |
| s09 | 122,7 | 169,8 | 166,7 | — (0/2) |
| s10 | 122,1 | 199,1 | — (1/2) | *296,6* |

**Lectura por variante:**
- **Sin cámara frontal (B):** completa los 10 mundos dentro de 240 s, pero el tiempo medio sube de 129,9 s a 186,0 s (+43 %). La ida se alarga un 38 % y la vuelta
  un 65 % (40,1 → 66,2 s), sobre todo por la velocidad: sin cámara el crucero es fijo de 4,0 rad/s; con ella llega a 8,0 rad/s. Las víctimas se siguen viendo con la
  esférica. Peor caso s03: 225,1 s (14,9 s de margen).
- **Sin cámara esférica (C):** el mapa solo recibe lo que tocan los IR. La ida media sube un 79 % (s03: 99,5 → 186,5 s; s07: 54,9 → 159,8 s) y los casi-contactos
  se multiplican por 7,6. Solo 6/10 dentro de 240 s. En s10 vuelve con 1/2: la frontal ve las dos víctimas desde lejos, pero durante el giro de la primera no
  vuelve a ver la segunda (a 0,94 m) y la regla de duplicados la descarta; luego la ve 117 veces en el barrido, pero el descarte es permanente.
- **Sin mapa (D):** navega como un Bug. Solo 6/10 completas y 2/10 dentro de 240 s (s01 y s06). Falla en s03 (2/2, pero se agota el tiempo en la vuelta tras 4
  líneas), s05 y s07 (llega al fondo a los 373,2 y 427,6 s y se agota el tiempo sin rescatar) y s09 (da vueltas entre z ≈ 2 y z ≈ 6 durante 600 s). Son los
  cuatro mundos con callejones sin salida, igual que la versión antigua sin memoria (tabla 2). El recorrido medio casi se duplica (100,5 m frente a 53,5 m).

## 5. Corrección durante las pruebas: rescatar dos veces la misma víctima

En la primera ejecución de s03 sin cámara frontal, el robot dio los dos giros junto a la **misma** víctima y el CSV contaba 2/2. La esférica, desde lejos y sin
la frontal, creó dos candidatas para la misma persona a 0,69 m (por encima de los 0,6 m a partir de los que se fusionan). La segunda recibía observaciones
durante el primer giro (era la misma persona), así que no se descartaba, y como era la única pendiente el robot fue a por ella.

**Regla añadida (aplazamiento):** una candidata a menos de 1 m de una víctima rescatada no se persigue mientras quede barrido por hacer y sobre tiempo
(`kDeferNearRescuedReserveS` = 45 s por encima de la regla de vuelta); el robot sigue barriendo y solo va a por ella si el barrido se agota o falta tiempo.
No se prohíbe del todo porque en s10 las dos víctimas reales están a 0,83 m: si es la misma víctima, el error cuesta ~8 s de giro; si es otra, se rescata.

Con la regla, s03 sin cámara frontal (límite 600 s) sigue barriendo, encuentra la segunda víctima real y completa en 225,1 s. Con el límite real de 240 s a esa
variante ya no le queda tiempo para barrer y sigue yendo a por la candidata dudosa. Las tandas C y D se ejecutaron ya con el código final; de las 20 ejecuciones
de A y B solo tres pasaban por ese código (A-s10, B-s03, B-s10) y se repitieron; en las otras 17 el resultado es idéntico (comprobado repitiendo A-s03: 199,3 s).

## Lectura de los resultados

- **Tiempo:** crece con los obstáculos rodeados en la ida; la vuelta dura 33–70 s porque el mapa ya está construido y el A\* prefiere lo recorrido.
- **Margen:** s03 es el caso crítico (poca luz, más obstáculos, la segunda víctima en el otro extremo del fondo): ida de 99,5 s y vuelta de 69,6 s; total 199,3 s,
  **40,7 s** por debajo del límite. La esférica fue utilizable el 93 % del tiempo (se apaga en los tramos más oscuros). En el resto de mundos el margen es de al menos 90 s.
- **Niebla (s04–s09):** no penaliza de forma apreciable (96,1–149,6 s): la frontal no da nada por libre más allá de 3,5 m y la esférica solo mira hasta 1,6 m.
- **Casi-contactos:** episodios con algún IR < 3 cm o saturado, no choques confirmados.
- **Frenos de emergencia = 0** en las 40 ejecuciones.

## Qué no cubren estos datos

- Repetir una ejecución con el mismo código da exactamente los mismos números (simulación determinista), así que repetirla no mide la variación. Para medirla hay que
  cambiar la pose de salida (la competición la elige al azar) o el mundo. No se ha probado con poses aleatorias.
- Las colisiones no se han medido con un sensor: los casi-contactos son lecturas IR; los rescates y los choques se revisaron visualmente.
- Todas las pruebas usan el GPS sin cuantizar (`resolution -1`, ver [CAMBIOS.md](CAMBIOS.md)); con el GPS original (`resolution 3`) y solo odometría no se ha probado.
- El tiempo es simulado, no tiempo real.

## Cómo reproducir

1. Abrir `worlds/final_scenarioNN.wbt`, compilar y pulsar Play. Sin más `controllerArgs` que `tag=NN` se ejecuta la **misión completa**.
2. Esperar `[RESULT] MISION COMPLETA`. Si se agotan los 240 s aparece `TIEMPO AGOTADO`.
3. El resultado se **añade como una fila** a `results/mission_results.csv`. Si cambia el encabezado, el archivo anterior se aparta como
   `mission_results_old_<fecha>.csv` y se crea uno nuevo. Con `nofiles` no se escribe nada (ni CSV, ni mapa, ni log).

La consola completa de cada ejecución queda en `logs/<mundo>_<fecha-hora>.log` (un archivo por ejecución, no se versiona): ahí están las líneas
`[MISION]`, `[VICTIMAS]` y `[RESULT]` para revisar qué pasó.

Variantes para comparar: `nocam`, `nomap`, `nosphere`. El CSV las distingue (columnas `camera`, `map`, `sphere`) y cada una guarda su propio mapa.
Si una variante necesita más de 240 s, añadir `timelimit=SEG` (p. ej. `timelimit=600`): el límite queda en la columna `time_limit_s`, y `success` = 1
solo indica que volvió antes de ese límite (para el enunciado hay que mirar `time_s` ≤ 240).

## Columnas de `mission_results.csv` (modo `mission`)

| Columna | Significado |
|---|---|
| `when`, `tag`, `camera` | Fecha y hora; mundo (`s01`…`s10`, detectado, o el valor de `tag=`); `cam` o `nocam` |
| `start_x`, `start_z`, `start_yaw_deg` | Pose inicial (yaw 180° = mirando a +Z) |
| `success` | 1 si la vuelta terminó (z ≤ −8,2) antes del límite (`time_limit_s`, 240 por defecto) |
| `victims` | Víctimas rescatadas (0–2) |
| `time_s`, `outbound_s`, `return_s` | Tiempo total, de la ida y de la vuelta (−1 si no hubo vuelta) |
| `victim1_s`, `victim2_s` | Instante en que terminó el giro de cada víctima (−1 si no se rescató) |
| `victim1_dist_m`, `victim2_dist_m` | Distancia estimada robot–víctima al parar (debe ser < 1) |
| `lines_back` | Líneas negras (z = 6…−6) cruzadas en la vuelta |
| `path_m` | Distancia recorrida según los encoders |
| `near_contacts`, `emergency_stops`, `min_ir_m` | Episodios con IR < 3 cm o saturado; episodios del `SafetyGuard`; distancia IR mínima vista |
| `sphere_valid_pct` | % del tiempo que la esférica fue utilizable |
| `map`, `sphere` | Variante: `map`/`nomap` y `sphere`/`nosphere` |
| `time_limit_s` | Límite de la misión usado (240 por defecto; otro valor con `timelimit=SEG`) |

## Mapas ASCII (`results/map_<mundo>_mission[_nocam][_nomap][_nosphere].txt`)

Dibujo de 100 × 200 caracteres (10 cm por carácter), con **arriba = +z** (víctimas) e **izquierda = −x**. La misión guarda `map_<mundo>_mission.txt` (con sufijo si es una variante) al terminar
(con la ruta de vuelta como último plan).

| Símbolo | Significado |
|---|---|
| `#` | Celda ocupada (pared exterior fija, obstáculo visto por IR o esférica, víctima rescatada) |
| `.` | Libre, vista por algún rayo |
| `o` | Celda por la que pasó el robot |
| `*` | Último plan A\* calculado |
| ` ` | Desconocido |

Si el archivo se llama `map_s??_...`, el controlador no pudo detectar el mundo; usar `tag=sNN` en `controllerArgs`. Cada ejecución del mismo mundo y variante sobrescribe su mapa.

## Estado

| Prueba | Estado |
|---|---|
| Misión completa con el controlador de entrega, 10 mundos | Hecha (tabla 3): 10/10 misiones y 20/20 víctimas |
| Comparación `nocam` / `nosphere` / `nomap`, 10 mundos | Hecha (sección 4) |
| Rescates (dos víctimas distintas, < 1 m) | Verificados visualmente en Webots en las 40 ejecuciones |
| Poses de salida aleatorias | No probado |
| Colisiones | Sin métrica automática: los casi-contactos son lecturas IR; revisión visual |
| GPS | Decidido: `resolution -1`, documentado en [CAMBIOS.md](CAMBIOS.md). Con `resolution 3` + odometría no se ha probado |
