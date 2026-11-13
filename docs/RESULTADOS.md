# Resultados

Todos los tiempos son **segundos simulados** y todas las cifras salen de ejecuciones en **Webots**.
La carpeta `results/` no se versiona (solo `.gitkeep`): los CSV y mapas se generan al ejecutar y son los de la copia de trabajo de quien ejecuta. La tabla de la misión
completa (sección 3) sale de `results/mission_results.csv` de la última tanda; las de la ida (secciones 1 y 2) son **resúmenes copiados de ejecuciones anteriores** y
no se pueden verificar con los archivos actuales. Para comprobarlas hay que repetir las pruebas.

## Qué está validado

| Prueba | Estado |
|---|---|
| Ida con mapa + esférica + cámara, 10 mundos | **10/10** (tabla 1) |
| Ida sin esférica (solo IR + cámara), 10 mundos | Hecha antes de la esférica (tabla 2, histórica) |
| **Misión completa** (ida + víctimas + vuelta), 10 mundos | **10/10 vueltas** y **17/20 víctimas** rescatadas (tabla 3) |

## 1. Ida en Webots, versión actual

Configuración: cámara frontal + mapa + A\* + cámara esférica en el mapa, valores por defecto, un intento por mundo, modo `navigate`.
Origen: `results/outbound_results.csv` (filas `sphere`).

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

## 3. Misión completa en Webots

Modo `mission` (predeterminado), un intento por mundo. Origen: `results/mission_results.csv`.

| Mundo | Inicio (x, z) | Vuelta ok | Víctimas | Total (s) | Ida (s) | Víctima 1 (s) | Víctima 2 (s) | Vuelta (s) | Dist. final a víctimas (m) | Líneas de vuelta | Casi-contactos |
|---|---|---|---|---|---|---|---|---|---|---|---|
| s01 | (3,00, −9,00) | sí | 2/2 | 98,5 | 33,2 | 42,9 | 63,3 | 35,2 | 0,85 / 0,84 | 7 | 2 |
| s02 | (0,00, −9,05) | sí | 2/2 | 134,0 | 67,4 | 74,5 | 90,4 | 43,5 | 0,85 / 0,85 | 7 | 17 |
| s03 | (0,00, −9,05) | sí | 0/2 | 236,0 | 186,5 | — | — | 49,5 | — / — | 7 | 143 |
| s04 | (0,00, −9,05) | sí | 2/2 | 96,1 | 41,2 | 50,3 | 63,0 | 33,1 | 0,70 / 0,85 | 7 | 7 |
| s05 | (0,00, −9,05) | sí | 2/2 | 148,2 | 65,4 | 74,6 | 106,7 | 41,5 | 0,85 / 0,85 | 7 | 3 |
| s06 | (0,00, −9,05) | sí | 2/2 | 135,8 | 63,6 | 72,5 | 100,9 | 34,9 | 0,85 / 0,85 | 7 | 21 |
| s07 | (0,00, −9,05) | sí | 2/2 | 123,2 | 54,9 | 61,8 | 84,3 | 38,9 | 0,70 / 0,85 | 7 | 19 |
| s08 | (0,00, −9,05) | sí | 2/2 | 110,6 | 49,7 | 60,3 | 76,0 | 34,6 | 0,85 / 0,85 | 7 | 0 |
| s09 | (0,00, −9,05) | sí | 1/2 | 106,0 | 39,6 | 55,7 | — | 30,7 | 0,85 / — | 7 | 2 |
| s10 | (0,00, −9,05) | sí | 2/2 | 118,7 | 65,5 | 75,4 | 83,2 | 35,5 | 0,85 / 0,85 | 7 | 19 |

Resumen: vuelta completada en 10/10 · 17/20 víctimas (2/2 en 8 mundos, 1/2 en s09, 0/2 en s03) · tiempo total medio 130,7 s (mín. 96,1 s en s04, máx. 236,0 s en s03).

La distancia final es la **estimada** por el controlador entre el robot y el centro de la víctima al parar (objetivo: < 1 m); no se ha contrastado con la posición real de las
víctimas. Los casi-contactos son lecturas IR < 3 cm o saturadas, no choques confirmados.

## Lectura de los resultados

- **Tiempo:** crece con los obstáculos rodeados en la ida; la vuelta es parecida entre mundos (31–50 s) porque el mapa ya está construido y el A\* prefiere lo recorrido.
- **Margen:** s03 es el caso crítico: la ida tardó 186,5 s, no se rescató ninguna víctima, el total fue 236,0 s y quedaron solo 4 s sobre el límite de 240 s; además la esférica
  se marcó utilizable 0 % del tiempo y hubo 143 casi-contactos. En el resto de mundos el margen es de al menos 91 s.
- **s09:** se rescató 1/2 víctimas; el tiempo total (106,0 s) deja margen, así que el fallo está en la búsqueda o la detección de la segunda, no en el reloj.
- **Casi-contactos:** cuentan episodios con algún IR < 3 cm o saturado. `min_ir_m` rondó 0,10 m, así que reflejan la bandera de saturación y no un roce confirmado.
  Si hubo colisiones hay que revisarlo visualmente en Webots.
- **Frenos de emergencia = 0** en todas las pruebas.

## Qué no cubren estos datos

- La misión completa se ejecutó una vez por mundo; falta medir la variación entre ejecuciones y con otras poses de salida.
- No se ha medido la distancia real a las víctimas (solo la estimada) ni las colisiones.
- No hay todavía comparación `nocam`/`nomap` con la misión completa, ni pruebas con GPS cuantizado (pendiente conocido).
- El tiempo es simulado, no tiempo real.

## Cómo reproducir

1. Abrir `worlds/final_scenarioNN.wbt`, compilar y pulsar Play. Sin `controllerArgs` se ejecuta la **misión completa**; con `navigate` solo la ida.
2. Esperar `[RESULT] MISION COMPLETA` (misión) o `[RESULT] IDA COMPLETADA` (`navigate`). Si se agotan los 240 s aparece `TIEMPO AGOTADO`.
3. El resultado se **añade como una fila** al CSV del modo (misión: `results/mission_results.csv`; ida: `results/outbound_results.csv`). Si cambia el encabezado, el archivo
   anterior se aparta como `*_old_<fecha>.csv` y se crea uno nuevo.

Variantes para comparar: `nocam`, `nomap`, `nosphere` y, opcionalmente, `tag=s03-nocam` para distinguir las filas.

## Columnas de `mission_results.csv` (modo `mission`)

| Columna | Significado |
|---|---|
| `when`, `tag`, `camera` | Fecha y hora; mundo (`s01`…`s10`, detectado, o el valor de `tag=`); `cam` o `nocam` |
| `start_x`, `start_z`, `start_yaw_deg` | Pose inicial (yaw 180° = mirando a +Z) |
| `success` | 1 si la vuelta terminó (z ≤ −8,2) antes de 240 s |
| `victims` | Víctimas rescatadas (0–2) |
| `time_s`, `outbound_s`, `return_s` | Tiempo total, de la ida y de la vuelta (−1 si no hubo vuelta) |
| `victim1_s`, `victim2_s` | Instante en que terminó el giro de cada víctima (−1 si no se rescató) |
| `victim1_dist_m`, `victim2_dist_m` | Distancia estimada robot–víctima al parar (debe ser < 1) |
| `lines_back` | Líneas negras (z = 6…−6) cruzadas en la vuelta |
| `path_m` | Distancia recorrida según los encoders |
| `near_contacts`, `emergency_stops`, `min_ir_m` | Episodios con IR < 3 cm o saturado; episodios del `SafetyGuard`; distancia IR mínima vista |
| `sphere_valid_pct` | % del tiempo que la esférica fue utilizable |

## Columnas de `outbound_results.csv` (modo `navigate`)

| Columna | Significado |
|---|---|
| `when`, `tag`, `camera`, `start_x`, `start_z`, `start_yaw_deg` | Como arriba |
| `success` / `time_s` | 1 si cruzó la línea final (z ≥ 8,2) / tiempo simulado |
| `max_z` / `lines` | Máximo z alcanzado / líneas cruzadas de 8 (7 negras de z = −6 a 6 y la amarilla en z = 8) |
| `path_m` | Distancia recorrida según los encoders |
| `obstacles` | Veces que se empezó a rodear un obstáculo (`FOLLOW_WALL`) |
| `side_switches` | Medias vueltas para rodear por el otro lado o por plan (callejones) |
| `stuck_recoveries` | Veces que se activó `RECOVER` por atasco |
| `emergency_stops` / `near_contacts` / `min_ir_m` | Como arriba |
| `map` | `map` o `nomap` |
| `cam_valid_pct` | % de fotogramas con la cámara frontal utilizable (0 si no se usó) |
| `replans` | Número de veces que se ejecutó el A\* |
| `sphere` / `sphere_valid_pct` | `sphere` o `nosphere` / % del tiempo que la esférica fue utilizable |

## Mapas ASCII (`results/map_<mundo>_<cam|nocam|mission>.txt`)

Dibujo de 100 × 200 caracteres (10 cm por carácter), con **arriba = +z** (víctimas) e **izquierda = −x**. La misión guarda `map_<mundo>_mission.txt` al terminar
(con la ruta de vuelta como último plan).

| Símbolo | Significado |
|---|---|
| `#` | Celda ocupada (pared exterior fija, obstáculo visto por IR o esférica, víctima rescatada) |
| `.` | Libre, vista por algún rayo |
| `o` | Celda por la que pasó el robot |
| `*` | Último plan A\* calculado |
| ` ` | Desconocido |

Si el archivo se llama `map_s??_...`, el controlador no pudo detectar el mundo; usar `tag=sNN` en `controllerArgs`. Cada ejecución del mismo mundo y modo sobrescribe su mapa.

## Pendiente

| Prueba | Estado |
|---|---|
| Ida con cámara + mapa + esférica, 10 mundos | Hecho (tabla 1) |
| Ida sin cámara (`nocam`) y sin mapa (`nomap`), 10 mundos | Pendiente (comparación) |
| Misión completa en Webots, 10 mundos | Hecha (tabla 3); pendiente revisar s03 (0/2) y s09 (1/2) |
| Medir colisiones y distancia real a las víctimas | Pendiente |
| Evaluación con GPS cuantizado | Pendiente |
