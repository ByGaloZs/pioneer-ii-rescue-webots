# Resultados

Todos los tiempos son **segundos simulados** y todas las cifras salen de ejecuciones en **Webots**.
La carpeta `results/` no se versiona (solo `.gitkeep`): los CSV y mapas se generan al ejecutar y son los de la copia de trabajo de quien ejecuta. La tabla de la misión
completa (sección 3) sale de `results/mission_results.csv` de la última tanda; las de la ida (secciones 1 y 2) son **resúmenes copiados de ejecuciones anteriores** y
no se pueden verificar con los archivos actuales. Para comprobarlas hay que repetir las pruebas.

## Qué está validado

| Prueba | Estado |
|---|---|
| Ida con mapa + esférica + cámara, 10 mundos | **10/10** (tabla 1, histórica: modo `navigate`, ya retirado) |
| Ida sin esférica (solo IR + cámara), 10 mundos | Hecha antes de la esférica (tabla 2, histórica) |
| **Misión completa** (ida + víctimas + vuelta), 10 mundos | **10/10 misiones completas**: vuelta a tiempo y **20/20 víctimas**, verificadas contra la posición real (tabla 3) |

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

## 3. Misión completa en Webots

Modo `mission` (predeterminado), un intento por mundo, tanda del 17-11-2026 (17:00), con el `Pioneer2` de Webots (los mundos usan ahora `Pioneer2_Robot`, que con
los IR delanteros a 0,4 m es idéntico; ver [CAMBIOS.md](CAMBIOS.md)). Origen: `results/mission_results.csv` y la consola de cada ejecución en `logs/`.

**Dos medidas distintas:** *Vuelta a tiempo* (columna `success` del CSV) solo indica que el robot volvió a la salida (z ≤ −8,2) antes de 240 s, sin mirar las víctimas;
*Víctimas* (columna `victims`) cuenta las rescatadas. Una **misión completa** exige las dos cosas: vuelta a tiempo y 2/2 víctimas.

| Mundo | Inicio (x, z) | Vuelta a tiempo | Víctimas | Total (s) | Ida (s) | Víctima 1 (s) | Víctima 2 (s) | Vuelta (s) | Dist. **real** robot–víctima en el giro (m) | Líneas de vuelta | Casi-contactos |
|---|---|---|---|---|---|---|---|---|---|---|---|
| s01 | (3,00, −9,00) | sí | 2/2 | 98,5 | 33,2 | 42,9 | 63,3 | 35,2 | 0,79 / 0,79 | 7 | 2 |
| s02 | (0,00, −9,05) | sí | 2/2 | 134,0 | 67,4 | 74,5 | 90,4 | 43,5 | 0,83 / 0,78 | 7 | 17 |
| s03 | (0,00, −9,05) | sí | 2/2 | 199,3 | 99,5 | 107,3 | 129,7 | 69,6 | 0,69 / 0,80 | 7 | 49 |
| s04 | (0,00, −9,05) | sí | 2/2 | 96,1 | 41,2 | 50,3 | 63,0 | 33,1 | 0,73 / 0,79 | 7 | 7 |
| s05 | (0,00, −9,05) | sí | 2/2 | 149,6 | 65,4 | 74,6 | 107,2 | 42,3 | 0,77 / 0,80 | 7 | 0 |
| s06 | (0,00, −9,05) | sí | 2/2 | 137,7 | 63,6 | 72,5 | 103,5 | 34,2 | 0,78 / 0,80 | 7 | 16 |
| s07 | (0,00, −9,05) | sí | 2/2 | 123,2 | 54,9 | 61,8 | 84,3 | 38,9 | 0,72 / 0,79 | 7 | 19 |
| s08 | (0,00, −9,05) | sí | 2/2 | 110,6 | 49,7 | 60,3 | 76,0 | 34,6 | 0,80 / 0,79 | 7 | 0 |
| s09 | (0,00, −9,05) | sí | 2/2 | 122,7 | 39,6 | 59,8 | 87,0 | 35,6 | 0,79 / 0,79 | 7 | 4 |
| s10 | (0,00, −9,05) | sí | 2/2 | 121,9 | 65,5 | 75,4 | 86,0 | 35,8 | 0,80 / 0,83 | 7 | 21 |

En esta tanda s01 salía de (3, −9); la salida ha vuelto a la original (0, −9,05) en todos los mundos, así que s01 se repetirá en la tanda final.

Resumen: vuelta a tiempo en 10/10 · **20/20 víctimas** · **misión completa en 10/10** · tiempo total medio 129,4 s (mín. 96,1 s en s04, máx. 199,3 s en s03).

**Verificación de los rescates.** Para cada giro de 360° se tomó la posición del robot de la consola (`logs/`) y se midió su distancia a la víctima **real** (posición
del `.wbt`) a la que corresponde el rescate. Los 20 giros se hicieron a **0,69–0,83 m** de una víctima real (objetivo < 1 m), y en cada mundo las dos víctimas rescatadas
son distintas. La posición estimada de cada víctima cayó a ≤ 0,11 m de la real, salvo la primera de s03 (0,32 m). Los casi-contactos son lecturas IR < 3 cm o
saturadas, no choques confirmados.

**Cambios respecto a la tanda anterior (13-11 / 17-11, 17/20 víctimas):**
- **s03** (0/2 → 2/2): el mundo tenía guardado `nosphere` de una prueba comparativa; sin la esférica la ida tardaba 186,5 s y no quedaba tiempo para buscar. Con ella,
  99,5 s. Además la misma víctima generaba dos candidatas (una desde lejos con error) y se "rescataba" dos veces; ahora las candidatas junto a una rescatada van al
  final de la cola y el robot va a por la otra víctima.
- **s09** (1/2 → 2/2): la víctima de la esquina (−4,21; 9,46) está en una zona muy iluminada; el umbral de verde (1,15 × brillo del suelo) pasaba de 255 y ninguna
  cámara podía verla. El umbral tiene ahora un tope (200). El barrido llega más a las esquinas (x = ±3,8) y da una vuelta en cada extremo.
- **s10** (2/2 aparente → 2/2 real): las dos víctimas están a 0,83 m; un "fantasma" entre ellas hacía que el segundo giro se hiciera a 1,30 m de la víctima 2. Ahora
  una candidata a menos de 0,6 m de una rescatada se descarta y, entre las cercanas, se elige la más alejada de la rescatada.

## Lectura de los resultados

- **Tiempo:** crece con los obstáculos rodeados en la ida; la vuelta suele durar 33–44 s porque el mapa ya está construido y el A\* prefiere lo recorrido.
- **Margen:** s03 sigue siendo el caso crítico: ida de 99,5 s, la segunda víctima en el otro extremo del fondo y vuelta de 69,6 s; total 199,3 s, **40,7 s** por debajo
  del límite. La esférica fue utilizable el 93 % del tiempo (se apaga en los tramos más oscuros). En el resto de mundos el margen es de al menos 90 s.
- **Casi-contactos:** cuentan episodios con algún IR < 3 cm o saturado. `min_ir_m` rondó 0,10 m, así que reflejan la bandera de saturación y no un roce confirmado.
  Si hubo colisiones hay que revisarlo visualmente en Webots.
- **Frenos de emergencia = 0** en todas las pruebas.

## Qué no cubren estos datos

- Repetir la tanda con el mismo código da exactamente los mismos números (simulación determinista), así que repetirla no mide la variación. Para medirla hay que
  cambiar la pose de salida (por ejemplo, arrastrando el robot en Webots antes de pulsar Play) o el mundo.
- Las colisiones no se han medido (los casi-contactos son lecturas IR, no choques).
- No hay todavía comparación `nocam`/`nomap` con la misión completa.
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

## Pendiente

| Prueba | Estado |
|---|---|
| Comparación `nocam` / `nomap` / `nosphere` con la misión completa, 10 mundos | Pendiente (tanda final) |
| Tanda final con `Pioneer2_Robot` y la salida original en s01 | Pendiente |
| Misión completa en Webots, 10 mundos | Hecha (tabla 3): 10/10 misiones y 20/20 víctimas |
| Distancia real robot–víctima en cada giro | Hecha (tabla 3): 0,69–0,83 m |
| Colisiones | Pendiente: los casi-contactos son lecturas IR; hay que revisarlo visualmente en Webots |
| GPS | Decidido: `resolution -1`, documentado en [CAMBIOS.md](CAMBIOS.md). Con `resolution 3` + odometría no se ha probado |
