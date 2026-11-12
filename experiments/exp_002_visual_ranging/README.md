# EXP-002 — Visual radial ranging

**Estado:** `partially_closed` para consistencia geométrica simulada en escenario 01. **Pendiente:** metrología física independiente, validación completa de extrínsecos y generalización a otros escenarios. **Hipótesis:** la cámara esférica puede producir observaciones angulares útiles como pseudo-LiDAR, con límites de alcance e incertidumbre explícitos. **Orientación de trabajo seleccionada:** `camera_s` inclinada −30° (`rotation 1 0 0 -0.5236`). No se compararán otras inclinaciones salvo que se reabra el experimento.

**Escenario inicial común:** una copia de `worlds/final_scenario01.wbt`, guardada como `worlds/experiment_test_scenario01.wbt`. Se usa el mismo mundo de prueba para todos los experimentos en la primera fase; se regenera esa copia con los parámetros del ensayo activo. Los mundos canónicos `final_scenario*.wbt` no se modifican.

## Arquitectura compartida de pruebas

Todos los experimentos que requieran ejecución C++ en Webots usan **un único** `controllers/experiment_test_controller/`, con su propio punto de entrada y `Makefile`. Se edita/configura según el experimento activo; no se crea un controlador diferente por experimento y no se añaden loggers de prueba a `fool_controller`. El mundo compartido selecciona ese controlador y recibe una etiqueta explícita `experiment=002;trial=<id>`.

```bash
make -C controllers/experiment_test_controller
```

Webots debe estar disponible en `WEBOTS_HOME`. El logger de esta fase solo escribe en `experiments/exp_002_visual_ranging/results/`; requiere un identificador válido y falla si no encuentra esa ruta. Las corridas de otros experimentos usarán el mismo controlador y su propia raíz de resultados, según la implementación activa.

### Regenerar el único mundo de pruebas

El generador parte siempre de `final_scenario01.wbt`, conserva el escenario fuente y modifica solo la pose/GPS/cámara requeridos por el ensayo, el controlador y la etiqueta. Sin `--overwrite`, rechaza cambios respecto al archivo existente. Cada ensayo mantiene su configuración y etiqueta en el registro experimental antes de volver a generar la misma copia.

```bash
# Primera captura P1 a −30°; el mismo WBT se reconfigura luego para P2/P3
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id calib01_tilt30_p1_r01 --z -9.0 --strip-camera-debug --overwrite

# Siguientes corridas, una a la vez, sobre el mismo archivo
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id calib01_tilt30_p2_r01 --z -8.8 --strip-camera-debug --overwrite
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id calib01_tilt30_p3_r01 --z -8.3 --strip-camera-debug --overwrite
```

### Repetición controlada P1/P2/P3

Para caracterizar la variabilidad, repetir cada pose tres veces. Cambiar `r01` por `r02` y `r03` después de guardar cada corrida; cada repetición regenera **el mismo WBT**, abre/reinicia Webots y deja una carpeta `run_...` independiente.

| Pose | Z robot (m) | Trial IDs |
| --- | ---: | --- |
| P1 | −9,0 | `calib01_tilt30_p1_r01`–`calib01_tilt30_p1_r03` |
| P2 | −8,8 | `calib01_tilt30_p2_r01`–`calib01_tilt30_p2_r03` |
| P3 | −8,3 | `calib01_tilt30_p3_r01`–`calib01_tilt30_p3_r03` |

Ejemplo para P1 repetición 1:

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id calib01_tilt30_p1_r01 --z -9.0 --strip-camera-debug --overwrite
```

Tras guardar las nueve corridas, reconstruir el índice offline desde GPS, analizar únicamente las corridas de pared y regenerar la comparación:

```bash
python3 experiments/exp_002_visual_ranging/src/build_run_index.py
python3 experiments/exp_002_visual_ranging/src/analyze_runs.py \
  --run-index experiments/exp_002_visual_ranging/results/run_index.csv \
  --mask-config experiments/exp_002_visual_ranging/config/self_occlusion_tilt30.json \
  --output-dir experiments/exp_002_visual_ranging/results/analysis_v3_tilt30
python3 experiments/exp_002_visual_ranging/src/compare_wall_ranges.py
```

El indexador asigna P1/P2/P3 mediante `gps_z_m`, separa repeticiones por `trial_id` y orden de corrida, y excluye rumbos distintos de cero, como la toma del pasillo. Si varias corridas reutilizaron el mismo `trial_id`, numera esas repeticiones offline por orden temporal y deja una nota; no modifica los CSV brutos.

Abrir `worlds/experiment_test_scenario01.wbt`, ejecutar y guardar cada corrida antes de regenerar el mundo para el siguiente ensayo. Los parámetros físicos se registran fuera del controlador; este no lee geometría WBT ni distancias de referencia. La misma copia común se usará primero para EXP-001–010 durante desarrollo; la matriz de escenarios adicionales se ejecutará después de validar el comportamiento base, según `docs/test_plan.md`.

## V3 — observaciones angulares y geometría (prototipo offline)

**Estado:** prototipo implementado, sin validación métrica en Webots ni integración con el mapa. La cámara esférica actual tiene resolución 160×160 y FOV horizontal aproximado de π rad (**180°, no 360°**). Medir radios desde el centro de la imagen no corresponde a un barrido LiDAR.

Módulos principales:
- `src/spherical_geometry.py`: rayos por píxel, cámara con tilt nominal −30°, altura 0,44 m y desplazamiento de montaje. Calcula intersecciones candidatas con suelo plano y descarta rayos al cielo/horizonte.
- `src/angular_observations.py`: sectores por columnas, segmentación HSV conservadora y transición **suelo visible → obstáculo visible**. Los píxeles de autooclusión quedan desconocidos. Si la máscara solo tapa píxeles de campo cercano debajo de una secuencia visible de suelo/contacto, se conserva el rango del contacto lejano y se marca el campo cercano como desconocido; si la máscara corta la transición, el sector completo queda `unknown`. Nunca se infiere espacio libre.
- `src/segment_image.py`: V3 angular por defecto; el algoritmo circular antiguo solo está disponible como `--mode radial-legacy` para reproducibilidad, no para métricas.
- `src/analyze_runs.py`: analiza corridas etiquetadas y produce salidas V3 separadas, sin modificar los CSV originales.
- `src/build_run_index.py`: reconstruye el índice offline desde GPS; corrige poses mal etiquetadas sin tocar los CSV brutos y excluye corridas con otro rumbo.

`config/self_occlusion_tilt30.json` contiene la máscara de carrocería revisada en cuatro capturas de pared y una del pasillo. Enmascara el cuerpo blanco y la pieza amarilla del robot; deja visible la franja gris candidata a suelo. Esta verificación cubre la silueta en imagen, **no** valida distancias ni zonas libres. `classify_pixels` ahora también puede sembrar componentes grises conectados al borde de la máscara, ya que el robot ocupa la parte inferior de estas imágenes.

El eje óptico asumido es `-Z`; píxeles a la derecha se tratan como azimut positivo. El sentido y el montaje deben verificarse con un marcador lateral en Webots antes de usar rumbos en el mapa. La intersección suelo-rayo solo es candidata para contacto visible sobre suelo plano, no distancia a cualquier superficie. La confianza 0,5 es heurística y el alcance [0,2; 4,0] m es un filtro provisional, no un resultado medido.

## Ejecución del piloto y análisis

1. Compilar `experiment_test_controller` y abrir la copia única de prueba en Webots.
2. Ejecutar una sola configuración etiquetada, verificar en consola `trial_id`, pose, cámara y captura guardada.
3. Inspeccionar primero el JPEG bruto. Registrar suelo, pared, carrocería, condiciones y fallos; no modificar datos de una corrida anterior.
4. Para la siguiente pose/inclinación, regenerar **el mismo** `worlds/experiment_test_scenario01.wbt` con `--overwrite`, abrir/reiniciar y guardar la corrida nueva.
5. Comparar los candidatos con referencias independientes antes de aceptar una calibración. La comparación con el plano WBT descrita abajo es solo una comprobación geométrica preliminar.

## Siguiente fase — geometría y referencias independientes

Las repeticiones P1/P2/P3 existentes repiten renders idénticos y no miden variabilidad estadística. El piloto de cierre usa el mismo mundo compartido y dos poses de escenario 01. La geometría del WBT se emplea únicamente en el análisis offline como referencia del simulador; nunca se entrega al controlador. Esto permite medir consistencia simulada, aunque no sustituye metrología física independiente.

### Pose anterior en escenario 01 — pared a la derecha

Se configura la copia compartida con el robot en `(x,z)=(-2.0,-3.58418) m`, yaw `−0.785398 rad` (−45°), cámara a −30° y etiqueta `geom01_tilt30_right_r02`. La pared `solid2` queda en el cuadrante delantero derecho. El logger guarda una captura al primer paso y otra aproximadamente al segundo simulado, para comprobar el desplazamiento de pose, y después finaliza. La copia se genera desde `final_scenario01.wbt`; solo ese test WBT se actualiza:

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --x -2.0 --z -3.58418 --yaw -0.785398 \
  --trial-id geom01_tilt30_right_r02 --strip-camera-debug --overwrite
WEBOTS_HOME=/usr/local/webots make -C controllers/experiment_test_controller
```

Al abrir `worlds/experiment_test_scenario01.wbt` y dar Play, el controlador mantiene motores detenidos y guarda ambas capturas. Luego se analiza esa corrida con `analyze_runs.py`. `geom01_*` queda reservado a la validación geométrica y se acepta en el análisis offline; no se etiqueta como P1/P2/P3 para no contaminar el índice de calibración anterior. Esta primera toma comprueba la visibilidad/azimut de una pared lateral y la cadena de captura. El rango se compara offline con la geometría de `solid2` como **referencia de verdad del simulador**, no como metrología física independiente.

#### Resultados de la pose anterior `geom01_tilt30_right_r02`

| Run | Webots | Robot según WBT | GPS registrado (0,032 s y 1,056 s) | Capturas |
| --- | --- | --- | --- | ---: |
| `run_20261111_195735` | R2020b | `x=-2, z=-3,58418, yaw=-0,785398` | `x=-2, z=-3,216276, yaw=-0,785398` en ambas | 2 |

Las dos imágenes son idénticas y la pose GPS no varía entre las capturas. El GPS difiere `+0,367904 m` en Z de la traslación configurada en WBT; la diferencia ya está presente en la primera muestra (`t=0,032 s`), por lo que no se observa deriva durante el intervalo de un segundo. La causa/origen de esa discrepancia entre marcos queda pendiente; usar la pose GPS registrada al comparar esta corrida.

V3 obtiene en ambas imágenes los mismos candidatos visibles: sector 9 (`1,3652 m`, bearing `+18,46°`) y sector 10 (`1,3091 m`, bearing `+30,24°`). Ambos conservan `near_field_occluded=true`: los píxeles cercanos enmascarados siguen desconocidos, pero ya no invalidan la transición suelo-contacto visible. El resto queda desconocido; no se infiere espacio libre.

Como comprobación geométrica condicional, el sector 10 se comparó con la cara de `solid2` mediante una intersección de rayo/plano usando la pose GPS registrada y el bearing del sector: referencia simulada `1,24474 m`, candidato `1,3091 m`, error firmado `+0,06436 m`. Esta comprobación depende del bearing estimado y de la asociación al plano de `solid2`; **no es una referencia física independiente ni una MAE validada**. Archivos: `results/analysis_geom01/run_20261111_195735/` y `results/independent_validation_geom01/`. Webots R2020b; HEAD `cad6ae7` (árbol de trabajo con cambios locales).

La corrida `run_20261111_195735` quedó supersedida para esa comparación cuando se recargó el WBT y se alinearon la traslación configurada y el GPS. Su comparación de `+0,06436 m` se conserva como antecedente, pero no se usa como referencia principal del cierre parcial.

### Pose activa en escenario 01 — pared al frente

La pose nueva registrada fue `(x,z)=(-1.3,4.0) m`, yaw `+1.5708 rad`. La corrida `run_20261111_203724` conserva la etiqueta antigua `geom01_tilt30_right_r02` en sus CSV; su GPS/compás permiten identificarla, pero no se deben editar los logs originales. La copia compartida queda configurada con etiqueta descriptiva nueva para las siguientes corridas:

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --x -1.3 --z 4.0 --yaw 1.5708 \
  --trial-id geom01_tilt30_front_r01 --strip-camera-debug --overwrite
```

El analizador acepta `geom01_tilt30_front_rNN`. Las capturas futuras de esta pose usarán esa etiqueta; la carpeta histórica conserva el `trial_id` recibido originalmente y `results/geometry_run_index_scenario01.csv` guarda la corrección offline sin editar los CSV brutos.

### Cierre parcial — escenario 01

Se acepta como **consistencia geométrica simulada** que el modelo V3 produce rangos próximos a las caras de obstáculo visibles en dos ubicaciones de escenario 01:

| Corrida / referencia | Sectores comparados | MAE (m) | RMSE (m) | Error máximo abs. (m) |
| --- | ---: | ---: | ---: | ---: |
| `run_20261111_201357`, cara de `solid2` | 1 | 0,0198 | 0,0198 | 0,0198 |
| `run_20261111_203724`, cara de `solid10` | 7 | 0,0269 | 0,0299 | 0,0517 |

La segunda pose registra siete candidatos entre `0,7180` y `0,9861 m`, con bearings de `−40,72°` a `+43,96°`. Las referencias se calcularon offline desde la geometría y la pose GPS/yaw; en el contraste por sectores el plano se intersecta usando el bearing estimado, así que el resultado **no valida azimut de forma independiente**. Los errores son de consistencia con el simulador, no mediciones físicas. Las superficies enmascaradas siguen desconocidas; no se infiere espacio libre.

Los dos frames de cada corrida son idénticos byte por byte. `run_20261111_203525` también repite la misma imagen/pose que `run_20261111_201357`; se registra como repetición del render, no como observación estadística independiente. El índice y los archivos reproducibles están en `results/geometry_run_index_scenario01.csv`, `results/geometry_reference_scenario01_*.csv`, `results/analysis_geom01/` y `results/independent_validation_geom01_*/`.

**Conclusión parcial:** `accept` para la comprobación de rango visual frente a geometría de Webots en estas poses de escenario 01. `not_measured` para exactitud física independiente, calibración absoluta del azimut y robustez fuera de escenario 01. EXP-003 puede comenzar sobre el mismo escenario 01; no se hacen afirmaciones de generalización a otros mundos.

### 1. Validar azimut y alineación lateral

1. Mantener `camera_s` a −30° y el robot detenido. Presentar un marcador visual pequeño, identificable y no confundible con suelo/pared, a la izquierda y a la derecha del eje delantero. El marcador debe estar en el FOV y separado de la silueta del robot.
2. Medir independientemente su bearing horizontal respecto al eje delantero del robot (por ejemplo, levantando las coordenadas laterales y longitudinales con una regla/escuadra y calculando `atan2(x, -z)`). No tomar el bearing del `.wbt`, de la proyección V3 ni del índice de sector como referencia.
3. Capturar cada posición con el logger y anotar el `run_id`, `frame_file`, coordenadas del centro del marcador en píxeles, bearing medido y método de levantamiento. Usar corridas distintas para izquierda/derecha; si se modifica un objetivo, regenerar únicamente la copia compartida y conservar cada corrida.
4. Incluir ambas direcciones y más de un bearing por lado antes de concluir que el signo, FOV y centrado están bien. El análisis devuelve sesgo, MAE y RMSE angular; definir y registrar una tolerancia antes de revisar resultados, no después.

### 2. Contrastar rango con contacto visible

1. Colocar una pared/placa plana con un borde de contacto suelo-objeto claramente visible, sin que el robot tape el suelo cercano del sector medido. Mantener el suelo y objetivo dentro de condiciones de iluminación registradas.
2. Para cada sector y captura seleccionados, medir independientemente la distancia horizontal desde el origen óptico de la cámara hasta el punto de contacto visible. Registrar instrumento, referencia del origen (altura/desplazamiento de cámara medidos) e incertidumbre/resolución del instrumento en `reference_method`/`notes`. La distancia no se calcula a partir de la pose del WBT ni del plano `solid1`.
3. Hacer primero capturas para ajustar/revisar el modelo y conservar capturas separadas como validación fuera de muestra; no ajustar con las mismas filas que luego se reportan como evaluación.
4. El comparador excluye automáticamente sectores cuyo contacto es desconocido/está tapado. Un contacto pared-suelo visible se compara aunque haya píxeles más cercanos tapados; `near_field_occluded=true` se conserva como dato y no convierte esa zona cercana en libre.

El archivo `results/independent_references_template.csv` contiene solo encabezados, no datos inventados. Se puede duplicar dentro de `results/` para registrar lecturas. Los valores `reference_bearing_deg` son relativos al eje delantero del robot; `reference_distance_m` es cámara-origen a contacto suelo-objeto. En filas `bearing` se llenan píxeles X/Y y bearing; en filas `range`, `sector_id` y distancia. `reference_method` debe identificar cómo se levantó la referencia. Cada `run_id` debe corresponder al nombre de su carpeta `run_...`.

Analizar las capturas con la misma máscara validada a −30° y luego comparar referencias:

```bash
python3 experiments/exp_002_visual_ranging/src/analyze_runs.py \
  experiments/exp_002_visual_ranging/results/run_YYYYMMDD_HHMMSS \
  --mask-config experiments/exp_002_visual_ranging/config/self_occlusion_tilt30.json \
  --camera-height-m 0.44 --camera-tilt-rad -0.5236 \
  --camera-offset-x-m 0.025 --camera-offset-z-m -0.05 --camera-yaw-rad 0 \
  --output-dir experiments/exp_002_visual_ranging/results/independent_analysis

python3 experiments/exp_002_visual_ranging/src/validate_independent_references.py \
  --references experiments/exp_002_visual_ranging/results/independent_references.csv \
  --analysis-dir experiments/exp_002_visual_ranging/results/independent_analysis \
  --camera-tilt-rad -0.5236 --camera-yaw-rad 0 \
  --output-dir experiments/exp_002_visual_ranging/results/independent_validation
```

El primer comando acepta las etiquetas `calib01_tilt30_p1[_rNN]` actuales. Repetir `analyze_runs.py` para cada carpeta de corrida indicada en el CSV, siempre con los **mismos parámetros** usados para evaluar; la salida debe quedar como `independent_analysis/<run_id>/v3_sectors.csv`. Los parámetros iniciales anteriores son los nominales del prototipo, no medidas certificadas. Se pueden ajustar con la tanda de ajuste y después congelarlos antes de procesar la tanda fuera de muestra. El yaw de cámara usado por ambos comandos debe ser el mismo. Si el comparador excluye un sector por oclusión/desconocido, corregir la vista/escena y hacer una nueva captura, sin editar el resultado bruto. Revisar `independent_reference_comparison.csv` junto con `independent_reference_summary.json`.

**Criterio de cierre de esta fase:** extrínsecos/signo y rango respaldados por referencias independientes, muestras a ambos lados, datos de ajuste separados de evaluación y contacto medido claramente visible. El espacio cercano oculto sigue desconocido. Fijar los márgenes aceptables antes de medir, con base en la resolución de referencia y el uso previsto. Hasta entonces, no declarar MAE validada ni integrar rangos visuales al mapa. Una vez superada escenario 01, continuar la matriz prevista en 03, 07, 08 y 10.

Las corridas anteriores fueron limpiadas. La primera tanda está registrada en `results/run_index.csv`; los cuatro ensayos de pared y el ensayo orientado al pasillo quedaron en `results/run_.../`. Las imágenes de revisión de máscara están en `results/mask_review_tilt30/` y los sectores V3 en `results/analysis_v3_tilt30/`. Las cinco corridas iniciales se registraron con `trial_id=calib01_tilt30_p1`; el índice las distingue mediante GPS y pose, sin editar los CSV originales.

| Pose/ensayo | Run ID | Sectores candidatos `occupied` | `unknown` |
| --- | --- | ---: | ---: |
| P1, Z=−9,0 m | `run_20261111_151236` | 10 | 6 |
| P2, Z=−8,8 m | `run_20261111_151352` | 9 | 7 |
| Adicional, Z=−8,5 m | `run_20261111_151438` | 6 | 10 |
| P3, Z=−8,3 m | `run_20261111_151728` | 3 | 13 |
| Pasillo, X=−1,5/Z=−8,0 m | `run_20261111_153626` | 2 | 14 |

Todas las observaciones `occupied` llevan `near_field_occluded=true` y confianza heurística 0,5. Indican candidatos de contacto visible con campo cercano desconocido; no son evidencia de suelo transitable ni resultados métricos independientes. En la toma del pasillo, los sectores centrales permanecen `unknown`, así que la abertura visual no se clasifica como zona libre.

### Comparación preliminar con el plano de pared del escenario 01

`src/compare_wall_ranges.py` compara cada rango V3 candidato contra el plano interior de `solid1` en `Z=−9,8 m` (centro WBT `Z=−9,9 m`, espesor 0,2 m). La referencia se calcula desde el origen de la cámara (`GPS Z − 0,05 m`) y se corrige por el rumbo sectorial: `d_ref = (z_cámara − z_pared) / cos(β)`. El pasillo se excluye de esta calibración.

| Pose | Candidatos | Desconocidos | MAE candidato (m) | Error medio con signo (m) |
| --- | ---: | ---: | ---: | ---: |
| P1, Z=−9,0 m | 10 | 6 | 0,0633 | +0,0633 |
| P2, Z=−8,8 m | 9 | 7 | 0,0812 | +0,0812 |
| Adicional, Z=−8,5 m | 6 | 10 | 0,0982 | +0,0982 |
| P3, Z=−8,3 m | 3 | 13 | 0,0641 | +0,0641 |
| **Conjunto** | **28** | **36** | **0,0766** | **+0,0766** |

Los 28 candidatos incluidos tienen `near_field_occluded=true`; los errores son positivos en todas las comparaciones. Son **errores de consistencia con un plano derivado offline del WBT**, no una MAE validada con medición física independiente. La marca significa que los píxeles más cercanos siguen desconocidos; no invalida por sí sola un contacto pared-suelo más lejano que sí se ve. Además, el rumbo sectorial depende del modelo de cámara aún pendiente de verificar con un marcador lateral. No usar estos valores para aceptar navegación ni inferir espacio libre. Los CSV reproducibles están en `results/wall_range_comparison_tilt30.csv` y `results/wall_range_summary_tilt30.csv`; el registro original de cada corrida conserva su etiqueta recibida y `run_index.csv` aporta la asociación offline de poses.

### Repeticiones registradas

El indexador incorporó las corridas posteriores desde sus Z GPS. P1 y P2 tienen tres repeticiones cada una; P3 tiene cuatro, porque se hizo una captura adicional. Los logs de estas repeticiones llevan la misma etiqueta `calib01_tilt30_p1_r01`; `run_index.csv` asigna P2/P3 mediante GPS y enumera repeticiones duplicadas por orden temporal, sin editar los CSV brutos.

| Pose | Repeticiones | Candidatos por repetición | MAE candidato por repetición (m) |
| --- | ---: | ---: | ---: |
| P1, Z=−9,0 m | 3 | 10 | 0,0633 |
| P2, Z=−8,8 m | 3 | 9 | 0,0812 |
| P3, Z=−8,3 m | 4 | 3 | 0,0641 |

Las imágenes JPEG de las repeticiones de cada pose son idénticas byte por byte: la repetición comprueba que el render estático es reproducible, no que haya variabilidad estadística independiente. Los errores por repetición coinciden exactamente con la primera tanda. Todas las observaciones candidatas siguen con `near_field_occluded=true`; estas cifras siguen siendo comprobaciones contra la geometría WBT, no precisión validada. El campo cercano oculto permanece desconocido.

### Primera tanda de repeticiones P1/P2/P3

Se localizaron diez corridas de calibración: P1 y P2 tienen tres poses cada una; P3 tiene cuatro (una repetición adicional). El indexador las relaciona por GPS y orden temporal. Aunque los logs nuevos se etiquetaron todos `calib01_tilt30_p1_r01`, los CSV brutos no se modificaron; `run_index.csv` registra la corrección offline de pose/repetición.

Los JPEG de las repeticiones de cada pose son idénticos entre sí (SHA-256), así que las repeticiones confirman la **reproducibilidad del render estático**, no variabilidad estadística independiente. El resumen V3 de esta tanda da los mismos candidatos por pose que la primera captura: P1 10, P2 9 y P3 3 sectores candidatos; todos los candidatos siguen con `near_field_occluded=true`. Las tres MAE candidatas por pose son 0,0633 m, 0,0812 m y 0,0641 m; el agregado de las diez corridas es 0,0704 m sobre 69 sectores candidatos. No tomar el agregado como mejora de precisión: las imágenes repetidas son idénticas y la referencia sigue siendo el plano WBT offline. El campo cercano enmascarado permanece desconocido.

| Pose | Repeticiones indexadas | Run IDs |
| --- | ---: | --- |
| P1, Z=−9,0 m | 3 | `163245`, `180603`, `180900` |
| P2, Z=−8,8 m | 3 | `163324`, `180617`, `180919` |
| P3, Z=−8,3 m | 4 | `163353`, `180641`, `180845`, `180940` |

Los sufijos de `Run ID` corresponden a `run_20261111_<sufijo>`. El indexador excluyó los tres intentos `unlabeled` y la toma del pasillo al reconstruir la comparación de pared.

Reproducir la comparación desde la raíz del repositorio:

```bash
python3 experiments/exp_002_visual_ranging/src/compare_wall_ranges.py
```

Las etiquetas `calib01_*`, incluidas las de repetición `_r01`–`_r03`, capturan desde 1 s simulado y finalizan tras guardar la primera imagen; las otras etiquetas capturan periódicamente. El identificador es solo metadato, nunca aporta geometría o distancia al controlador.

Pruebas offline del generador y del prototipo:

```bash
python3 -m pip install -r experiments/exp_002_visual_ranging/requirements.txt
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v
```

Procesar una captura con `segment_image.py` o `analyze_runs.py` usando `config/self_occlusion_tilt30.json`. Los sectores `occupied` son candidatos geométricos a contactos visibles; las regiones enmascaradas y el campo cercano oculto siguen desconocidos, nunca libres. No reportar la comparación WBT como error métrico validado: faltan referencias físicas independientes y validación completa de extrínsecos. `camera_f` permanece reservada para detección de víctimas, fuera del alcance de EXP-002.

## Métricas y aceptación

- Error absoluto por sector/distancia, proporción de rayos utilizables, falsos libres y sensibilidad a niebla/iluminación.
- Aceptar para integración únicamente con alcance utilizable y errores medidos en referencias independientes, además de tratamiento `unknown` para observaciones dudosas.
- Empezar en escenario 01; solo después de validar allí, confirmar robustez en 03, 07, 08 y 10 antes de conectar observaciones visuales al mapeo.

En inspecciones históricas algunos límites podían corresponder a la silueta del robot; sus datos brutos se eliminaron. La máscara `config/archive/self_occlusion_downward_unverified.json` pertenece a la cámara mirando hacia abajo y permanece sin verificar. No usarla a −30°.
