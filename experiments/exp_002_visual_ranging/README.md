# EXP-002 — Visual radial ranging

**Estado:** in_progress (captura y prototipo de segmentación disponibles; calibración métrica pendiente). **Hipótesis:** la cámara esférica puede producir observaciones radiales útiles como pseudo-LiDAR, con límites explícitos de alcance e incertidumbre.

**Escenarios:** 01 (paredes), 03 (focos y cajas), 07 (niebla de rango bajo), 08 (otra niebla e iluminación), 10 (42 cajas).

## V3 — observaciones angulares y geometría (prototipo offline)

**Estado: implementado como prototipo, sin validación métrica en Webots ni integración con el mapa.** Se corrige el supuesto de V2: medir radios desde el centro de una imagen esférica **no** corresponde a un barrido LiDAR. La configuración actual de `camera_s` es `spherical TRUE`, resolución 160×160, FOV horizontal ≈ π rad (**180°, no 360°**). El siguiente experimento separa la detección de víctimas con `camera_f` de la hipótesis de pseudo-LiDAR con `camera_s`.

**Módulos:**
- `src/spherical_geometry.py`: rayos por píxel de la proyección esférica, rotación de la cámara (tilt nominal −30°), altura 0,44 m y desplazamiento de montaje (X=0,025, Z=−0,05). Calcula intersecciones con el plano del suelo; rechaza rayos al cielo o próximos al horizonte. Entrega distancia horizontal **desde la cámara**, punto en marco del robot y rumbo estimado.
- `src/angular_observations.py`: sectores en columnas angulares, tres columnas por sector, segmentación HSV conservadora en cuatro clases y transición **suelo visible → obstáculo visible**. Si falta suelo visible, hay autooclusión, contradicción entre columnas, un rayo no corta el suelo o no existe máscara de carrocería verificada, devuelve `unknown` y `distance_m=null`. Nunca produce espacio libre.
- `src/segment_image.py`: la CLI usa `--mode angular` por defecto para V3. El antiguo algoritmo circular sigue disponible **solo** como `--mode radial-legacy` por reproducibilidad; no es metrizable.
- `src/analyze_runs.py`: analiza las carpetas `run_...` de P1, P2 y P3 usando los `trial_id` registrados. Guarda `v3_sectors.csv`, `v3_summary.json` y la imagen `*_classes.png` en una ruta separada. No modifica `frames.csv` ni `visual_measurements.csv`.

**Precauciones geométricas:** la geometría está parametrizada desde la cámara de las copias de calibración (no se lee el archivo WBT en el controlador). Se usa un modelo esférico angular y eje óptico `-Z` del dispositivo, con píxeles a la derecha como azimut positivo; **el sentido y el montaje deben verificarse con un marcador lateral de posición conocida en Webots**, especialmente antes de emplear los rumbos en el mapa. La intersección suelo-rayo solo es válida para obstáculos con contacto visible con un suelo plano y no representa distancia a una superficie arbitraria. Un valor de 0,5 en `confidence` es heurístico, no probabilidad calibrada. El alcance [0,2, 4,0] m es un **filtro provisional del prototipo**, no un alcance demostrado.

### Ejecutar V3 sobre las capturas P1–P3

Desde la raíz del repositorio:

```bash
python3 -m pip install -r experiments/exp_002_visual_ranging/requirements.txt
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v

python3 experiments/exp_002_visual_ranging/src/analyze_runs.py \
  experiments/exp_002_visual_ranging/results/run_20261110_202108 \
  experiments/exp_002_visual_ranging/results/run_20261110_202308 \
  experiments/exp_002_visual_ranging/results/run_20261110_202340 \
  --output-dir /tmp/exp002_v3
```

Sin máscara de autooclusión **verificada por inspección de las imágenes**, la salida es deliberadamente `unknown` para los 16 sectores de cada captura: **no se debe reportar MAE**. Se podrá proporcionar una máscara **específica de la cámara inclinada −30°** con `--mask-config <file.json>`; su campo `verified` solo debe marcarse `true` después de contrastarla visualmente con varias imágenes reales. La máscara central antigua de `self_occlusion_pilot.json` está sin verificar y fue creada para otra orientación; **no reutilizarla**.

Para inspeccionar una sola imagen y su clasificación:

```bash
python3 experiments/exp_002_visual_ranging/src/segment_image.py \
  experiments/exp_002_visual_ranging/results/run_20261110_202308/frame_000000.jpg \
  --mode angular --tilt-deg -30 --output-dir /tmp/exp002_v3/frame_review
```

Los archivos de salida `angular_classes.png` y `angular_sectors.json` son diagnósticos. Un sector `occupied` significa **candidato geométrico**, no evidencia aceptada para el mapa; integrar con sensores IR solo tras medir errores y detectar falsos positivos en imágenes independientes. La cámara frontal `camera_f` permanece reservada para detección visual de víctimas, fuera del alcance de EXP-002.

**Capturas piloto existentes:** P1 (0,8 m nominal centro-robot–pared), P2 (1,0 m), P3 (1,5 m), todas a 1,024 s. En su inspección visual anterior se observó el contorno superior de la pared y zonas de carrocería, pero **no se confirmó una frontera continua suelo-pared**. En ausencia de esa frontera y una máscara validada, esas imágenes no demuestran rangos métricos. Se necesitan capturas con suelo visible y un marcador lateral para validar el marco de ángulos, además de referencias independientes antes de calcular MAE.

**Referencia técnica:** [Webots Camera, proyección esférica](https://cyberbotics.com/doc/reference/camera): píxeles `(s,t)` como direcciones angulares, distintas de proyección cilíndrica. Las pruebas nuevas son sintéticas; no sustituyen la verificación en Webots.

---

## Fase 2 — Piloto de calibración de pared (preparado, pendiente de ejecutar)

**Objetivo:** verificar en tres capturas si la transición visual suelo/pared cambia de manera reproducible con la distancia. Esta etapa **no implementa aún píxeles → metros** ni acepta el pseudo-LiDAR.

Se han creado tres **copias experimentales del escenario 01** con `camera_s` a **−30°** (`rotation 1 0 0 -0.5236`), GPS `resolution -1`, X=0, yaw=0, los mismos obstáculos e iluminación, y **sin los elementos gráficos auxiliares** `LENS`, `STRUCTURE_AXES`, `CAMERA_SHAPE` ni `CAMERA_OUTLINE`. La cámara y sus propiedades ópticas permanecen intactas.

| Mundo listo en `worlds/` | Trial ID | Z del robot | Separación nominal centro-pared* |
| --- | --- | ---: | ---: |
| `exp002_calibration_01_p1.wbt` | `calib01_tilt30_p1` | −9,00 m | 0,80 m |
| `exp002_calibration_01_p2.wbt` | `calib01_tilt30_p2` | −8,80 m | 1,00 m |
| `exp002_calibration_01_p3.wbt` | `calib01_tilt30_p3` | −8,30 m | 1,50 m |

*Referencias de configuración obtenidas **offline** del escenario 01 (pared posterior, cara interior ≈ Z=−9,80 m). Son distancias horizontales **desde el origen del robot**, no la distancia óptica exacta a la pared ni la distancia mínima desde la carrocería; comprobar el contacto, perspectiva y referencia de cámara antes de calibrar.

### Ejecución en Webots

1. Actualizar desde GitHub: `git pull origin main`.
2. Abrir `worlds/exp002_calibration_01_p1.wbt`, **compilar `fool_controller` actualizado**, reiniciar la simulación desde t=0 y ejecutarla **al menos 1 segundo simulado**.
3. Esperar en consola `[EXP002_FRAME] ... saved=1` y `[EXP002_DONE] trial=calib01_tilt30_p1`. Solo entonces detener/pausar la simulación. El controlador mantiene las ruedas detenidas.
4. Repetir con `p2` y `p3`, abriendo cada mundo y reiniciando a t=0. No cambiar iluminación, posición manualmente ni inclinación.
5. Guardar o subir a GitHub las tres carpetas `results/run_.../` completas (especialmente `frame_000000.jpg`, `frames.csv` y `summary.txt`), sin sobreescribir corridas anteriores.
6. Inspeccionar primero las **imágenes JPEG sin procesar**: pared visible, suelo, autooclusión y una frontera físicamente coherente. Si hay manchas ambiguas, reportar `unknown` y no calcular metros.

El campo `controllerArgs` de cada copia contiene **solo un identificador textual de prueba**: `exp002_trial=calib01_tilt30_pN`. Webots lo pasa como argumento a `main`; el controlador **no consulta la geometría WBT ni recibe la distancia de referencia**. El logger guarda `trial_id` en `frames.csv` y `summary.txt`.

Para las corridas etiquetadas `calib01_*`, el primer intento de captura es **a partir de t=1,0 s simulado** (32 pasos de 32 ms ≈ 1,024 s), en vez de t=0,032 s. Si la imagen se guarda, la corrida es **single-shot**: se escribe `status=completed` y se cierran los CSV y el resumen automáticamente. Si falla, vuelve a intentar aproximadamente cada segundo. La cámara no se mueve ni se calibra automáticamente. Los mundos anteriores sin etiqueta mantienen registro periódico.

**Validaciones antes de pasar a distancia métrica:**
- `frames.csv`: `image_saved=1`, `gps_z_m` próximo al Z configurado y `trial_id` correcto.
- `summary.txt`: `capture_mode=single_after_1s`, `capture_count`, `images_saved`, `status=completed` (una corrida terminada), sin interpretar `status=recording` como fallo de cámara si se detuvo anticipadamente.
- Determinar si existe transición suelo-pared visible a las tres distancias. Las antiguas máscaras de autooclusión no están calibradas para estas vistas: no reutilizarlas automáticamente.
- Medir posteriormente referencias ópticas independientes y definir método geométrico en `camera_projection.py`. `visual_measurements.csv` seguirá vacío en esta fase deliberadamente.

Reproducción de mundos (están **ya versionados**, no es necesario generarlos de nuevo):

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py --mode calibration
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v
```

El generador conserva `final_scenario*.wbt`, verifica los nodos esperados antes de eliminar visuales y rechaza archivos editados sin `--overwrite`. Las pruebas Python validan la generación; **aún falta compilar y ejecutar Webots en el equipo del usuario**.

---

## Prueba comparativa de dirección de cámara — lista para ejecutar

**Objetivo inmediato:** comprobar si cambiar únicamente la inclinación de la cámara esférica aumenta el suelo y los obstáculos visibles y reduce la autooclusión. **No es todavía una prueba de distancias en metros.**

En el directorio [`worlds/`](../../worlds/) se incluyen **tres mundos piloto versionados**:

| Mundo | Rotación de `camera_s` | GPS | Robot |
| --- | --- | --- | --- |
| `exp002_camera_01_tilt45.wbt` | `1 0 0 -0.7854` (45°) | `resolution -1` | (X=0, Z=−8.8, yaw=0) |
| `exp002_camera_01_tilt30.wbt` | `1 0 0 -0.5236` (30°) | `resolution -1` | Misma pose |
| `exp002_camera_01_horizontal.wbt` | `1 0 0 0` (horizontal nominal) | `resolution -1` | Misma pose |

Los tres derivan del **escenario 01 actual** y mantienen su geometría, iluminación, dispositivos y controlador. El único cambio del sensor GPS es su resolución: según la [referencia de Webots GPS R2020b](https://cyberbotics.com/doc/reference/gps?version=R2020b), `-1` desactiva la cuantización. **La orientación y la cobertura reales de la cámara deben comprobarse con imágenes de Webots.** Los diez archivos `final_scenario*.wbt` no se editan.

### Procedimiento (abrir los mundos ya incluidos, no hace falta generarlos)

1. Desde la raíz del proyecto: `git pull origin main`.
2. Abrir **primero** `worlds/exp002_camera_01_tilt45.wbt` en Webots, compilar/ejecutar `fool_controller`. El robot debe permanecer inmóvil.
3. Confirmar en consola `[START]` y en `frames.csv` **Z cercano a −8.8**, sin redondeo en saltos de 3 m. Si difiere, detener y verificar mundo abierto/estado guardado.
4. Copiar o registrar el **nombre de la corrida** `results/run_...` y su inclinación. Guardar al menos una imagen `frame_000000.jpg` por variante.
5. Repetir desde un **reinicio del mundo** con `exp002_camera_01_tilt30.wbt` y `exp002_camera_01_horizontal.wbt`. No cambiar posición, orientación, materiales ni iluminación.
6. Comparar visualmente las tres imágenes brutas: fracción visible de carrocería, suelo, pared posterior y posibles transiciones suelo-obstáculo. Anotar resultados y condiciones en la siguiente tabla.

| Variante | Run ID | GPS Z (m) | Carrocería visible | Suelo visible | Pared visible | Problemas |
| --- | --- | ---: | --- | --- | --- | --- |
| 45° | pendiente | — | — | — | — | — |
| 30° | pendiente | — | — | — | — | — |
| Horizontal | pendiente | — | — | — | — | — |

**Importante:** la antigua máscara `config/self_occlusion_pilot.json` fue propuesta para la cámara mirando hacia abajo. **No reutilizarla automáticamente** en las nuevas inclinaciones: crearía mediciones engañosas. Primero revisar las imágenes reales y, si se aplica `segment_image.py`, generar/ajustar una máscara independiente **por orientación**. La máscara debe permanecer `verified=false` hasta inspección manual.

Los sectores `usable` son fronteras visuales, **no** garantía de terreno libre; el campo oculto por el robot permanece `unknown`. No estimar distancias en metros ni declarar EXP-002 `accepted` por completar esta comparación.

### Reproducibilidad

Los mundos de comparación ya están incluidos en GitHub. Para regenerarlos desde la geometría del `final_scenario01.wbt` existente:

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py
```

El generador compara el contenido con los archivos existentes: si coinciden, no escribe nada; si son diferentes, falla antes de sobrescribirlos. Para volver a crearlos voluntariamente después de cambiar parámetros del experimento, usar `--overwrite` y registrar la modificación. Se puede elegir otra carpeta mediante `--output-dir`. **No utilizar este comando para reescribir los mundos `final_scenario*.wbt`.**

Pruebas offline del generador:

```bash
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v
```

Las pruebas sintéticas verifican sustitución de pose, GPS, rotación de cámara, comportamiento idempotente y rechazo de archivos incompatibles. **No equivalen a ejecutar los mundos en Webots.**

---

## Procedimiento
1. Capturar imágenes estáticas a distancias y orientaciones conocidas de paredes y cajas, usando posiciones de prueba sin colisiones.
2. Segmentar suelo/obstáculo y medir límites radiales por sectores. Calibrar píxeles a distancia solo cuando exista relación verificable.
3. Comparar la estimación con distancias de referencia obtenidas **offline** del WBT o con medidas independientes; repetir en visibilidad variable.
4. Registrar casos de oclusión, saturación, falso pasillo y dato desconocido.

## Métricas y aceptación
- Error absoluto por sector/distancia, proporción de rayos utilizables, falsos libres, sensibilidad a iluminación/niebla.
- **Aceptar para integración** únicamente si se define alcance utilizable, errores observados y un mecanismo que marque como `unknown` las observaciones dudosas. No hay umbral métrico validado: debe fijarse después del piloto de 01 y antes de comparar la versión candidata final.

**Salida:** imágenes de muestra referenciadas, mediciones compactas, método de calibración y limitaciones en `results/`.

## Registro automático por corrida

Al iniciar el controlador se crea `results/run_YYYYMMDD_HHMMSS/` (con sufijo para evitar colisiones). Cada segundo simulado guarda una imagen JPEG de la cámara esférica, si está disponible, junto con:

- `frames.csv`: índice, tiempo simulado, pose GPS, orientación, disponibilidad/guardado de imagen, ruta de imagen, resolución y campo de visión.
- `visual_measurements.csv`: una fila preparada por imagen para anotar escenario, tipo de obstáculo, distancia de referencia, sector, límite radial en píxeles, distancia estimada, error y estado `usable`/`unknown`.
- `summary.txt`: timestep, resolución, FOV, duración simulada, número de intentos de captura e imágenes guardadas.

El controlador no identifica el escenario ni lee la geometría del WBT. Por eso la distancia de referencia, el objeto y la condición de visibilidad se completan offline en `visual_measurements.csv`, usando copias de prueba o mediciones independientes. Las filas sin medición permanecen vacías; capturar la imagen no equivale a validar una estimación métrica.

## Prototipo de visión offline — implementado

El código Python es **independiente del controlador Webots**:

- [`src/camera_projection.py`](src/camera_projection.py): transformación de píxeles a coordenadas angulares de la imagen según la descripción de proyección esférica de Webots, muestreo radial de sectores y validación geométrica. La rotación física de la cámara y la correspondencia entre sector visual y rumbo del robot **aún deben comprobarse con capturas reales**.
- [`src/segment_image.py`](src/segment_image.py): segmentación HSV conservadora en cuatro clases (`unknown`, `floor`, `obstacle`, `victim`); busca suelo candidato en un anillo **fuera de la carrocería enmascarada** y exige acuerdo entre tres rayos por sector. Devuelve `unknown` ante oclusiones no modeladas, sombras o límites ambiguos.
- [`tests/test_exp002_vision.py`](tests/test_exp002_vision.py): pruebas unitarias con imágenes **sintéticas**, que comprueban coordenadas, clasificación, fronteras y tratamiento de incertidumbre.

Los valores de confianza son **indicadores heurísticos**, no probabilidades calibradas. `boundary_px` expresa distancia **en píxeles respecto al centro de imagen**; **no es una distancia en metros**. Los ángulos de los sectores también pertenecen al plano de la imagen y no se deben utilizar como orientaciones globales sin registrar el montaje de la cámara.

### Instalación y ejecución

Requiere Python 3.10 o superior, NumPy y OpenCV:

```bash
python3 -m pip install -r experiments/exp_002_visual_ranging/requirements.txt
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v
```

Procesar **una imagen real guardada por `Exp002Logger`** y generar visualizaciones de diagnóstico:

```bash
python3 experiments/exp_002_visual_ranging/src/segment_image.py \
  experiments/exp_002_visual_ranging/results/run_YYYYMMDD_HHMMSS/frame_000000.jpg \
  --mask-config experiments/exp_002_visual_ranging/config/self_occlusion_pilot.json \
  --sectors 16 --output-dir /tmp/exp002_preview
```

El comando imprime JSON y, con `--output-dir`, guarda `mask.png`, `overlay.png`, `occlusion_mask.png` y `boundaries.json`. No modifica `visual_measurements.csv`, no consulta `.wbt`, no estima metros y no modifica el controlador.

### Validación pendiente antes de EXP-003

1. Inspeccionar una muestra real del escenario 01; comprobar orientación/centro de cámara y clases de píxeles, especialmente bajo niebla, sombras y exposición.
2. Ajustar HSV y la región semilla sobre datos de **entrenamiento**, conservando pruebas separadas.
3. Obtener referencias independientes por sector, relacionar `boundary_px` con distancia real **solo si los datos respaldan la geometría**, y medir error.
4. Implementar análisis automático por corrida y generación de `visual_measurements.csv` con distancias/errores, dejando `unknown` sin distancia.
5. Verificar robustez en 03, 07, 08 y 10. Solo entonces decidir si integrar la visión en el mapeo.

La validación anterior de segmentación incluyó 15 pruebas unitarias sintéticas; el generador de los tres mundos cuenta además con cuatro pruebas offline propias. La comparación de orientaciones ya dispone de tres capturas; esta fase de calibración aún no tiene capturas reales. **No** implica rendimiento demostrado en Webots ni aceptación de EXP-002.

### Primera inspección de imagen Webots

La captura anterior, con GPS cuantizado a 3 metros, `results/run_20261110_174034/frame_000000.jpg` se guardó correctamente (160 × 160 px, FOV 3.1415 rad; pose registrada `(0,-6)`, rumbo `0 rad`). El escenario no quedó etiquetado en los metadatos. El análisis produjo 13 sectores `unknown` por transición ambigua y tres límites en sectores 12–14, a 58, 60 y 66 px, confianza heurística 0.6. **Los 16 sectores tienen `near_field_occluded=true`.**

El detalle está en [`analysis_summary.md`](results/run_20261110_174034/analysis_summary.md). Esta inspección no valida la máscara provisional ni la distancia métrica; los tres límites pueden corresponder a la silueta visible del robot. Revisar la máscara con la imagen real antes de marcarla como verificada.


## Nota sobre la captura anterior

La corrida `run_20261110_174034` se capturó con la cámara mirando hacia abajo y el GPS del mundo con resolución de 3 m. El registro `Z=-6` fue una lectura cuantizada, no prueba de una pose real distinta. Esa imagen mostró principalmente la carrocería y motivó el experimento comparativo. Sus tres límites visuales (sectores 12–14) no se aceptan como medidas de pared ni como distancias.

La máscara `config/self_occlusion_pilot.json` continúa **sin verificar** y solo corresponde a la hipótesis de cámara hacia abajo. Para 45°, 30° y horizontal se deben revisar primero las imágenes brutas y, si procede, crear máscaras independientes antes de ejecutar segmentación y calibración. Mantener los sectores ambiguos en estado `unknown`.
