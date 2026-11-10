# EXP-002 — Visual radial ranging

**Estado:** in_progress (captura y prototipo de segmentación disponibles; calibración métrica pendiente). **Hipótesis:** la cámara esférica puede producir observaciones radiales útiles como pseudo-LiDAR, con límites explícitos de alcance e incertidumbre.

**Escenarios:** 01 (paredes), 03 (focos y cajas), 07 (niebla de rango bajo), 08 (otra niebla e iluminación), 10 (42 cajas).

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

La validación anterior de segmentación incluyó 15 pruebas unitarias sintéticas; el generador de los tres mundos cuenta además con cuatro pruebas offline propias. La ejecución comparativa en Webots está pendiente. **No** implica rendimiento demostrado en Webots ni aceptación de EXP-002.

### Primera inspección de imagen Webots

La captura anterior, con GPS cuantizado a 3 metros, `results/run_20261110_174034/frame_000000.jpg` se guardó correctamente (160 × 160 px, FOV 3.1415 rad; pose registrada `(0,-6)`, rumbo `0 rad`). El escenario no quedó etiquetado en los metadatos. El análisis produjo 13 sectores `unknown` por transición ambigua y tres límites en sectores 12–14, a 58, 60 y 66 px, confianza heurística 0.6. **Los 16 sectores tienen `near_field_occluded=true`.**

El detalle está en [`analysis_summary.md`](results/run_20261110_174034/analysis_summary.md). Esta inspección no valida la máscara provisional ni la distancia métrica; los tres límites pueden corresponder a la silueta visible del robot. Revisar la máscara con la imagen real antes de marcarla como verificada.


## Nota sobre la captura anterior

La corrida `run_20261110_174034` se capturó con la cámara mirando hacia abajo y el GPS del mundo con resolución de 3 m. El registro `Z=-6` fue una lectura cuantizada, no prueba de una pose real distinta. Esa imagen mostró principalmente la carrocería y motivó el experimento comparativo. Sus tres límites visuales (sectores 12–14) no se aceptan como medidas de pared ni como distancias.

La máscara `config/self_occlusion_pilot.json` continúa **sin verificar** y solo corresponde a la hipótesis de cámara hacia abajo. Para 45°, 30° y horizontal se deben revisar primero las imágenes brutas y, si procede, crear máscaras independientes antes de ejecutar segmentación y calibración. Mantener los sectores ambiguos en estado `unknown`.
