# EXP-002 — Visual radial ranging

**Estado:** in_progress (captura y prototipo de segmentación disponibles; calibración métrica pendiente). **Hipótesis:** la cámara esférica puede producir observaciones radiales útiles como pseudo-LiDAR, con límites explícitos de alcance e incertidumbre.

**Escenarios:** 01 (paredes), 03 (focos y cajas), 07 (niebla de rango bajo), 08 (otra niebla e iluminación), 10 (42 cajas).

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
- [`tests/test_exp002_vision.py`](tests/test_exp002_vision.py): ocho pruebas unitarias con imágenes **sintéticas**, que comprueban coordenadas, clasificación, fronteras y tratamiento de incertidumbre.

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

La validación local del prototipo y la exclusión de carrocería incluye **15 pruebas unitarias sintéticas y una ejecución del CLI con una imagen sintética**. **No** implica rendimiento demostrado en Webots ni aceptación de EXP-002.

### Primera inspección de imagen Webots

La captura `results/run_20261110_174034/frame_000000.jpg` se guardó correctamente (160 × 160 px, FOV 3.1415 rad; pose registrada `(0,-6)`, rumbo `0 rad`). El escenario no quedó etiquetado en los metadatos. El análisis produjo 13 sectores `unknown` por transición ambigua y tres límites en sectores 12–14, a 58, 60 y 66 px, confianza heurística 0.6. **Los 16 sectores tienen `near_field_occluded=true`.**

El detalle está en [`analysis_summary.md`](results/run_20261110_174034/analysis_summary.md). Esta inspección no valida la máscara provisional ni la distancia métrica; los tres límites pueden corresponder a la silueta visible del robot. Revisar la máscara con la imagen real antes de marcarla como verificada.


## Piloto de carrocería y pared posterior — escenario 01

La cámara esférica **ve parte de la propia carrocería**. Una muestra central de color gris puede ser robot, no suelo. Antes de calibrar distancia, hay que excluir dicha silueta explícitamente y comprobar qué zonas son realmente observables.

### Paso A: preparar la copia del mundo (desde la raíz del repositorio)

```bash
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py
```

Este comando **no modifica** `worlds/final_scenario01.wbt`: crea `worlds/exp002_pilot_01.wbt` con el robot `Pioneer2` en `translation 0 0 -8.8` y `rotation 0 1 0 0`. Es la franja central próxima a la pared posterior. A modo de referencia offline: cara interior de la pared próxima a Z ≈ −9.8, separación horizontal nominal **centro de robot–pared ≈ 1.0 m**. **No confundir con la distancia óptica exacta de la cámara al muro**. Verificar ausencia de colisiones y orientación real en Webots.

Si se necesita otra pose, borrar **solo** la copia generada y ejecutar con `--z`, `--x` y `--yaw`; conservar el mundo original sin cambios.

### Paso B: capturar una imagen real

1. Abrir `worlds/exp002_pilot_01.wbt` en Webots y compilar `fool_controller`.
2. Iniciar la simulación. El controlador actual debe dejar **las ruedas detenidas**.
3. Localizar en consola `[EXP002_LOG]` y, después, el archivo `experiments/exp_002_visual_ranging/results/run_.../frame_000000.jpg`.
4. Registrar manualmente `scenario=01`, `pose=(0,-8.8)`, `yaw=0` y la distancia de referencia independiente cuando se mida. El logger no lee datos de los mundos.

### Paso C: revisar la máscara provisional

El archivo `config/self_occlusion_pilot.json` contiene **una elipse central de ejemplo, no una máscara verificada**. Ejecutar el comando de segmentación mostrado arriba y abrir `occlusion_mask.png` y `overlay.png`.

- Editar `center` y `radii` en la elipse utilizando **coordenadas normalizadas entre 0 y 1**, o sustituir la elipse por regiones `polygon` con una lista de puntos normalizados.
- La máscara debe abarcar la carrocería visible (incluidos bordes), **sin tapar el suelo válido**. Si la silueta no está clara o no se puede separar, ampliar `unknown` en lugar de adivinar.
- **No marcar `verified=true` hasta inspeccionar visualmente la máscara sobre imágenes reales de la cámara**. Repetir con otra orientación para comprobar que la silueta se mantiene.
- Si el robot oculta un sector entero, su resultado será `unknown`; no se convertirá a espacio libre.
- Un sector con `status=usable` y `near_field_occluded=true` **solo tiene un límite visible más lejos de la carrocería**. La zona escondida sigue siendo desconocida; nunca alimentar ese dato directamente a un planificador.

### Paso D: interpretar el resultado

`boundaries.json` registra por sector `boundary_px`, `status`, `unknown_reason`, `near_field_occluded` y `first_visible_floor_px`. Los sectores representan **ángulos en la imagen**, no direcciones del robot. La salida **no estima distancias en metros** y no actualiza `visual_measurements.csv`: la calibración y las métricas de EXP-002 siguen pendientes.

### Criterio de revisión del piloto

Solo avanzar a calibración cuando las capturas reales permitan reconocer suelo, pared y carrocería **de forma reproducible**, sin falsos pasos libres. Si la elipse provisional no coincide con la silueta, ajustarla en JSON y documentar el cambio junto con el `run_id`.
