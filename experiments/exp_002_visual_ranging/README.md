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
- [`src/segment_image.py`](src/segment_image.py): segmentación HSV conservadora en cuatro clases (`unknown`, `floor`, `obstacle`, `victim`); exige que el suelo candidato esté conectado a una zona central y que tres rayos por sector confirmen una transición suelo-obstáculo. Devuelve `unknown` si hay ambigüedad, oclusión o discrepancia de rayos.
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
  --sectors 16 --output-dir /tmp/exp002_preview
```

El comando imprime JSON y, con `--output-dir`, guarda `mask.png`, `overlay.png` y `boundaries.json`. No modifica `visual_measurements.csv`, no consulta `.wbt`, no estima metros y no modifica el controlador.

### Validación pendiente antes de EXP-003

1. Inspeccionar una muestra real del escenario 01; comprobar orientación/centro de cámara y clases de píxeles, especialmente bajo niebla, sombras y exposición.
2. Ajustar HSV y la región semilla sobre datos de **entrenamiento**, conservando pruebas separadas.
3. Obtener referencias independientes por sector, relacionar `boundary_px` con distancia real **solo si los datos respaldan la geometría**, y medir error.
4. Implementar análisis automático por corrida y generación de `visual_measurements.csv` con distancias/errores, dejando `unknown` sin distancia.
5. Verificar robustez en 03, 07, 08 y 10. Solo entonces decidir si integrar la visión en el mapeo.

La validación local realizada para este cambio incluye **ocho tests sintéticos y una prueba de ejecución del CLI con imagen sintética**. **No** implica rendimiento demostrado en Webots ni aceptación de EXP-002.
