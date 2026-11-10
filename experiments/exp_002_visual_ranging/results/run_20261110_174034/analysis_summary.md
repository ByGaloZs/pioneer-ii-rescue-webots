# Inspección visual — run_20261110_174034

- Imagen: `frame_000000.jpg` (160 × 160 px); guardado confirmado en `frames.csv`.
- Pose registrada: `(x=0.000, z=-6.000)`, rumbo `0.000 rad`; el controlador no registró el escenario.
- Máscara: `self_occlusion_pilot.json`, aún `verified=false`; 2.915 píxeles excluidos.
- Sectores 0–11 y 15: `unknown`, motivo `ambiguous_transition`.
- Sectores 12–14: límites de imagen a 58, 60 y 66 px, confianza heurística 0.6; `near_field_occluded=true`.

## Interpretación

Es una salida en espacio de imagen, no una medida en metros ni evidencia de espacio libre. Los límites utilizables podrían ser el borde de la carrocería que permanece visible fuera de la máscara provisional. No se acepta ni calibra el pseudo-LiDAR con esta corrida. Inspeccionar y ajustar la máscara sobre imágenes reales antes de repetir el análisis; mantener `unknown` cualquier región ambigua.

Las visualizaciones `overlay.png`, `mask.png`, `occlusion_mask.png` y `boundaries.json` se generaron en `/tmp/exp002_preview` para esta inspección y no forman parte de esta corrida guardada.
