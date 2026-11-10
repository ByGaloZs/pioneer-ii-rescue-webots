# EXP-002 — Visual radial ranging

**Estado:** planned. **Hipótesis:** la cámara esférica puede producir observaciones radiales útiles como pseudo-LiDAR, con límites explícitos de alcance e incertidumbre.

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
