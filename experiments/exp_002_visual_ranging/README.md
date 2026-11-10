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
