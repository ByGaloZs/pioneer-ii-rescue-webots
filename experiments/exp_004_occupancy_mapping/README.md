# EXP-004 — Incremental occupancy mapping

**Estado:** planned. **Hipótesis:** los datos fusionados y la pose permiten construir un mapa de ocupación local/acumulado que distingue libre, ocupado y desconocido.

**Escenarios:** 01 (paredes), 03 (cajas), 07 (niebla), 10 (alta densidad).

## Procedimiento
1. Iniciar con mapa vacío; registrar observaciones inmóviles y desplazamientos controlados usando GPS, brújula y encoders.
2. Transformar rayos al marco del mundo; actualizar celdas y aplicar margen por huella del robot.
3. Comparar snapshots contra un mapa de referencia obtenido offline de los WBT (nunca disponible para el controlador).
4. Revisar contradicciones, deriva, celdas desconocidas y falsos corredores.

## Métricas y aceptación
- Precisión de ocupación en celdas observadas, tasa de falsos libres, integridad del espacio desconocido, deriva de mapa.
- **Aceptar** si el mapa integra observaciones sin rellenar desconocidos como libres, e identifica obstáculos que impedirían el paso seguro. Ajustar tolerancias con prueba estática antes de navegación autónoma.

**Dependencia:** EXP-003. **Salida:** mapas comparativos y métricas pequeñas en `results/`.
