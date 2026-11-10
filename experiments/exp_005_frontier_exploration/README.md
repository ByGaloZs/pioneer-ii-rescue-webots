# EXP-005 — Goal-biased frontier exploration

**Estado:** planned. **Hipótesis:** seleccionar fronteras accesibles con sesgo hacia +Z produce avance sin ciclos persistentes.

**Escenarios:** 01 (geometría), 03 (combinación de obstáculos), 07 (niebla), 10 (alta densidad).

## Procedimiento
1. Ejecutar selección de fronteras sobre mapas parciales sintéticos con callejones, rutas alternativas y fronteras no alcanzables.
2. Conectar exploración + A* en Webots; realizar recorridos parciales empezando por escenario 01.
3. Registrar cada frontera candidata, ruta posible, progreso en +Z, bloqueo y cambios de objetivo.
4. Comparar contra selección de frontera más cercana en iguales condiciones.

## Métricas y aceptación
- Progreso hacia +Z, nº fronteras inviables, revisitas, tiempo sin progreso, distancia y colisiones.
- **Aceptar** si no planifica a través de desconocidos, descarta metas inalcanzables y recupera el progreso al detectar un bloqueo; estudiar contratiempos y fallos en cada mapa.

**Dependencias:** EXP-004 y EXP-006. **Salida:** trazas de decisiones y métricas comparativas.
