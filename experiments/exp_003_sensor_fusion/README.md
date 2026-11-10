# EXP-003 — Sensor fusion

**Estado:** planned. **Hipótesis:** combinar distancias calibradas y visión radial disminuye los falsos espacios transitables frente a usar solo cámara.

**Escenarios:** 01, 07, 08, 10; incluyen ausencia de niebla, niebla de distinto tipo y alta cantidad de cajas.

## Procedimiento
1. Reproducir los mismos puntos/ángulos de observación para tres variantes: cámara sola, proximidad sola y fusión.
2. Comparar ocupación por sector y evaluar cobertura, desacuerdos, saturación y lecturas ausentes.
3. Si los sensores discrepan, exigir tratamiento explícito de incertidumbre en lugar de declarar paso libre.
4. Contrastar con geometría **offline** y registrar casos problemáticos.

## Métricas y aceptación
- Falsos libres/falsos ocupados, rango válido, cobertura angular, nº discrepancias clasificadas como desconocidas.
- **Aceptar** si la fusión no empeora los errores de seguridad observados frente al mejor sensor individual y no transforma una lectura incierta en paso confirmado sin soporte. Criterios métricos de tolerancia tras piloto.

**Dependencias:** EXP-001 y EXP-002. **Salida:** comparación A/B/C documentada en `results/`.
