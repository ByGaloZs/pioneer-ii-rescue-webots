# EXP-003 — Sensor fusion

**Estado:** planned (siguiente experimento). **Hipótesis:** combinar candidatos de contacto visual V3 con lecturas calibradas de proximidad conserva los contactos útiles y no reduce regiones desconocidas a espacio libre sin evidencia.

**Alcance inicial acordado:** únicamente `worlds/experiment_test_scenario01.wbt`, por ahora. No se ejecutará ni afirmará generalización en los otros escenarios hasta que se reabra ese alcance. La matriz más amplia sigue en `docs/test_plan.md`.

## Preparación

- Dependencias: EXP-001 y cierre parcial de EXP-002 para consistencia geométrica del pseudo-LiDAR en escenario 01.
- Mantener la inclinación de `camera_s` a −30° y la máscara específica `self_occlusion_tilt30.json`.
- Antes de ejecutar EXP-003, reconfigurar el mismo `controllers/experiment_test_controller/` para capturar cámara y sensores de proximidad con etiquetas `experiment=003;trial=<id>`, y escribir únicamente bajo `experiments/exp_003_sensor_fusion/results/`.
- Regenerar la única copia `worlds/experiment_test_scenario01.wbt` desde `final_scenario01.wbt`; no duplicar controladores ni editar mundos canónicos.
- Preparar comparaciones offline sobre capturas/poses idénticas para cámara sola, proximidad sola y fusión. Registrar la pose GPS/yaw real de cada captura.

## Procedimiento
1. En escenario 01, reproducir los mismos puntos/ángulos de observación para tres variantes: cámara sola, proximidad sola y fusión.
2. Comparar ocupación por sector y evaluar cobertura, desacuerdos, saturación y lecturas ausentes.
3. Si los sensores discrepan, exigir tratamiento explícito de incertidumbre en lugar de declarar paso libre.
4. Contrastar con geometría de escenario 01 **offline** y registrar casos problemáticos; no exponer referencias al controlador.

## Métricas y aceptación
- Falsos libres/falsos ocupados, rango válido, cobertura angular, nº discrepancias clasificadas como desconocidas.
- Registrar antes de ejecutar la tolerancia de error/cobertura para esta matriz de escenario 01; no inferirla de los resultados después de observarlos.
- **Aceptar** si la fusión no empeora los errores de seguridad observados frente al mejor sensor individual y no transforma una lectura incierta en paso confirmado sin soporte.

**Dependencias:** EXP-001 y EXP-002. **Salida:** comparación A/B/C documentada en `results/`. La corrida es futura; no hay resultados de EXP-003 todavía.
