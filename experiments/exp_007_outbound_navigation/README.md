# EXP-007 — Outbound navigation

**Estado:** planned. **Hipótesis:** el robot cruza del área inicial hasta la línea de progreso de Z = +8 sin identificar el escenario.

**Desarrollo:** 01, 03, 07, 10. **Puerta de integración:** **10 WBT × S1/S2/S3 = 30 corridas**, con las poses validadas descritas en `docs/test_plan.md`.

## Procedimiento
1. Compilar la combinación de mapa, explorador, A* y control seguro.
2. Crear copias de los WBT con cada pose inicial S1–S3 detrás de la línea de salida.
3. Ejecutar sin rescate; registrar progreso y riesgos. No usar geometría del archivo como entrada del controlador.
4. Revisar bloqueos, bucles, zonas ambiguas y tiempo hasta la línea final.

## Métricas y aceptación
- Cruce de Z = +8, tiempo de simulación, distancia recorrida, nº colisiones, bloqueos, replanteos y máxima línea negra cruzada.
- **Objetivo:** 30/30 llegadas a la zona de meta sin colisiones; no declarar robustez general si faltan escenarios o poses. Medir tiempo para reservar presupuesto de rescate y retorno.

**Dependencias:** EXP-005/006. **Salida:** una fila por ejecución y resumen por mundo.
