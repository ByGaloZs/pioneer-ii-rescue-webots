# EXP-010 — Full rescue mission

**Estado:** planned. **Hipótesis:** la integración resuelve la misión completa sobre escenarios desconocidos dentro de 240 s simulados.

**Prueba de humo:** 01, 07, 10. **Evaluación completa:** **10 escenarios × 3 poses S1/S2/S3 = 30 corridas**. Registrar versiones y usar copias de prueba para poses aleatorias/controladas, sin tocar los WBT originales.

## Procedimiento
1. Ejecutar el controlador integrado desde cada pose válida detrás de Z = −8, con orientación asignada.
2. Medir ida hacia la región de víctimas, dos aproximaciones con giro de 360°, y regreso al origen.
3. Finalizar al completar misión o alcanzar 240 s, registrando causa precisa de fallo y avance parcial.
4. Comparar resultados por mundo, orientación, iluminación, niebla y densidad de obstáculos.

## Métricas y aceptación
- Resultado por corrida, tiempo total simulado, colisiones, líneas negras cruzadas, nº víctimas únicas rescatadas, distancia < 1 m y giro de 360° por rescate, regreso al inicio.
- **Objetivo final:** **30/30 misiones completas dentro de 240 s sin colisiones**. Es una meta de diseño, no una afirmación sobre el rendimiento.
- Ante fallos, reportar indicadores parciales conforme al enunciado y generar experimento correctivo sin borrar los resultados negativos.

**Dependencias:** EXP-001 a EXP-009. **Salida:** `results/metrics.csv`, resumen por escenario y evidencia para informe y presentación.
