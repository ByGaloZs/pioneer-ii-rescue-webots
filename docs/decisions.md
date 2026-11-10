# Registro de decisiones técnicas

Cada decisión conserva contexto, alternativas, justificación, consecuencias y prueba requerida. **Diseño elegido no significa implementación probada.**

## ADR-001 — No identificar el escenario
**Estado:** adoptada para el diseño. **Decisión:** navegar con medidas online y no cargar geometría del escenario de evaluación. **Alternativa:** reconocer cuál de los diez mapas se usa. **Motivo:** la identificación local puede ser ambigua y consumir el tiempo de misión. **Consecuencia:** se requiere mapeo incremental. **Prueba:** comparar progreso y tiempo en los diez mundos.

## ADR-002 — Mapas local y acumulado
**Estado:** adoptada para el diseño. **Decisión:** construir rejilla local con observaciones, acumularla en el marco global usando la pose. **Alternativa:** control reactivo sin memoria o mapeo completo previo. **Motivo:** memorizar recorridos y permitir el retorno sin explorar toda la arena. **Riesgo:** deriva y errores de sensores. **Prueba:** consistencia de celdas y zonas transitables.

## ADR-003 — Fusión visual y de distancia
**Estado:** hipótesis pendiente de validación. **Decisión:** usar cámara 360° como fuente radial calibrada y 16 sensores de proximidad para evidencia de obstáculos, con GPS/brújula/encoders para pose. **Alternativa:** confiar en una modalidad. **Motivo:** complementar cobertura y proximidad. **Riesgo:** estimaciones visuales no métricas sin calibración. **Prueba:** error por dirección, falsos libres, pasos no transitables e iluminación.

## ADR-004 — Frontier-Based Exploration dirigida
**Estado:** adoptada para el diseño. **Decisión:** priorizar fronteras **alcanzables** por progreso hacia el extremo opuesto, coste, seguridad e información nueva. **Alternativa:** frontera más cercana o seguir pared indefinidamente. **Motivo:** evitar exploración irrelevante con 240 s de misión. **Riesgo:** ignorar rodeos necesarios. **Prueba:** bloqueos, distancia recorrida y progreso.

## ADR-005 — A* con ocupación confirmada
**Estado:** adoptada para el diseño. **Decisión:** planificar hasta fronteras mediante A* solo por celdas confirmadas como libres e infladas; replanificar ante riesgo detectado. **Alternativa:** tratar desconocido como libre o depender de Bug2. **Motivo:** navegación segura con memoria. **Limitación:** A* no descubre obstáculos por sí solo. **Prueba:** colisiones y replanteos eficaces.

## ADR-006 — Separar experimentos
**Estado:** adoptada. **Decisión:** carpetas `experiments/exp_NNN_short_name`, protocolo reproducible y registro de resultados, sin prototipos dentro del controlador de misión. **Alternativa:** archivos y salidas mezclados. **Motivo:** trazabilidad y evidencia para memoria final. **Prueba:** reproducibilidad y registro de fallos/decisiones.

## ADR-007 — Presupuesto temporal explícito
**Estado:** pendiente de calibración. **Decisión:** supervisar tiempo simulado y estimar coste de retorno; evitar una reserva arbitraria antes de medir. **Alternativa:** umbral fijo. **Motivo:** misión de 240 s y necesidad de regresar. **Prueba:** tiempos por fase y éxito en escenarios representativos.

## Historial
- **2026-11-10:** decisiones de diseño registradas. Los algoritmos de navegación y rescate aún no se consideran validados.
