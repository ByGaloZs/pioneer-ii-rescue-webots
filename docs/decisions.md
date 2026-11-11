# Registro de decisiones técnicas

Cada decisión conserva contexto, alternativas, justificación, consecuencias y prueba requerida. **Diseño elegido no significa implementación probada.**

## ADR-001 — No identificar el escenario
**Estado:** adoptada para el diseño. **Decisión:** navegar con medidas online y no cargar geometría del escenario de evaluación. **Alternativa:** reconocer cuál de los diez mapas se usa. **Motivo:** la identificación local puede ser ambigua y consumir el tiempo de misión. **Consecuencia:** se requiere mapeo incremental. **Prueba:** comparar progreso y tiempo en los diez mundos.

## ADR-002 — Mapas local y acumulado
**Estado:** adoptada para el diseño. **Decisión:** construir rejilla local con observaciones, acumularla en el marco global usando la pose. **Alternativa:** control reactivo sin memoria o mapeo completo previo. **Motivo:** memorizar recorridos y permitir el retorno sin explorar toda la arena. **Riesgo:** deriva y errores de sensores. **Prueba:** consistencia de celdas y zonas transitables.

## ADR-003 — Fusión visual y de distancia
**Estado:** hipótesis pendiente de validación. **Decisión revisada (EXP-002 V3):** tratar `camera_s` como fuente de candidatos de distancia por sectores angulares (FOV actual ≈180°, **no 360°**) con geometría y autooclusión verificadas; usar los 16 sensores de proximidad para seguridad cercana, GPS/brújula/encoders para pose y `camera_f` para detección de víctimas. **Alternativa:** usar el radio de píxel como rango LiDAR o confiar en una sola modalidad. **Motivo:** complementar cobertura y proximidad sin confundir imagen esférica con un escáner real. **Riesgo:** contacto con el suelo no visible, errores de montaje/proyección y falsas distancias si se ignora la carrocería. **Prueba requerida:** orientación con marcador lateral, contacto visible suelo-obstáculo, MAE por sector con referencias externas y verificación de falsos libres en imágenes independientes. Hasta entonces no conectar V3 al mapa.

## ADR-004 — Frontier-Based Exploration dirigida
**Estado:** adoptada para el diseño. **Decisión:** priorizar fronteras **alcanzables** por progreso hacia el extremo opuesto, coste, seguridad e información nueva. **Alternativa:** frontera más cercana o seguir pared indefinidamente. **Motivo:** evitar exploración irrelevante con 240 s de misión. **Riesgo:** ignorar rodeos necesarios. **Prueba:** bloqueos, distancia recorrida y progreso.

## ADR-005 — A* con ocupación confirmada
**Estado:** adoptada para el diseño. **Decisión:** planificar hasta fronteras mediante A* solo por celdas confirmadas como libres e infladas; replanificar ante riesgo detectado. **Alternativa:** tratar desconocido como libre o depender de Bug2. **Motivo:** navegación segura con memoria. **Limitación:** A* no descubre obstáculos por sí solo. **Prueba:** colisiones y replanteos eficaces.

## ADR-006 — Aislar experimentos del controlador estable
**Estado:** adoptada. **Decisión:** mantener hipótesis, código offline y resultados por experimento; todos los ensayos Webots reutilizan un solo `controllers/experiment_test_controller/` y una única copia regenerable de scenario 01 en `worlds/experiment_test_scenario01.wbt`. El mundo recibe experimento/trial explícitos y el controlador escribe solo en la carpeta de resultados del experimento activo. `fool_controller` no escribe resultados experimentales. **Alternativa:** loggers/prototipos en el controlador de misión o un controlador y mundo separado por experimento. **Motivo:** evitar contaminación de resultados y facilitar desarrollo incremental en el mismo escenario base, preservando una línea base estable. **Promoción:** integrar únicamente módulos validados, sin reemplazar el punto de entrada ni los mundos canónicos. **Prueba:** el único WBT selecciona el harness correcto y solo produce resultados en la carpeta del experimento activo.

## ADR-007 — Presupuesto temporal explícito
**Estado:** pendiente de calibración. **Decisión:** supervisar tiempo simulado y estimar coste de retorno; evitar una reserva arbitraria antes de medir. **Alternativa:** umbral fijo. **Motivo:** misión de 240 s y necesidad de regresar. **Prueba:** tiempos por fase y éxito en escenarios representativos.

## ADR-008 — Resolución del GPS sin cuantización
**Estado:** adoptada. **Decisión:** configurar `GPS.resolution -1` en los diez mundos canónicos y en la copia compartida de pruebas. **Alternativa:** conservar la resolución original de 3 m. **Motivo:** evitar cuantización de la posición GPS durante validación y navegación. **Alcance:** es un cambio explícito en los WBT; la API del controlador lee el GPS, pero la resolución la configura cada mundo. No se alteran otros campos originales. **Verificación:** comprobar `resolution -1` en todos los WBT y registrar sus hashes actuales en `worlds/README.md`.

## Historial
- **2026-11-10:** decisiones de diseño registradas. Los algoritmos de navegación y rescate aún no se consideran validados.

- **2026-11-10 (EXP-002 V3):** proyección esférica por columnas y estimación suelo-rayo offline implementadas como prototipo; no validadas con Webots ni fusionadas con sensores IR.
- **2026-11-11:** el usuario precisó que el proyecto debe usar un solo harness y una copia compartida de scenario 01 para todos los experimentos; se ajustó la decisión a ese flujo. La compilación y ejecución de Webots deben verificarse en el entorno de desarrollo.
