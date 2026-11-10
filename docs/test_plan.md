# Plan maestro de pruebas — Pioneer II Rescue

**Estado:** PLANIFICADO. No hay corridas nuevas ni resultados de navegación registrados. Este plan proviene del [catálogo estático de escenarios](scenario_catalog.md) y de las reglas de `Práctica06_Trabajo Final.pdf`. Se ejecutará una validación progresiva, no una matriz exhaustiva de diez experimentos por diez mundos.

## Estrategia y fases

1. **Prueba de humo:** escenario 01, para verificar que el componente funciona sin complicar la depuración.
2. **Pruebas de estrés dirigidas:** seleccionar mundos con características relevantes (03, 07, 08, 09, 10); medir fallos y ajustar solo sobre resultados documentados.
3. **Puerta de integración:** navegar en los diez mundos con poses iniciales variadas. EXP-010 evalúa la misión completa en todos.
4. **Regresión:** si cambia calibración, percepción, generación de mapas o seguridad, repetir los casos representativos afectados y volver a ejecutar integración cuando proceda.

No interpretar que pasar en un mundo implica generalización. Tampoco que un mundo con más objetos sea automáticamente más difícil.

## Matriz de asignación

| ID | Experimento | Desarrollo / estrés | Extensión obligatoria | Dependencia |
| --- | --- | --- | --- | --- |
| [EXP-001](../experiments/exp_001_sensor_validation/README.md) | Inicialización y lecturas | 01, 07, 08, 10 | 10 mundos, al menos un arranque cada uno | Ninguna |
| [EXP-002](../experiments/exp_002_visual_ranging/README.md) | Pseudo-LiDAR visual | 01, 03, 07, 08, 10 | Revalidar donde falle integración | 001 |
| [EXP-003](../experiments/exp_003_sensor_fusion/README.md) | Fusión visual/proximidad | 01, 07, 08, 10 | Revalidar donde falle integración | 001–002 |
| [EXP-004](../experiments/exp_004_occupancy_mapping/README.md) | Mapa incremental | 01, 03, 07, 10 | Revalidar donde falle integración | 003 |
| [EXP-005](../experiments/exp_005_frontier_exploration/README.md) | Selección de fronteras | 01, 03, 07, 10 | Sus efectos se evalúan en 007 y 010 | 004, 006 |
| [EXP-006](../experiments/exp_006_astar_planning/README.md) | A* y replanificación | Mapas sintéticos + 01, 03, 10 | Sus efectos se evalúan en 007 y 010 | 004 |
| [EXP-007](../experiments/exp_007_outbound_navigation/README.md) | Navegación de ida | 01, 03, 07, 10 | **10 mundos × 3 arranques = 30 corridas** | 005–006 |
| [EXP-008](../experiments/exp_008_victim_rescue/README.md) | Detección y rescate | 01, 07, 08, 09, 10 | Validación transversal en 010 | 002–003, control |
| [EXP-009](../experiments/exp_009_return_navigation/README.md) | Regreso | 01, 03, 07, 10 | Validación transversal en 010 | 004–007 |
| [EXP-010](../experiments/exp_010_full_mission/README.md) | Misión completa | Integración previa en 01, 07, 10 | **10 mundos × 3 arranques = 30 corridas** | 001–009 |

Las dependencias significan que debe existir un componente estable; no obligan a terminar todas las corridas del experimento precedente para comenzar trabajo en paralelo.

## Poses iniciales y repetibilidad

La competición puede comenzar en una pose/orientación aleatoria **detrás** de `STARTLINE` (Z = −8). No utilizar la pose grabada en el WBT (Z = −5.6) como prueba suficiente.

| ID | Posición propuesta (X,Z) m | Orientación prevista | Propósito |
| --- | --- | --- | --- |
| S1 | (−3, −8.8) | Hacia +Z | Arranque lateral con progreso directo |
| S2 | (0, −8.8) | Opuesta a +Z | Probar reorientación completa |
| S3 | (+3, −8.8) | Perpendicular a +Z | Probar orientación y salida lateral |

**Precondición:** medir el radio de seguridad y verificar en Webots que las tres poses están libres, que quedan realmente detrás de la línea, y que la conversión de yaw/brújula corresponde a las orientaciones descriptas. Ajustar en copias de prueba si falla una pose y documentar el motivo. No modificar los WBT originales.

Para cobertura adicional: repetir las corridas con semillas fijas si hay aleatoriedad, documentando cada semilla. La matriz mínima planificada usa S1–S3; no equivale a probar todas las poses posibles.

## Criterios de aceptación generales

**Criterios funcionales obligatorios del enunciado:** llegar al extremo opuesto, rescatar dos víctimas (detenerse a < 1 m, girar 360° e imprimir mensaje), volver al punto de partida, minimizar colisiones y tiempo simulado total ≤ 240 s. El informe debe exponer resultados por escenario.

**Objetivos de ingeniería previos a medición:** evitar declarar como libre una región no observada; verificar accesibilidad por el tamaño del Pioneer II; que A* no atraviese celdas ocupadas o desconocidas; que el controlador priorice parada segura y replanificación; que un rescate no se duplique.

**Métricas adicionales cuya tolerancia se fijará con evidencia:** error métrico de pseudo-LiDAR, probabilidad de ocupación, desviación de pose/retorno, tiempo máximo sin avance, tiempo reservado al regreso. No inventar umbrales numéricos sin ensayos iniciales.

Para evaluar EXP-007 y EXP-010, registrar **por corrida**: escenario, pose/yaw, commit, versión de Webots, tiempo simulado, colisiones, línea de progreso alcanzada, distancia, nº víctimas detectadas/rescatadas, regreso, finalización y motivo de fallo. Diferenciar `not_measured` de `0`.

## Reglas de ejecución

- Ejecutar el controlador **sin conocer la identidad del escenario** ni cargar la geometría de los WBT.
- Las posiciones de víctimas y formas de obstáculos extraídas de `.wbt` solo se usarán para construir casos y evaluar el resultado **fuera del controlador**.
- Crear copias temporales para poses/control de pruebas; no commitear modificaciones sobre los diez mundos de referencia.
- Registrar el commit, configuración, semilla, versión de Webots y cambios en calibración. Medir **tiempo simulado**, no tiempo de reloj.
- Guardar salidas pequeñas en `experiments/exp_NNN.../results/` y documentar cualquier fallo o prueba descartada.
- No anunciar aprobación de un experimento mientras no existan logs, métricas y una conclusión reproducible.

## Próximas acciones

1. Ejecutar EXP-001 en 01 y 07; confirmar cámaras, GPS, brújula, sensores y parada.
2. Verificar las poses S1–S3 en copias de los mundos usados.
3. Reunir muestras visuales y de distancia para EXP-002.
4. Avanzar por la cadena 003 → 004 → 006 → 005, y recién entonces 007–010.

La elección de escenarios puede revisarse ante evidencias; documentar el cambio en [decisions.md](decisions.md).
