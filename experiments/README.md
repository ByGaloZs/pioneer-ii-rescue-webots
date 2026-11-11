# Registro de experimentos

Pruebas aisladas de navegación, percepción y rescate. Los experimentos Webots comparten un controlador de pruebas y una copia de scenario 01, configurados para la hipótesis activa. [Protocolo](../docs/experiments.md) · [Plan maestro](../docs/test_plan.md) · [Catálogo WBT](../docs/scenario_catalog.md) · [Plantilla](_template/README.md).

| Experimento | Tema | Escenarios de desarrollo |
| --- | --- | --- |
| [EXP-001](exp_001_sensor_validation/README.md) | Sensores | 01, 07, 08, 10; revisión inicial en los 10 |
| [EXP-002](exp_002_visual_ranging/README.md) | Pseudo-LiDAR | 01, 03, 07, 08, 10 |
| [EXP-003](exp_003_sensor_fusion/README.md) | Fusión | 01, 07, 08, 10 |
| [EXP-004](exp_004_occupancy_mapping/README.md) | Mapa incremental | 01, 03, 07, 10 |
| [EXP-005](exp_005_frontier_exploration/README.md) | Fronteras | 01, 03, 07, 10 |
| [EXP-006](exp_006_astar_planning/README.md) | A* | Sintéticos + 01, 03, 10 |
| [EXP-007](exp_007_outbound_navigation/README.md) | Ida | 01, 03, 07, 10; luego los 10 |
| [EXP-008](exp_008_victim_rescue/README.md) | Rescate | 01, 07, 08, 09, 10 |
| [EXP-009](exp_009_return_navigation/README.md) | Regreso | 01, 03, 07, 10 |
| [EXP-010](exp_010_full_mission/README.md) | Misión completa | 01, 07, 10; luego los 10 |

**Secuencia:** ejecutar primero EXP-001–010 sobre la copia común de escenario 01 para validar incrementalmente la línea base. Cuando el conjunto funcione, pasar a condiciones representativas y, para ida/misión completa, completar la matriz de los diez escenarios y tres poses verificadas.

**Estado:** EXP-001 `accepted`; EXP-002 `in_progress`; EXP-003–010 `planned`. La etiqueta de estado y las evidencias específicas están en el README de cada experimento. Cada corrida debe producir evidencias y conclusiones, incluso si falla.
