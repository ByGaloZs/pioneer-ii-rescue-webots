# Escenarios originales — Práctica 06

Los diez archivos `final_scenario01.wbt` a `final_scenario10.wbt` provienen de `Práctica06_src.zip`, el material original proporcionado para el trabajo final. Por decisión del proyecto, su única modificación común es `GPS.resolution: 3 → -1`, para desactivar la cuantización del GPS. El encabezado, la pose, el nodo `Pioneer2`, cámaras, luces, geometría de obstáculos y demás contenido original se conservan.

## Integridad

| Escenario | Versión | SHA-1 original importado | SHA-1 WBT actual |
| --- | --- | --- | --- |
| `final_scenario01.wbt` | R2020b | `fea5f899b9c10c312f43b55caae9b30079a0fe72` | `6f60aad0b408a080316cde686917f5bc7b2453e6` |
| `final_scenario02.wbt` | R2020b | `feca6a0278dc790f837e37275faa4e9999043651` | `c39dbf07b3d30549631c6488e9b62ce17dfdb361` |
| `final_scenario03.wbt` | R2020b | `31f4a5252c28f4a49c78ed1efd54470e75698efa` | `a0802398d09fa9aa1b3a8d26c9a20c5b0dd7c52a` |
| `final_scenario04.wbt` | R2020b | `b12a61699dccf5c1b4678d3f173b34033444f519` | `a658ce33b38b7e9fc42a9cc43e3e86986fd96878` |
| `final_scenario05.wbt` | R2020b | `2f7235405d9d8648db254dc66731c57e0fcd114f` | `bd444ffc2c070e399d8c435688b89ae226978c84` |
| `final_scenario06.wbt` | R2018b | `72cfbc06e705515e22b6ed9fd4b1af3b37a05e09` | `2fd705f8f66d7e3b613fcb7daf0ccd9fdb056d15` |
| `final_scenario07.wbt` | R2018b | `617f6e698359e26329b3ca98f2cd8cd2a134a18b` | `3d4fa85f81934705172cd02e64af1b6b6cbd4a56` |
| `final_scenario08.wbt` | R2020b | `33dff6244df55fee2255aad80e622991f718d305` | `c409c94d4249e66ae33f8d4cfa62c0b27bcc01da` |
| `final_scenario09.wbt` | R2020b | `dcfd065d31de5cb0d31d7acdff908a469f7fe568` | `6e1c918028975def34b26bf4df6bc35b638fcbc1` |
| `final_scenario10.wbt` | R2020b | `9e9315959277d2afcd4800ec71ebe2275f8f3f0a` | `f12f41126a5a70c5714898245c1f86556ba07981` |

Los hashes originales conservan la procedencia del material de entrada; los hashes actuales permiten verificar las copias tras la modificación autorizada del GPS. Los hashes no representan un test de simulación.

## Inicio y compatibilidad

- Los diez archivos inicializan al robot en **`translation 0 0 -9.05`** y **`rotation 0 1 0 3.141592653589793`**.
- Todos los WBT de esta carpeta configuran el GPS con `resolution -1` (sin cuantización).
- Los mundos originales usan el nodo **`Pioneer2`**, controlador **`fool_controller`**, brújula, GPS y cámaras `camera_s` y `camera_f` en `extensionSlot`.
- El repositorio también contiene `protos/Pioneer2_Robot.proto`, **un nombre de robot distinto**. No se ha convertido automáticamente ninguno de los mundos para utilizar ese PROTO.
- Los archivos 06 y 07 declaran `#VRML_SIM R2018b utf8`; los demás declaran `R2020b`. Su compatibilidad con la versión concreta de Webots del usuario **debe verificarse**.
- Si un Webots moderno no reconoce `Pioneer2`, preparar una adaptación en una **copia de prueba** claramente identificada; no modificar silenciosamente otros campos de estos mundos.
- Para experimentar con posiciones aleatorias de inicio, generar copias y registrar pose, ángulo y escenario. Mantener intactos el GPS `-1` y el resto de los campos canónicos.

## Validación pendiente

Abrir los diez escenarios en Webots, verificar la disponibilidad del robot y que los dispositivos conserven los nombres y características esperados por `SensorManager`. Posteriormente ejecutar `EXP-001`. No se afirma que los mundos originales ya se hayan ejecutado en Webots.

Consulta también el [catálogo estructural](../docs/scenario_catalog.md) y el [plan de pruebas](../docs/test_plan.md).

## Mundo único de pruebas (copia, no escenario original)

`experiment_test_scenario01.wbt` es la única copia reutilizable derivada de `final_scenario01.wbt` para todos los experimentos. El generador `experiments/exp_002_visual_ranging/src/create_pilot_world.py` la regenera con GPS `-1`, pose, sensores y trial del experimento activo. Selecciona `experiment_test_controller`; cada configuración reemplaza a la anterior en este único archivo, mientras cada corrida conserva su carpeta de resultados. Consulta el [protocolo experimental](../docs/experiments.md) y el [README de EXP-002](../experiments/exp_002_visual_ranging/README.md).
