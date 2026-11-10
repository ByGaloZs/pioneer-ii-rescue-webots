# Escenarios originales — Práctica 06

Los diez archivos `final_scenario01.wbt` a `final_scenario10.wbt` de esta carpeta son **copias byte por byte** de `Práctica06_src.zip`, el material original proporcionado para el trabajo final. Se sustituyeron las versiones de prueba utilizadas inicialmente; no se modificó la geometría de los mundos originales.

## Integridad

| Escenario | Versión declarada en archivo | Git blob SHA-1 |
| --- | --- | --- |
| `final_scenario01.wbt` | R2020b | `fea5f899b9c10c312f43b55caae9b30079a0fe72` |
| `final_scenario02.wbt` | R2020b | `feca6a0278dc790f837e37275faa4e9999043651` |
| `final_scenario03.wbt` | R2020b | `31f4a5252c28f4a49c78ed1efd54470e75698efa` |
| `final_scenario04.wbt` | R2020b | `b12a61699dccf5c1b4678d3f173b34033444f519` |
| `final_scenario05.wbt` | R2020b | `2f7235405d9d8648db254dc66731c57e0fcd114f` |
| `final_scenario06.wbt` | R2018b | `72cfbc06e705515e22b6ed9fd4b1af3b37a05e09` |
| `final_scenario07.wbt` | R2018b | `617f6e698359e26329b3ca98f2cd8cd2a134a18b` |
| `final_scenario08.wbt` | R2020b | `33dff6244df55fee2255aad80e622991f718d305` |
| `final_scenario09.wbt` | R2020b | `dcfd065d31de5cb0d31d7acdff908a469f7fe568` |
| `final_scenario10.wbt` | R2020b | `9e9315959277d2afcd4800ec71ebe2275f8f3f0a` |

Los hashes son los identificadores Git del contenido original y se verificaron al subir los archivos; no representan un test de simulación.

## Inicio y compatibilidad

- Los diez archivos inicializan al robot en **`translation 0 0 -9.05`** y **`rotation 0 1 0 3.141592653589793`**.
- Los mundos originales usan el nodo **`Pioneer2`**, controlador **`fool_controller`**, brújula, GPS y cámaras `camera_s` y `camera_f` en `extensionSlot`.
- El repositorio también contiene `protos/Pioneer2_Robot.proto`, **un nombre de robot distinto**. No se ha convertido automáticamente ninguno de los mundos para utilizar ese PROTO.
- Los archivos 06 y 07 declaran `#VRML_SIM R2018b utf8`; los demás declaran `R2020b`. Su compatibilidad con la versión concreta de Webots del usuario **debe verificarse**.
- Si un Webots moderno no reconoce `Pioneer2`, preparar una adaptación en una **copia de prueba** claramente identificada; no modificar silenciosamente los originales.
- Para experimentar con posiciones aleatorias de inicio, generar copias y registrar pose, ángulo y escenario. Los originales deben permanecer sin cambios.

## Validación pendiente

Abrir los diez escenarios en Webots, verificar la disponibilidad del robot y que los dispositivos conserven los nombres y características esperados por `SensorManager`. Posteriormente ejecutar `EXP-001`. No se afirma que los mundos originales ya se hayan ejecutado en Webots.

Consulta también el [catálogo estructural](../docs/scenario_catalog.md) y el [plan de pruebas](../docs/test_plan.md).

## Mundos de prueba EXP-002 (no son escenarios originales)

Se incluyen tres copias de `final_scenario01.wbt` de la versión de desarrollo actual: [`exp002_camera_01_tilt45.wbt`](exp002_camera_01_tilt45.wbt), [`exp002_camera_01_tilt30.wbt`](exp002_camera_01_tilt30.wbt) y [`exp002_camera_01_horizontal.wbt`](exp002_camera_01_horizontal.wbt). Usan la misma pose del robot `(X=0, Z=-8.8, yaw=0)` y GPS `resolution -1`; solo difiere la rotación de `camera_s`. No usar estas copias como prueba de que los diez mundos originales permiten alterar sensores. Consulta [EXP-002](../experiments/exp_002_visual_ranging/README.md).

### Fase 2: posiciones de calibración (solo copias)

Se incorporaron `exp002_calibration_01_p1.wbt`, `exp002_calibration_01_p2.wbt` y `exp002_calibration_01_p3.wbt` para el estudio del muro posterior del escenario 01. Sus poses Z son −9.0, −8.8 y −8.3 m; todos usan cámara a −30°, GPS sin cuantización y no incluyen los cuatro visuales de diagnóstico dentro de la cámara. El robot incluye `controllerArgs` para etiquetar la corrida, no para suministrar geometría. Los diez `final_scenario*.wbt` permanecen intactos. Consulta [fase 2 de EXP-002](../experiments/exp_002_visual_ranging/README.md).
