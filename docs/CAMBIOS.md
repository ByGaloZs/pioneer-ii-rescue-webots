# Cambios respecto a los archivos de la práctica

La profesora permite cualquier cambio sobre los archivos entregados con la Práctica 06 siempre que se documente (incluidos el GPS, el alcance de
los sensores de distancia y su `lookupTable`). Esta página recoge **todos** los cambios hechos en los mundos y en el robot, por qué se hicieron
y cómo deshacerlos. La comparación se ha hecho contra los mundos originales (commit `ce6fa5e`, «restore original Practice 06 WBT scenarios»),
sin contar las diferencias de formato que introduce Webots al guardar (decimales reescritos).

## Resumen

| Archivo | Cambio | ¿Afecta al comportamiento? |
|---|---|---|
| `worlds/final_scenario01…10.wbt` | El robot es `Pioneer2_Robot` (el de `protos/`) en lugar del `Pioneer2` de la biblioteca de Webots | **Sí**: los 8 IR delanteros ven hasta ≈ 0,52 m en vez de ≈ 0,21 m |
| `worlds/final_scenario01…10.wbt` | GPS sin cuantizar: se elimina `resolution 3` (Webots usa entonces `-1`) | **Sí**: el robot conoce su posición con precisión |
| `worlds/final_scenario01…10.wbt` | `controllerArgs` en el robot (`nofiles`, `nolive`, `mission`, `tag=NN`) | No cambia la misión: solo evita escribir archivos y etiqueta la prueba |
| `worlds/final_scenario01…10.wbt` | `Viewpoint` con vista cenital | No: solo la cámara del editor |
| `worlds/final_scenario06/07.wbt` | Cabecera `R2018b` → `R2020b` (Webots lo actualiza al guardar) | No |
| `protos/Pioneer2_Robot.proto` | **Sin cambios**: sensores de distancia, alcances y `lookupTable` originales | — |

## 1. GPS sin cuantizar (`resolution -1`)

**Original:** el nodo `GPS` del `extensionSlot` de cada mundo tenía `resolution 3`, es decir, la posición llegaba redondeada a múltiplos de 3 m.

**Ahora:** el campo `resolution` se ha quitado en los 10 mundos. En Webots R2020b el valor por defecto es `-1`, que significa **sin cuantizar**
(posición exacta del simulador).

**Por qué:** el controlador construye un mapa de ocupación de 10 cm por celda y planifica con A\* (ver [NAVEGACION.md](NAVEGACION.md)). Con saltos
de 3 m el mapa quedaría desplazado varias decenas de celdas, el A\* planificaría sobre obstáculos que no están donde cree y no se podría
comprobar la parada a menos de 1 m de cada víctima. Con el GPS exacto la posición estimada de las víctimas quedó a ≤ 0,11 m de la real en 19
de 20 rescates (ver [RESULTADOS.md](RESULTADOS.md)).

**Cómo usarlo:** `kUseGpsForPosition = true` en `config/RobotConfig.h` (valor actual). La posición sale del GPS y la orientación de la brújula.

**Cómo deshacerlo:** volver a poner `resolution 3` en el nodo `GPS` de cada mundo y `kUseGpsForPosition = false`: la posición saldrá entonces
solo de la odometría (encoders + brújula), calibrada en [CALIBRACION.md](CALIBRACION.md). Esa configuración **no se ha probado** con la misión
completa.

## 2. `controllerArgs` y vista del editor

- Cada mundo lleva `controllerArgs ["tag=NN"]`. El controlador ya detecta el mundo por el nombre del archivo; la etiqueta solo asegura que
  las filas del CSV y los mapas se llamen `s01`…`s10`. Vaciar el campo sigue ejecutando la misión completa (ver la tabla de argumentos del
  [README](../README.md)).
- La pose de salida del robot es la original en los 10 mundos (`translation 0 0 -9.05`, mirando a +Z).
- El `Viewpoint` se cambió a una vista cenital de la arena para seguir la misión. No afecta a la simulación.

## 3. Robot (`Pioneer2_Robot.proto`)

**Original:** los mundos declaraban el robot como `Pioneer2 { … }`. Webots busca el PROTO por el nombre del nodo y en `protos/` solo está
`Pioneer2_Robot.proto` (que declara `PROTO Pioneer2_Robot`), así que cargaba el `Pioneer2` de su propia biblioteca.

**Ahora:** los 10 mundos usan `Pioneer2_Robot { … }`, es decir, el robot de `protos/`, que es el que se entrega. Los campos son los mismos
(pose, `controller`, `controllerArgs`, `extensionSlot`), así que no hay que tocar nada más.

**Diferencia entre los dos robots:** solo las `lookupTable` de los 8 IR delanteros (ds0–ds3, ds12–ds15): 1 m en `Pioneer2_Robot` frente a
0,4 m en el `Pioneer2` de Webots. Con la atenuación de Webots para el cian (factor 0,52), eso da ≈ 0,52 m frente a ≈ 0,21 m. Los laterales y
traseros (0,4 m), el tipo de sensor (`infra-red`), las ruedas y el resto del robot son iguales.

**Efecto en el controlador:** lee la tabla de cada sensor del propio robot (`DistanceSensorCalibration`), así que las distancias en metros
siguen siendo correctas. El mapa solo confía en los IR hasta 0,19 m (`kIrMaxRangeM`), igual que antes. Lo que cambia es que la marcha lenta
de `GO_TO_GOAL` (con cualquier IR delantero detectando algo) se activa antes.

**Resultados:** las tandas de [RESULTADOS.md](RESULTADOS.md) hasta el 17-11-2026 se hicieron con el `Pioneer2` de Webots; la tanda final se
hace con `Pioneer2_Robot`.

El PROTO en sí no se ha modificado.

Si en el futuro se cambian, hay que documentarlo aquí y volver a ejecutar `calibrate`, porque `kObstacleReflectionFactor` y las distancias
máximas de `RobotConfig.h` dependen de esas tablas.

## 4. Interpretaciones del enunciado

No son cambios en los archivos, pero condicionan cómo se mide el éxito:

- **«A menos de 1 m de la víctima»** se mide de centro a centro (robot–cilindro). El robot para a 0,85 m (`kStopDistanceM`).
- **La vuelta termina** al cruzar `z ≤ −8,2`, justo después de la línea amarilla de salida (`z = −8`).
- **El límite de 240 s** es tiempo simulado de Webots.
