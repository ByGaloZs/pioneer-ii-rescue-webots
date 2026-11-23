# Cambios respecto a los archivos de la práctica

La profesora permite cualquier cambio sobre los archivos entregados con la Práctica 06 siempre que se documente (incluidos el GPS, el alcance de
los sensores de distancia y su `lookupTable`). Esta página recoge **todos** los cambios hechos en los mundos y en el robot, por qué se hicieron
y cómo deshacerlos. La comparación se ha hecho contra los mundos originales (commit `ce6fa5e`, «restore original Practice 06 WBT scenarios»),
sin contar las diferencias de formato que introduce Webots al guardar (decimales reescritos).

## Resumen

| Archivo | Cambio | ¿Afecta al comportamiento? |
|---|---|---|
| `worlds/final_scenario01…10.wbt` | GPS sin cuantizar: se elimina `resolution 3` (Webots usa entonces `-1`) | **Sí**: el robot conoce su posición con precisión |
| `worlds/final_scenario01…10.wbt` | `controllerArgs ["tag=NN"]` en el `Pioneer2` | No: solo etiqueta las filas del CSV y los mapas |
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

No se ha modificado: se usan los 16 sensores de distancia, sus alcances y sus `lookupTable` tal como vienen. El controlador invierte la tabla
de cada sensor (`DistanceSensorCalibration`) y corrige la atenuación por color de Webots (`kObstacleReflectionFactor`); por eso el alcance
efectivo frente a obstáculos cian es ≈ 0,21 m. Aunque está permitido cambiar el alcance y la `lookupTable`, no ha hecho falta: los obstáculos
lejanos los detecta la cámara esférica.

Si en el futuro se cambian, hay que documentarlo aquí y volver a ejecutar `calibrate`, porque `kObstacleReflectionFactor` y las distancias
máximas de `RobotConfig.h` dependen de esas tablas.

## 4. Interpretaciones del enunciado

No son cambios en los archivos, pero condicionan cómo se mide el éxito:

- **«A menos de 1 m de la víctima»** se mide de centro a centro (robot–cilindro). El robot para a 0,85 m (`kStopDistanceM`).
- **La vuelta termina** al cruzar `z ≤ −8,2`, justo después de la línea amarilla de salida (`z = −8`).
- **El límite de 240 s** es tiempo simulado de Webots.
