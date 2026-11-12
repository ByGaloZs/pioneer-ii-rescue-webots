# Arquitectura del controlador

Guía para colaboradores: qué hace cada módulo de `controllers/fool_controller/`, qué recibe, qué entrega y dónde
tocar según lo que quieras cambiar. Los parámetros numéricos están en [CONFIGURACION.md](CONFIGURACION.md) y la lógica
de navegación en [NAVEGACION.md](NAVEGACION.md).

## 1. Bucle principal (`fool_controller.cpp`)

1. Crea el `Robot`, el `MotorController` y el `SensorManager`. Si los sensores no se inicializan, termina con error.
2. Lee `controllerArgs` (`parseOptions`) y decide el **modo**, si usa **cámara** y si usa **mapa**.
3. Si el modo es `navigate` y se usa cámara, activa la cámara frontal y crea el `FloorSegmenter`.
4. En cada paso de simulación (`robot.step`):
   - `sensors.read()` → `SensorSnapshot`.
   - `localizer.update(snapshot)` → pose.
   - Según el modo se calcula un `WheelCommand`:
     - `calibrate`: `SensorCalibration::step`.
     - `monitor`: robot parado; imprime `[MONITOR]` cada segundo.
     - `navigate`: segmenta la imagen y llama a `OutboundRun::step`; imprime `[NAV]` cada segundo.
   - `motors.setWheelSpeeds` aplica el comando.
5. Al terminar la simulación detiene los motores y apaga los sensores.

Para añadir un modo nuevo (por ejemplo la vuelta o las víctimas): añade un valor a `StartupMode`
(`RobotConfig.h`), una cadena en `parseOptions`, un `case` en el `switch` del bucle y un módulo de misión análogo a `OutboundRun`.

## 2. Sensores (`sensors/`)

### `SensorManager`
- Obtiene y habilita GPS, brújula, 2 encoders y los 16 sensores de distancia con el paso de tiempo básico del mundo.
- Aplica una **mediana móvil** de `kDistanceMedianWindow` (3) muestras a cada IR para eliminar picos de ruido.
- Convierte cada respuesta en metros con `DistanceSensorCalibration` y calcula los **sectores** (ver abajo).
- La cámara frontal (`camera_f`) solo se activa bajo demanda (`enableFrontCamera`) porque renderizarla cuesta tiempo.
- `printDistanceSensorProfiles()` imprime al arrancar tipo y alcance de cada IR según Webots (útil para comprobar el mundo).

### `SensorSnapshot.h`
Foto de un paso de simulación: tiempo, GPS (x, z), yaw de la brújula, ángulos de los encoders, y para cada IR la
respuesta filtrada, la distancia estimada, si detecta algo y si está **saturado** (demasiado cerca para medir con exactitud).
Incluye los **sectores**, es decir, la distancia mínima de un grupo de sensores (`infinito` = nada detectado):

| Sector | Sensores | Para qué se usa |
|---|---|---|
| `front` | ds0, ds1, ds14, ds15 | Obstáculo delante |
| `frontLeft` / `frontRight` | ds0-1 / ds14-15 | Lado del obstáculo frontal |
| `diagonalLeft` / `diagonalRight` | ds1-2 / ds13-14 | Esquinas delanteras |
| `left` / `right` | ds2-4 / ds11-13 | Paredes laterales |
| `rearLeft` / `rearRight` | ds5-7 / ds8-10 | Margen para retroceder y girar |

### `DistanceSensorCalibration`
`fromLookupTable()` invierte la tabla `[distancia, respuesta, ruido]` del sensor activo en Webots por interpolación
lineal y multiplica por el **factor de reflexión** (`kObstacleReflectionFactor` = 0,52, obstáculos y paredes cian).
Devuelve distancia, si detecta, y si la respuesta está en la zona de saturación cercana (en cuyo caso la distancia
es un límite superior, no una medida exacta).

## 3. Localización (`localization/Localizer`)

- **Orientación:** brújula (`yaw`), más `kCompassYawOffsetRad`.
- **Posición:** GPS si `kUseGpsForPosition` y es válido; si no, odometría con encoders (`kWheelRadiusM`) y yaw de la brújula.
- Mantiene además, solo para diagnóstico, una odometría pura (`odometryPose`), un yaw solo por encoders
  (`encoderYawRad`, usa `kWheelSeparationM`), la **distancia total recorrida** (`travelledM`) y el **giro acumulado sin
  envolver** (`accumulatedYawRad`, sirve para el giro de 360° y para las futuras fases de víctimas).
- `initialize()` fija el origen con la primera lectura válida de GPS + brújula + encoders; hasta entonces `initialized()` es falso
  y la misión no arranca.

## 4. Percepción (`perception/FloorSegmenter`)

Convierte la imagen de la cámara frontal en un **perfil de espacio libre**: para ~60 columnas de la imagen, a qué distancia
(delante y a la izquierda, en metros desde el centro del robot) termina el suelo libre.

1. **Suelo de referencia:** mediana del color de la franja inferior (8 %) del 50 % central de la imagen.
2. Si el brillo de ese suelo es menor que `kMinFloorBrightness` (12/255), el perfil sale **no válido** y el robot navega solo con IR.
3. Cada píxel se clasifica **relativo al suelo** con cocientes g/r y b/r: verde = víctima (×1,35), cian = obstáculo (×1,15), resto = suelo
   (incluye líneas negras/amarillas y sombras). Así funciona con las iluminaciones tan distintas de los 10 mundos.
4. Cada columna se recorre desde abajo hasta el horizonte; el primer obstáculo (2 píxeles seguidos) fija el límite.
   La distancia se obtiene por geometría de suelo plano (cámara a 0,255 m de altura, horizontal).
5. Más allá de `kTrustedRangeM` (3,5 m, por la niebla) no se da nada por libre; píxeles casi negros se tratan como desconocidos.
6. `FreeSpaceProfile::corridorClearance(rumbo, semiancho, máx)` devuelve la distancia libre en un pasillo del ancho del robot
   hacia un rumbo relativo. Es lo que usa el navegador para elegir hueco y velocidad.
7. También cuenta `victimPixels` (píxeles verdes), reservado para la fase de víctimas, y `floorBrightness` (se imprime como `luz=`).

## 5. Mapa (`mapping/OccupancyGrid.h`)

Mapa de ocupación de **100 x 200 celdas de 10 cm** (x de −5 a 5, z de −10 a 10). Cada celda guarda una evidencia entera:

| Evento | Efecto |
|---|---|
| Impacto IR en la celda (a < 0,19 m) | +6 |
| Rayo IR que la atraviesa sin detectar | −1 |
| Celda bajo la huella del robot (radio 0,15 m) | mínimo (−20), seguro libre |

Una celda es **ocupada** con evidencia ≥ 10 (dos impactos). Las paredes exteriores (|x| > 4,8 o |z| > 9,8) se marcan desde el
principio y no se pueden borrar. Solo usa IR + pose, por lo que **no depende de la luz**.

Además cuenta las **visitas** de cada celda (cuántas veces entró el robot) y expone `version()`, que cambia cada vez que una celda pasa
de libre a ocupada o al revés; el navegador la usa para saber cuándo replanificar. `writeAscii()` genera el dibujo para el informe.
La geometría de los 16 IR (posición y ángulo en el robot) está tabulada dentro de la clase.

## 6. Planificación (`planning/PathPlanner.h`)

A\* de 8 vecinos sobre el mapa, desde la celda del robot hasta cualquier celda con `z ≥ zTarget` (o `≤` si la meta está hacia −z).
- **Optimista:** lo desconocido cuenta como libre; al descubrir una pared el mapa cambia y se replanifica.
- **Celdas prohibidas** (letales): a menos de 0,25 m de una celda ocupada (el robot no cabe). Se permite salir de ellas cerca del robot.
- **Costes extra:** hasta 0,08 m por celda junto a obstáculos (radio 0,5 m, anima a ir por el centro de los pasos) y 0,04 m por
  cada visita previa (máx. 5), para no repetir callejones.
- No corta esquinas en diagonal entre dos celdas ocupadas.
- Entrega la ruta, la longitud restante (`remainingM`) y `lookaheadYaw()`: el rumbo hacia el punto más lejano de la ruta a ≤ 0,8 m
  con línea de visión libre.

## 7. Navegación (`navigation/GoalNavigator`)

Máquina de estados que convierte pose + sensores + plan en un `WheelCommand`. Estados: `ALIGN`, `GO_TO_GOAL`, `FOLLOW_WALL_L/R`,
`REVERSE_OUT`, `TURN_AROUND`, `RECOVER`. Se explica con detalle en [NAVEGACION.md](NAVEGACION.md).

## 8. Misión (`mission/OutboundRun`)

Define la **prueba de ida** y es el único sitio que conoce la meta y las métricas:
- Al primer paso útil guarda la pose inicial, actualiza el mapa y fija la meta (`kOutboundGoalZ` = 8,6) en el navegador.
- En cada paso: actualiza métricas de proximidad, comprueba **éxito** (`z ≥ 8,2`) o **tiempo agotado** (240 s), actualiza el mapa,
  pide el comando al navegador y lo pasa por el `SafetyGuard`.
- `finish()` imprime `[RESULT]`, añade la fila al CSV (apartando un CSV de formato antiguo como `outbound_results_old.csv`) y guarda el mapa.
- Métricas: líneas cruzadas (8 en total: negras en z = −6…6 cada 2 m y amarilla en z = 8), recorrido por encoders, obstáculos rodeados,
  cambios de lado, atascos, frenos de emergencia, casi-contactos, distancia IR mínima, % de tiempo con cámara utilizable y número de replanificaciones.

## 9. Control (`control/`)

- **`WheelCommand`**: par (izquierda, derecha) en rad/s. Es el lenguaje común entre navegación, calibración, seguridad y motores.
- **`SafetyGuard`**: filtro final. Si ds0, ds1, ds14 o ds15 ven algo a < 6 cm (`kSafetyStopM`) y el comando avanza, le resta la componente
  de avance (el robot aún puede girar o retroceder). Cuenta *episodios* (no pasos) en `interventions()`.
- **`MotorController`**: pone los motores en modo velocidad (posición infinita), satura cada rueda a ±10 rad/s y expone `stop()`.

## 10. Calibración (`calibration/SensorCalibration`)

Rutina del paso 1 (modo `calibrate`). Ver [CALIBRACION.md](CALIBRACION.md).

## 11. Utilidades (`common/`)

- `Angles.h`: `wrap`, `toDeg`, `toRad`, `yawTowards(x, z, tx, tz)`.
- `RunOutput.h`: `directory()` (crea `../../results`, relativo al directorio del controlador que usa Webots), `timestamp()` y
  `resolveTag()` con prioridad `tag=` en `controllerArgs` > mundo detectado (variable `WEBOTS_WORLD` o la línea de órdenes del
  proceso padre de Webots) > `s??`.

## Dónde tocar según lo que quieras hacer

| Quiero… | Archivo |
|---|---|
| Cambiar velocidades, umbrales o distancias | `config/RobotConfig.h` |
| Cambiar cuándo se rodea un obstáculo o cómo se rodea | `navigation/GoalNavigator.cpp` (`goToGoal`, `followWall`) |
| Cambiar los costes del camino (más margen a las paredes, etc.) | `config/RobotConfig.h` sección `mapping` y `planning/PathPlanner.h` (`buildCostmap`) |
| Adaptar la cámara a una luz nueva | `config/RobotConfig.h` sección `camera` y `perception/FloorSegmenter.cpp` |
| Añadir una métrica al CSV | `mission/OutboundRun.cpp` (`header` y `row` deben cambiar a la vez) |
| Implementar la vuelta o las víctimas | Nuevo módulo en `mission/` + nuevo modo en `fool_controller.cpp` |
