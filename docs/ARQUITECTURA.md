# Arquitectura del controlador

Guía para colaboradores: qué hace cada módulo de `controllers/fool_controller/`, qué recibe, qué entrega y dónde
tocar según lo que quieras cambiar. Los parámetros numéricos están en [CONFIGURACION.md](CONFIGURACION.md) y la lógica
de navegación en [NAVEGACION.md](NAVEGACION.md).

## 1. Bucle principal (`fool_controller.cpp`)

1. Crea el `Robot`, el `MotorController` y el `SensorManager`. Si los sensores no se inicializan, termina con error.
2. Lee `controllerArgs` (`parseOptions`) y decide el **modo** (`mission` por defecto, `navigate`, `calibrate`, `monitor`, `capture`) y las opciones
   (`nocam`, `nomap`, `nosphere`, `nolive`, `snap`, `tag=`). Imprime `[STARTUP] controllerArgs recibidos`.
3. En `mission`/`navigate` activa la cámara frontal (si no hay `nocam`) y crea el `FloorSegmenter`; activa la cámara esférica (si no hay `nosphere`)
   y comprueba que tiene la geometría medida (`SphericalView::matches`).
4. En cada paso de simulación (`robot.step`):
   - `sensors.read()` → `SensorSnapshot`; `localizer.update(snapshot)` → pose.
   - Según el modo se calcula un `WheelCommand`:
     - `mission`: segmenta la imagen frontal y llama a `RescueMission::step`; imprime `[MISION]` cada segundo.
     - `navigate`: igual con `OutboundRun::step`; imprime `[NAV]` cada segundo.
     - `calibrate`: `SensorCalibration::step`. `monitor`: robot parado, imprime `[MONITOR]`. `capture`: robot parado, `SnapshotRecorder` guarda capturas.
   - `motors.setWheelSpeeds` aplica el comando.
5. Al terminar la simulación detiene los motores y apaga los sensores.

Para añadir un modo nuevo: un valor en `StartupMode` (`RobotConfig.h`), una cadena en `parseOptions`, un `case` en el `switch` del bucle y un módulo en `mission/`
(`RescueMission` es el modelo; si es solo cabecera no hay que tocar el `Makefile`).

## 2. Sensores (`sensors/`)

### `SensorManager`
- Obtiene y habilita GPS, brújula, 2 encoders y los 16 sensores de distancia con el paso de tiempo básico del mundo.
- Aplica una **mediana móvil** de `kDistanceMedianWindow` (3) muestras a cada IR para eliminar picos de ruido.
- Convierte cada respuesta en metros con `DistanceSensorCalibration` y calcula los **sectores** (ver abajo).
- Las cámaras frontal (`camera_f`, `enableFrontCamera`) y esférica (`camera_s`, `enableSphericalCamera`) solo se activan bajo demanda porque renderizarlas cuesta tiempo.
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
  envolver** (`accumulatedYawRad`; el giro de 360° de las víctimas mide su propio giro acumulado en `RescueMission`).
- `initialize()` fija el origen con la primera lectura válida de GPS + brújula + encoders; hasta entonces `initialized()` es falso
  y la misión no arranca.

## 4. Percepción (`perception/`)

### `FloorSegmenter` (cámara frontal)

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
7. También cuenta `victimPixels` y `floorBrightness` (se imprime como `luz=`).
8. **Víctimas:** una columna cuyo primer no-suelo es verde (y que no llega al borde inferior de la imagen, donde la base queda fuera de la imagen) se marca `victim`;
   columnas verdes consecutivas forman un bloque (`VictimBlob`) con el punto de la base más cercano al robot. El verde exige además ser más brillante que el suelo
   (`kMinGreen`, `kFloorGreenFactor`), para descartar ruido sobre las líneas negras.

### `SphericalView.h` (cámara esférica)
Geometría medida con capturas reales de Webots: montaje (0,44 m de altura, 5 cm delante y 2,5 cm a la derecha del centro, mirando al suelo), proyección
cilíndrica alrededor del eje, máscara del propio robot (`isRobot`) y conversión píxel → punto del suelo (`pixelToGround`). Zona ciega: ~0,45 m.

### `VictimDetector.h` (víctimas)
- `isVictimGreen(r, g, b, minGreen)`: verde claramente por encima de rojo y azul y con un mínimo absoluto.
- `fromFront(...)`: una observación por bloque verde de la cámara frontal (rango aceptado `kFrontMinRangeM`…`kFrontMaxRangeM`, 1–6 m).
- `fromSphere(...)`: píxel verde más cercano por dirección (bins de 3°) → base del cilindro; exige que la base se vea (no tapada por la silueta del robot) y rango
  entre `kSphereMinRangeM` (0,7 m) y `kSphereMaxRangeM` (3 m). El centro estimado está 0,3 m más allá de la base (radio del cilindro).
- `VictimTracker`: agrupa las observaciones en coordenadas del mundo (media ponderada; las cercanas, `< kCloseRangeM`, mandan sobre las lejanas), confirma una
  candidata con `kConfirmFrames` imágenes, fusiona duplicadas, ignora restos "detrás" de una víctima ya vista de cerca y recuerda cuáles están rescatadas o fallidas.

## 5. Mapa (`mapping/`)

### `OccupancyGrid.h`

Mapa de ocupación de **100 x 200 celdas de 10 cm** (x de −5 a 5, z de −10 a 10). Cada celda guarda una evidencia entera:

| Evento | Efecto |
|---|---|
| Impacto IR en la celda (a < 0,19 m) | +6 |
| Rayo IR que la atraviesa sin detectar | −1 |
| Celda bajo la huella del robot (radio 0,15 m) | mínimo (−20), seguro libre |

Una celda es **ocupada** con evidencia ≥ 10 (dos impactos). Las paredes exteriores (|x| > 4,8 o |z| > 9,8) se marcan desde el
principio y no se pueden borrar. Con solo IR + pose **no depende de la luz**; la cámara esférica añade evidencia por encima (`SphericalMapper`, más abajo).

Además cuenta las **visitas** de cada celda (cuántas veces entró el robot) y expone `version()`, que cambia cada vez que una celda pasa
de libre a ocupada o al revés; el navegador la usa para saber cuándo replanificar. `writeAscii()` genera el dibujo para el informe.
La geometría de los 16 IR (posición y ángulo en el robot) está tabulada dentro de la clase. `markDisc` marca un disco como ocupado (se usa con las víctimas
rescatadas) y `writeLive` vuelca el mapa para el visor (con las víctimas en la línea `victims=`).

### `SphericalMapper.h`
Lanza 90 rayos sobre el suelo alrededor del robot (0,5–1,6 m) y marca una pared donde el gris del suelo pasa a cian; cada impacto suma poca evidencia
(`kSphereHitNear/Far`) y nunca borra lo visto por los IR. Se descarta la imagen si el suelo se ve más oscuro de `kSphereMinBrightness` (25/255). El verde de
víctima **no** se trata como pared.

## 6. Planificación (`planning/PathPlanner.h`)

A\* de 8 vecinos sobre el mapa, desde la celda del robot hasta una **meta de línea** (`plan`: cualquier celda con `z ≥ zTarget`, o `≤` hacia −z) o una **meta puntual** (`planPoint`: celdas a ≤ r de un punto, para ir hasta una víctima).
- **Optimista:** lo desconocido cuenta como libre; al descubrir una pared el mapa cambia y se replanifica.
- **Celdas prohibidas** (letales): a menos de 0,25 m de una celda ocupada (el robot no cabe). Se permite salir de ellas cerca del robot.
- **Costes extra:** hasta 0,08 m por celda junto a obstáculos (radio 0,5 m, anima a ir por el centro de los pasos) y 0,04 m por
  cada visita previa (máx. 5), para no repetir callejones. Con `setPreferVisited(true)` (la vuelta) se invierte: las celdas **no** recorridas cuestan 0,06 m extra, así que prefiere el camino ya andado.
- No corta esquinas en diagonal entre dos celdas ocupadas.
- Entrega la ruta, la longitud restante (`remainingM`) y `lookaheadYaw()`: el rumbo hacia el punto más lejano de la ruta a ≤ 0,8 m
  con línea de visión libre.

## 7. Navegación (`navigation/GoalNavigator`)

Máquina de estados que convierte pose + sensores + plan en un `WheelCommand`, hacia una línea (`setGoalLine`) o un punto (`setGoalPoint`/`moveGoalPoint`). Estados: `ALIGN`, `GO_TO_GOAL`, `FOLLOW_WALL_L/R`,
`REVERSE_OUT`, `TURN_AROUND`, `RECOVER`. Se explica con detalle en [NAVEGACION.md](NAVEGACION.md).

## 8. Misión (`mission/`)

### `RescueMission.h` (modo `mission`, el predeterminado)
Máquina de estados de la misión completa; es el único sitio que conoce las metas, los tiempos y las métricas:

| Fase | Qué hace | Termina cuando |
|---|---|---|
| `IDA` | `GoalNavigator` hacia la línea z = 8,4 (`kOutboundGoalZ`) | z ≥ 8,2 (`kOutboundDoneZ`) |
| `BUSCAR` | Elige: volver, ir a una víctima confirmada o barrer el pasillo del fondo (z = 8,9; x = ±3,2 y 0) | ver abajo |
| `IR_A_VICTIMA` | A* a meta puntual hasta 1,15 m de la víctima (sin cámara en el navegador) | distancia ≤ 1,25 m, o 45 s (se abandona) |
| `ACERCAR` | Avance directo hasta `kStopDistanceM` (0,85 m del centro); retrocede si empieza a < 0,7 m | distancia ≤ 0,85 m, IR frontal < 12 cm, o 15 s; o `mustReturn` si aún está a ≥ 1 m (vuelve) |
| `GIRO_360` | Gira a la izquierda sin avanzar hasta acumular 6,40 rad; imprime `[VICTIMA n]` | giro completo (6,40 rad), o 8 s: entonces la víctima se marca fallida, no rescatada |
| `VUELTA` | `GoalNavigator` hacia z = −8,6 con `setPreferVisited(true)` | z ≤ −8,2 (`kReturnDoneZ`) |

Reglas de decisión en `BUSCAR`: vuelve si ya hay 2 rescatadas, si `mustReturn` (tiempo transcurrido + reserva de vuelta + margen > 240 s, con reserva =
`kReturnReserveFactor` × tiempo de ida + `kReturnReserveFixedS` y margen `kReturnSafetyMarginS` = 12 s) o si agotó los tres tramos de barrido; si no, va a la
víctima confirmada más cercana. Cada paso: actualiza mapa (IR + esférica), detecta víctimas, comprueba el límite de 240 s, ejecuta la fase y filtra por `SafetyGuard`.
Salida: `[MISION]`/`[VICTIMA]`/`[RESULT]` en consola, `results/mission_results.csv`, `results/map_<tag>_mission.txt` y `live_map.txt`.

### `OutboundRun` (modo `navigate`, solo la ida)
Prueba del paso 2: meta `kOutboundGoalZ`, éxito en z ≥ 8,2, tiempo agotado a 240 s. `finish()` imprime `[RESULT]`, **añade una fila** a `results/outbound_results.csv`
(si el encabezado cambia, aparta el archivo anterior como `outbound_results_old_<fecha>.csv`) y guarda `map_<tag>_<cam|nocam>.txt`.

## 9. Control (`control/`)

- **`WheelCommand`**: par (izquierda, derecha) en rad/s. Es el lenguaje común entre navegación, calibración, seguridad y motores.
- **`SafetyGuard`**: filtro final. Si ds0, ds1, ds14 o ds15 ven algo a < 6 cm (`kSafetyStopM`) y el comando avanza, le resta la componente
  de avance (el robot aún puede girar o retroceder). Cuenta *episodios* (no pasos) en `interventions()`.
- **`MotorController`**: pone los motores en modo velocidad (posición infinita), satura cada rueda a ±10 rad/s y expone `stop()`.

## 10. Calibración y capturas (`calibration/SensorCalibration`)

Rutina del paso 1 (modo `calibrate`). Ver [CALIBRACION.md](CALIBRACION.md). `capture/SnapshotRecorder.h` guarda pares de imágenes (frontal y esférica) con la pose en `results/captures/` (modo `capture`, o `snap` al navegar).

## 11. Utilidades (`common/`)

- `Angles.h`: `wrap`, `toDeg`, `toRad`, `yawTowards(x, z, tx, tz)`.
- `RunOutput.h`: `directory()` (crea `../../results`, relativo al directorio del controlador que usa Webots), `path(nombre)`, `timestamp()` y
  `resolveTag()` con prioridad `tag=` en `controllerArgs` > mundo detectado (variable `WEBOTS_WORLD` o la línea de órdenes del
  proceso padre de Webots) > `s??`.

## Dónde tocar según lo que quieras hacer

| Quiero… | Archivo |
|---|---|
| Cambiar velocidades, umbrales o distancias | `config/RobotConfig.h` |
| Cambiar cuándo se rodea un obstáculo o cómo se rodea | `navigation/GoalNavigator.cpp` (`goToGoal`, `followWall`) |
| Cambiar los costes del camino (más margen a las paredes, etc.) | `config/RobotConfig.h` sección `mapping` y `planning/PathPlanner.h` (`buildCostmap`) |
| Adaptar la cámara a una luz nueva | `config/RobotConfig.h` sección `camera` y `perception/FloorSegmenter.cpp` |
| Añadir una métrica al CSV | `mission/RescueMission.h` (`finish`, `header` y `row` a la vez) o `mission/OutboundRun.cpp` |
| Cambiar el orden o las reglas de la misión (búsqueda, tiempo, parada) | `mission/RescueMission.h` y sección `victims`/`mission` de `RobotConfig.h` |
| Ajustar la detección de víctimas | `perception/VictimDetector.h`, `perception/FloorSegmenter.cpp` y sección `victims` |
| Cambiar cómo se pinta el mapa de la esférica | `mapping/SphericalMapper.h` y sección `mapping` (`kSphere*`) |
