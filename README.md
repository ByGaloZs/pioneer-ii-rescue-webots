# Rescate con Pioneer II 

Controlador `fool_controller` (C++, Webots) para el trabajo final de Robótica.
Se construye por pasos; cada paso deja el robot funcionando antes de pasar al siguiente.

| Paso | Contenido | Estado |
|---|---|---|
| 1 | Sensores configurados y calibrados: GPS, brújula, encoders, 16 infrarrojos | **Hecho** (calibrado en escenario 01, 2026-11-12) |
| 2 | Ir hasta el final evitando obstáculos: mapa + A* con infrarrojos, Bug reactivo y guía con la cámara frontal | **En pruebas** |
| 3 | Volver a la salida | Pendiente |
| 4 | Cámaras: detectar víctimas, aproximarse y girar 360° | Pendiente |
| 5 | Los 10 escenarios, tiempos y tabla de resultados | Pendiente |

## Estructura

```
controllers/fool_controller/
├── fool_controller.cpp           bucle principal y selección de modo
├── config/RobotConfig.h          TODOS los parámetros del robot
├── common/Angles.h               utilidades de ángulos
├── common/RunOutput.h            carpeta results/ y etiqueta automática del mundo
├── sensors/
│   ├── SensorManager             activa y lee los dispositivos -> SensorSnapshot
│   ├── SensorSnapshot.h          lecturas de un paso + sectores de distancia en metros
│   └── DistanceSensorCalibration respuesta IR -> metros (tabla de Webots + factor cian)
├── localization/Localizer        pose (x, z, yaw): GPS + brújula; odometría de respaldo
├── perception/FloorSegmenter     cámara frontal: suelo / cian / verde -> espacio libre por dirección
├── mapping/OccupancyGrid.h       mapa de ocupación 10x20 m (celdas 10 cm) con IR + pose; memoria de lo recorrido
├── planning/PathPlanner.h        A* optimista sobre el mapa hasta la línea meta
├── navigation/GoalNavigator      seguir el plan, rodear obstáculos (Bug), cuñas, atascos
├── mission/OutboundRun           prueba de ida: meta, tiempo límite y métricas -> results.csv
├── control/
│   ├── MotorController           velocidades de rueda con saturación
│   ├── SafetyGuard.h             freno de emergencia con los IR delanteros
│   └── WheelCommand.h            orden de velocidades que produce cada comportamiento
└── calibration/SensorCalibration rutina automática de calibración (paso 1)

results/                          (se crea sola, junto a worlds/) informe de calibración y outbound_results.csv
```

## Convenios

- **Posición:** coordenadas del mundo de Webots `(x, z)` en metros. La salida está en `z ≈ -9`, las víctimas en `z ≈ +8.5`.
- **Orientación (yaw):** el mismo ángulo que el campo `rotation 0 1 0 a` de Webots, calculado como `atan2(c[2], c[0])` con los valores de la brújula.
  - `yaw = π` → mira hacia +Z (hacia las víctimas); `yaw = 0` → mira hacia −Z (hacia la salida).
  - Girar a la izquierda aumenta el yaw.
  - Dirección de avance: `(dx, dz) = (−sin yaw, −cos yaw)`.
- **Infrarrojos:** alcance nominal de 0,4 m, pero Webots atenúa la respuesta según el color. Con obstáculos cian (factor 0,52) el alcance real es de **unos 0,21 m**, así que el robot debe ir despacio cerca de los obstáculos.
- **GPS:** el mundo usa `resolution -1` (sin cuantizar). Si la evaluación usara el GPS original (3 m), poner `kUseGpsForPosition = false` en `RobotConfig.h`.

## Paso 1: calibrar

1. Abrir `worlds/final_scenario01.wbt` y compilar `fool_controller`.
2. Comprobar que el robot está en su pose original del archivo, `(0, −9.05)`, mirando a +Z.
3. Ejecutar la simulación. El modo por defecto es `calibrate` (≈40 s simulados):
   reposo → recta de 1 m → giro de 360° a izquierda y derecha → media vuelta → aproximación lenta a la pared → retroceso.
4. Al terminar, la consola muestra el **INFORME DE CALIBRACIÓN**, que también se guarda en `results/calibration_report.txt`. Cada comprobación sale como OK, REVISAR o FALLO, y al final aparecen los **valores recomendados** para `RobotConfig.h`. Las muestras de los infrarrojos se guardan en `results/calibration_ir_samples.csv`.
5. Copiar esos valores en `config/RobotConfig.h`, recompilar y volver a ejecutar. Todo debería salir OK.

Modo `monitor`: el robot queda parado y cada segundo imprime la pose y la distancia de cada sector (F, FL, FR, L, R, BL, BR). Sirve para arrastrar el robot en Webots y comprobar los sensores a mano. Se activa con `kStartupMode = StartupMode::kMonitor` en `RobotConfig.h`, o escribiendo `monitor` en el campo `controllerArgs` del robot.

## Paso 2: navegación de ida (prueba con y sin cámara)

El modo por defecto es ahora `navigate`: el robot va desde donde esté hasta cruzar la línea amarilla del fondo (z ≥ 8.2) y se detiene.

Comportamientos (`navigation/GoalNavigator`):
- **ALIGN**: si la meta queda a más de 60°, gira en el sitio hacia ella.
- **GO_TO_GOAL**: avanza hacia +Z. **Con cámara**, busca en ±20° el hueco libre más cercano a la meta y adapta la velocidad al espacio libre (hasta ~0,66 m/s). **Sin cámara**, va a ~0,33 m/s.
- **FOLLOW_WALL_L/R**: al encontrar un obstáculo lo rodea con los infrarrojos. Lo suelta cuando ha avanzado hacia la meta más que en el punto del choque y la dirección de la meta está libre.
- **REVERSE_OUT / TURN_AROUND**: si el paso se estrecha en cuña o rodeando se aleja más de 1,5 m de la meta, sale marcha atrás y rodea por el otro lado.
- **RECOVER**: si se ordena avanzar o girar y el robot no se mueve, retrocede y gira.
- Freno de emergencia (`SafetyGuard`): IR delantero a menos de 6 cm → se anula el avance.

### Mapa de ocupación y A* (memoria)

Sin memoria, el Bug vuelve a meterse en los mismos callejones (escenarios 03, 05, 07, 09). Ahora:

- `mapping/OccupancyGrid.h` construye un mapa de 100 x 200 celdas de 10 cm con los 16 infrarrojos y la pose (no depende de la luz). Las paredes exteriores del recinto (10 x 20 m) se marcan desde el principio. También cuenta cuántas veces ha pasado el robot por cada celda.
- `planning/PathPlanner.h` calcula con A* el camino más corto hasta la línea meta. Lo desconocido se considera libre (optimista); cuando aparece una pared nueva en el mapa se replanifica. Se prohíben las celdas donde el robot no cabe (0,25 m de un obstáculo) y se encarece pasar pegado a paredes o por zonas ya recorridas.
- `GoalNavigator` usa la dirección del plan (un punto a 0,8 m sobre la ruta) en lugar de "recto hacia +z". Al chocar rodea por el lado que indica el plan, y si rodeando el plan pide volver atrás (callejón), deja la pared y sigue el plan.
- Al terminar se guarda el mapa en `results/map_<tag>_<cam|nocam>.txt` (dibujo ASCII, útil para el informe).
- Con `nomap` en `controllerArgs` se desactiva y se vuelve al Bug puro, para comparar.

Los archivos nuevos son solo cabeceras: **no hay que tocar el Makefile**. Conviene hacer *Clean* y recompilar.

### Cámara y luz

Los escenarios tienen iluminación muy distinta (02 y 03 casi a oscuras, 04–07 con poca luz y niebla, 09 con niebla). El segmentador ahora aprende en cada imagen el color del suelo (franja inferior) y clasifica cada píxel relativo a él. Si el suelo se ve demasiado oscuro, la cámara se desactiva y se navega solo con infrarrojos. Más allá de 3,5 m (niebla) no se da nada por libre. La columna `cam_valid_pct` dice en qué % de la prueba la cámara se pudo usar, y la consola muestra `luz=` (brillo del suelo, 0–255).

### Cómo hacer la prueba

En el árbol de escena: `Pioneer2` → campo `controllerArgs`. Añadir un argumento por línea:

| Prueba | controllerArgs |
|---|---|
| Con cámara | `navigate`, `tag=s01` |
| Sin cámara | `navigate`, `nocam`, `tag=s01` |
| Sin mapa (Bug puro) | `navigate`, `nomap` |

`tag` es opcional: si no se pone, el controlador detecta el mundo (`s01`…`s10`) leyendo la línea de órdenes de Webots; si no lo consigue escribe `s??`. Úsalo para distinguir salidas distintas (`s01-a`). Para probar otras salidas, arrastra el robot a otra posición detrás de la línea amarilla y gíralo antes de pulsar Play.

Al terminar aparece en consola una línea `[RESULT]` y se añade una fila a `results/outbound_results.csv` (en la raíz del proyecto, junto a `worlds/`; la primera columna es la fecha y hora):

| Columna | Significado |
|---|---|
| success / time_s | si cruzó la línea final y en cuánto tiempo simulado |
| max_z / lines | avance máximo y líneas cruzadas (de 8) |
| path_m | distancia recorrida |
| obstacles / side_switches | obstáculos rodeados y medias vueltas |
| stuck_recoveries | atascos detectados |
| emergency_stops / near_contacts / min_ir_m | frenos de emergencia, lecturas IR < 3 cm (posible roce) y distancia mínima vista |
| map / cam_valid_pct / replans | si se usó el mapa, % del tiempo con la cámara utilizable y número de replanificaciones |

Las colisiones reales hay que mirarlas en la simulación: el controlador solo ve lo que miden los infrarrojos.

Todos los umbrales y velocidades están en `config/RobotConfig.h`, sección `nav`.
