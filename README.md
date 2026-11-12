# Rescate con Pioneer II (Webots)

Controlador `fool_controller` (C++, Webots R2020b) para el trabajo final de Robótica. Un robot Pioneer II
debe cruzar un recinto de 10 x 20 m con obstáculos, desde la salida (`z ≈ -9`) hasta la zona de las
víctimas (`z ≈ +8.5`), sin conocer el mapa de antemano.

El proyecto se construye por pasos; cada paso deja el robot funcionando antes de empezar el siguiente.

| Paso | Contenido | Estado |
|---|---|---|
| 1 | Sensores configurados y calibrados: GPS, brújula, encoders, 16 infrarrojos | **Hecho** (calibrado en el escenario 01) |
| 2 | Ir hasta el final evitando obstáculos: mapa + A\* con infrarrojos, Bug reactivo y guía con la cámara frontal | **Hecho: 10/10 escenarios superados** (con cámara) |
| 3 | Volver a la salida | Pendiente |
| 4 | Cámaras: detectar víctimas, aproximarse y girar 360° | Pendiente |
| 5 | Tabla final de resultados (ida + vuelta + víctimas) | Pendiente |

## Resultados del paso 2

Prueba de ida con cámara + mapa + A\*, un solo intento por mundo, límite de 240 s simulados
(datos completos y análisis en [`docs/RESULTADOS.md`](docs/RESULTADOS.md)).

| Mundo | Éxito | Tiempo (s) | Recorrido (m) | Obstáculos rodeados | Atascos | Frenos de emergencia |
|---|---|---|---|---|---|---|
| s01 | sí | 55,1 | 21,1 | 2 | 0 | 0 |
| s02 | sí | 99,7 | 30,3 | 5 | 0 | 0 |
| s03 | sí | 181,6 | 53,4 | 13 | 0 | 0 |
| s04 | sí | 58,6 | 23,1 | 1 | 0 | 0 |
| s05 | sí | 107,1 | 38,4 | 4 | 0 | 0 |
| s06 | sí | 130,8 | 34,7 | 12 | 0 | 0 |
| s07 | sí | 158,9 | 44,5 | 15 | 0 | 0 |
| s08 | sí | 120,8 | 37,3 | 6 | 0 | 0 |
| s09 | sí | 56,6 | 21,3 | 3 | 0 | 0 |
| s10 | sí | 68,1 | 22,4 | 4 | 0 | 0 |

Tiempo medio 103,7 s, recorrido medio 32,6 m; el más lento (s03, 181,6 s) queda a 58 s del límite.

## Inicio rápido

1. Abrir en Webots cualquier `worlds/final_scenarioNN.wbt` (R2020b).
2. Compilar `fool_controller` (en Webots: *Build → Build*; si vienes de una versión anterior, haz *Clean* antes).
3. Pulsar Play. Por defecto el robot **navega** hasta la línea amarilla del fondo y se detiene.
4. En consola aparecen líneas `[NAV]` cada segundo y, al terminar, una línea `[RESULT]`.
   Se añade una fila a `results/outbound_results.csv` y se guarda el mapa en `results/map_<mundo>_<cam|nocam>.txt`.

El comportamiento se cambia sin recompilar con el campo `controllerArgs` del robot (`Pioneer2` en el árbol de escena),
un argumento por línea:

| Argumento | Efecto |
|---|---|
| `navigate` | Prueba de ida (modo por defecto) |
| `calibrate` | Rutina de calibración del paso 1 (≈40 s simulados) |
| `monitor` | Robot parado; imprime pose y distancias cada segundo (para arrastrar el robot a mano y mirar sensores) |
| `nocam` / `cam` | Desactiva / activa la cámara frontal como ayuda |
| `nomap` / `map` | Desactiva / activa el mapa de ocupación + A\* (sin mapa = Bug puro) |
| `tag=XX` | Etiqueta manual de la prueba en el CSV (por defecto se detecta `s01`…`s10` del nombre del mundo) |

Los valores por defecto están en `config/RobotConfig.h` (`kStartupMode`, `kUseCameraByDefault`, `kUseMapByDefault`).

## Cómo funciona (visión general)

En cada paso de simulación el controlador ejecuta esta cadena:

```
 Sensores ──► SensorManager ──► SensorSnapshot ──► Localizer ──► Pose (x, z, yaw)
 (GPS, brújula,                  (lecturas ya        (GPS + brújula)        │
  encoders, 16 IR)                convertidas)                              ▼
                                                              OutboundRun (misión de ida)
 Cámara frontal ──► FloorSegmenter ──► espacio libre ──────────► │
                                                                 ├─ OccupancyGrid  (mapa con IR + pose)
                                                                 ├─ PathPlanner    (A* sobre el mapa)
                                                                 └─ GoalNavigator  (máquina de estados)
                                                                        │
                                                                        ▼
                                                          SafetyGuard (freno de emergencia)
                                                                        │
                                                                        ▼
                                                     MotorController ──► ruedas
```

1. **Percibir**: se leen GPS, brújula, encoders y 16 infrarrojos; los IR se filtran y se convierten a metros.
2. **Localizar**: la pose sale del GPS (posición) y la brújula (orientación).
3. **Recordar**: los IR y la pose alimentan un mapa de ocupación; el robot también cuenta por dónde ya pasó.
4. **Planificar**: A\* calcula el camino más corto a la línea meta, suponiendo libre lo desconocido y replanificando
   cada vez que el mapa cambia.
5. **Decidir**: una máquina de estados sigue el plan, usa la cámara para elegir huecos y velocidad,
   rodea obstáculos con los IR (Bug) y se recupera de atascos.
6. **Proteger y actuar**: el freno de emergencia anula el avance si algo está casi encima; después se envían las
   velocidades a los motores.

El detalle de cada módulo está en [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md).

## Estructura del repositorio

```
pioneer-ii-rescue-webots/
├── README.md                     este archivo
├── docs/
│   ├── ARQUITECTURA.md           qué hace cada módulo, sus entradas/salidas y cómo se conectan
│   ├── NAVEGACION.md             máquina de estados, mapa, A* y cámara explicados paso a paso
│   ├── CONFIGURACION.md          todos los parámetros de RobotConfig.h y cuándo tocarlos
│   ├── CALIBRACION.md            paso 1: rutina de calibración y valores obtenidos
│   └── RESULTADOS.md             resultados de los 10 mundos, cómo leer el CSV y el mapa
├── controllers/fool_controller/  código del controlador (ver tabla siguiente)
├── protos/Pioneer2_Robot.proto   modelo del robot
├── worlds/final_scenario01..10.wbt   los 10 escenarios de evaluación
└── results/                      salidas generadas por el controlador (CSV, mapas, informe de calibración)
```

### Código del controlador (`controllers/fool_controller/`)

| Archivo / carpeta | Qué hace |
|---|---|
| `fool_controller.cpp` | Punto de entrada. Lee `controllerArgs`, crea los módulos y ejecuta el bucle principal según el modo (`navigate`, `calibrate`, `monitor`). |
| `config/RobotConfig.h` | **Todos** los parámetros: nombres de dispositivos, cinemática, brújula, IR, umbrales de navegación, mapa y cámara. |
| `common/Angles.h` | Utilidades de ángulos (normalizar a (−π, π], grados↔radianes, yaw hacia un punto). |
| `common/RunOutput.h` | Carpeta `results/`, marca de tiempo y detección automática del mundo (`s01`…`s10`). |
| `sensors/SensorManager` | Activa y lee GPS, brújula, encoders, 16 IR (con mediana móvil) y la cámara frontal; entrega un `SensorSnapshot`. |
| `sensors/SensorSnapshot.h` | Estructura con las lecturas de un paso y los **sectores** de distancia (frente, diagonales, laterales, traseros) en metros. |
| `sensors/DistanceSensorCalibration` | Convierte la respuesta del IR en metros invirtiendo la tabla de Webots y aplicando el factor de reflexión del cian. |
| `localization/Localizer` | Pose `(x, z, yaw)`: GPS + brújula; odometría de respaldo y distancia recorrida. |
| `perception/FloorSegmenter` | Cámara frontal → clasifica suelo / obstáculo cian / víctima verde y devuelve el espacio libre por dirección. Se adapta a la luz de cada mundo. |
| `mapping/OccupancyGrid.h` | Mapa de ocupación de 100 x 200 celdas de 10 cm construido con IR + pose; recuerda lo recorrido. Guarda dibujo ASCII. |
| `planning/PathPlanner.h` | A\* optimista sobre el mapa hasta la línea meta, con costes por cercanía a paredes y zonas ya recorridas. |
| `navigation/GoalNavigator` | Cerebro de la ida: máquina de estados (alinear, ir a la meta, rodear, retroceder, media vuelta, recuperar). |
| `mission/OutboundRun` | Define la prueba de ida: meta, tiempo límite, métricas, resumen `[RESULT]`, CSV y mapa final. |
| `control/MotorController` | Aplica velocidades a las ruedas con saturación a 10 rad/s. |
| `control/SafetyGuard.h` | Último filtro: si un IR delantero ve algo a < 6 cm, anula el avance. |
| `control/WheelCommand.h` | Estructura `(izquierda, derecha)` en rad/s que produce cualquier comportamiento. |
| `calibration/SensorCalibration` | Rutina automática del paso 1 (modo `calibrate`). |

> Los módulos de mapa y planificación son solo cabeceras (`.h`), así que no hay que modificar el `Makefile` al añadirlos.

## Convenios (importante para colaborar)

- **Posición:** coordenadas del mundo de Webots `(x, z)` en metros. La salida está en `z ≈ −9`, las víctimas en `z ≈ +8,5`.
  El eje `y` (altura) no se usa.
- **Orientación (yaw):** el mismo ángulo que el campo `rotation 0 1 0 a` de Webots, calculado como
  `atan2(c[2], c[0])` con los valores de la brújula.
  - `yaw = π` → mira hacia +Z (hacia las víctimas); `yaw = 0` → mira hacia −Z (hacia la salida).
  - Girar a la izquierda **aumenta** el yaw.
  - Dirección de avance: `(dx, dz) = (−sin yaw, −cos yaw)`.
- **Velocidades:** rad/s de rueda. Multiplicar por el radio (0,0825 m) para obtener m/s (máx. 10 rad/s ≈ 0,83 m/s).
- **Infrarrojos:** alcance nominal 0,4 m, pero Webots atenúa la respuesta según el color. Con obstáculos cian
  (factor 0,52) el alcance real es de **≈ 0,21 m**, por eso el robot va despacio cerca de los obstáculos.
- **GPS:** los mundos usan `resolution -1` (sin cuantizar). Si la evaluación usara el GPS original (3 m), poner
  `kUseGpsForPosition = false` en `RobotConfig.h` y la posición saldrá solo de la odometría.
- **Sensores IR:** `ds0..ds7` lado izquierdo de delante a atrás, `ds15..ds8` lado derecho. `ds0/ds15` ≈ ±10°,
  `ds1/ds14` ≈ ±30°, `ds2/ds13` ≈ ±50°, `ds3/ds12` y `ds4/ds11` laterales, `ds5..ds7` / `ds8..ds10` traseros.

## Salidas en `results/`

El directorio se crea solo (junto a `worlds/`). Contenido:

| Archivo | Quién lo genera | Contenido |
|---|---|---|
| `outbound_results.csv` | prueba de ida | Una fila por ejecución (ver columnas en [`docs/RESULTADOS.md`](docs/RESULTADOS.md)). Si cambia el formato de columnas, el anterior se aparta como `outbound_results_old.csv`. |
| `map_<mundo>_<cam\|nocam>.txt` | prueba de ida con mapa | Dibujo ASCII del mapa final: `#` ocupado, `.` libre visto, `o` recorrido, `*` último plan. Arriba = +z. |
| `calibration_report.txt`, `calibration_ir_samples.csv` | modo `calibrate` | Informe con OK/REVISAR/FALLO y valores recomendados; muestras de los IR. |

## Limitaciones conocidas

- Las pruebas registradas son **solo con cámara y mapa** y con un intento por mundo. Falta repetir con `nocam` y `nomap`
  para la comparación de la tabla final.
- El controlador **no detecta colisiones reales**, solo lo que miden los IR. La columna `near_contacts` es un indicador
  de proximidad, no un choque confirmado: las colisiones hay que revisarlas viendo la simulación.
- Los pasos 3 (vuelta) y 4 (víctimas) no están implementados. La cámara esférica `camera_s` y el verde de las víctimas
  ya están previstos (`FloorSegmenter` cuenta píxeles verdes) pero sin usar.
- El mapa asume el recinto de 10 x 20 m del enunciado (`kArenaHalfXM/ZM` en `RobotConfig.h`).
