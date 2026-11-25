# Rescate con Pioneer II (Webots R2020b, C++)

Controlador `fool_controller`: un Pioneer II debe cruzar una arena de 10 × 20 m con obstáculos,
localizar dos víctimas (cilindros verdes), parar a menos de 1 m de cada una, dar un giro completo de 360° con mensaje en la terminal y
volver a la salida en menos de 240 s.

## Estado

| Etapa | Contenido | Estado |
|---|---|---|
| Sensores | Sensores configurados y calibrados (GPS, brújula, encoders, 16 IR) | **Hecho** (escenario 01) |
| Ida | Ida: mapa + A* (IR + cámara esférica), Bug reactivo, guía con cámara frontal | **Validado en Webots**: 10/10 mundos, 33–100 s (un intento por mundo) |
| Vuelta | Vuelta a la salida (A* que prefiere lo ya recorrido) | **Validado en Webots**: retorno completado en 10/10 mundos |
| Víctimas | Víctimas: detección, parada a < 1 m, giro 360° y mensaje | **Validado en Webots**: 20/20 víctimas en los 10 mundos, rescates verificados visualmente |
| Misión | Misión integral en los 10 escenarios | **Probada en Webots**: 10/10 misiones completas (vuelta a tiempo y 2/2 víctimas), todas en ≤ 240 s |
| Variantes | Comparación sin cámara frontal / sin esférica / sin mapa | **Hecha**: 10/10, 6/10 y 2/10 misiones completas en ≤ 240 s |

**Resultados finales (24-11-2026, código final, un intento por mundo):** con el controlador de entrega, **10/10 misiones completas** y **20/20 víctimas**, todas por debajo de 240 s; tiempo total medio **129,9 s**, mínimo **96,1 s** (s04), máximo **199,3 s** (s03, con 40,7 s de margen). La comparación de variantes (con `timelimit=600`) mide lo que aporta cada capa: sin cámara frontal, 10/10 dentro de 240 s pero un 43 % más lento; sin cámara esférica, 6/10; sin mapa, 2/10. Los rescates se verificaron visualmente en Webots. Los CSV y mapas de las cuatro tandas están en `results/Tanda A`…`Tanda D` y las consolas en `logs/Tanda A`…`Tanda D` (ninguno se versiona); repetir una ejecución con el mismo código da las mismas cifras (simulación determinista). Detalle en [docs/RESULTADOS.md](docs/RESULTADOS.md).

**Cambios sobre los archivos de la práctica** (permitidos si se documentan): los 10 mundos usan el robot de `protos/Pioneer2_Robot.proto` (nodo `Pioneer2_Robot`; los originales cargaban el `Pioneer2` de Webots), con la `lookupTable` de los 8 IR delanteros reducida de 1 m a 0,4 m; GPS sin cuantizar (`resolution -1`) y `controllerArgs`; la salida del robot es la original en todos. Detalle y cómo deshacerlos en [docs/CAMBIOS.md](docs/CAMBIOS.md).

## Inicio rápido

**Requisitos:** Webots **R2020b** (lo fija `protos/Pioneer2_Robot.proto`), compilador C++ y la variable `WEBOTS_HOME` (el `Makefile` incluye
`$(WEBOTS_HOME)/resources/Makefile.include`). Opcional, solo para el visor en directo: Python 3 y `matplotlib`.

1. Abrir `worlds/final_scenarioNN.wbt` (NN = 01…10) en Webots.
2. Compilar `fool_controller` (*Build* en el editor de Webots, o `make` dentro de `controllers/fool_controller/` con `WEBOTS_HOME` definida).
   Tras cambiar cabeceras conviene hacer *Clean* y recompilar. Los archivos nuevos del controlador deben estar en `CXX_SOURCES` del `Makefile`
   (hoy solo `.cpp`; las cabeceras no hacen falta).
3. En el árbol de escena, `Pioneer2_Robot` → `controllerArgs`: un argumento por línea. **Vacío = misión completa.** Ejemplos: `nosphere`; `calibrate`;
   `nofiles`. Lista completa abajo.
4. Pulsar Play. Mensajes en consola: `[MISION]`, `[VICTIMA n]`, `[RESULT]`.
5. Resultados en `results/` (se crea sola junto a `worlds/`; no se versiona). La salida de consola de cada ejecución se guarda además en
   `logs/<mundo>_<fecha-hora>.log` (tampoco se versiona); la primera línea `[STARTUP] Consola guardada en …` dice el archivo.
6. Visor en directo (opcional): `python3 tools/live_map.py --world worlds/final_scenarioNN.wbt`. El mapa empieza negro y se va llenando según el robot descubre el recinto. Para ver un mapa final como imagen: `python3 tools/live_map.py --map results/map_NN_mission.txt` (añade `--save mapa.png` para guardarlo).

### `controllerArgs`

| Argumento | Efecto |
|---|---|
| *(vacío)* o `mission` | **Misión completa** (modo por defecto, `kStartupMode = kMission`) → `results/mission_results.csv` |
| `calibrate` | Rutina de calibración de sensores → `results/calibration_report.txt` |
| `nocam`, `nomap`, `nosphere` | Sin cámara frontal / sin mapa+A* (Bug puro) / sin cámara esférica (ni en el mapa ni en las observaciones de víctimas del modo `mission`; para comparar) |
| `nolive` | Sin volcado del mapa en directo |
| `nofiles` | No escribe nada en disco (ni CSV, ni mapas, ni log): solo la consola |
| `timelimit=SEG` | Límite total de la misión en segundos (por defecto 240, el del enunciado). Para las comparaciones que necesitan más tiempo |
| `tag=XX` | Etiqueta de la prueba; si falta se detecta el mundo (`s01`…`s10`) o sale `s??` |

## Documentación

| Guía | Contenido |
|---|---|
| [docs/FUNCIONAMIENTO.md](docs/FUNCIONAMIENTO.md) | **Empieza aquí:** cómo funciona el robot de principio a fin (sensores, mapa, A*, navegación, misión) |
| [docs/DIAGRAMAS.md](docs/DIAGRAMAS.md) | Diagramas de flujo: arranque y modos, dependencias entre módulos, fases de la misión |
| [docs/ARQUITECTURA.md](docs/ARQUITECTURA.md) | Qué hace cada módulo, qué recibe y entrega, dónde tocar |
| [docs/NAVEGACION.md](docs/NAVEGACION.md) | Máquina de estados, mapa + A*, uso de las cámaras, misión (ida, víctimas, vuelta), depuración |
| [docs/CONFIGURACION.md](docs/CONFIGURACION.md) | Los parámetros de `RobotConfig.h` |
| [docs/CALIBRACION.md](docs/CALIBRACION.md) | Rutina de calibración de sensores y valores obtenidos |
| [docs/RESULTADOS.md](docs/RESULTADOS.md) | Resultados, columnas de los CSV, mapas ASCII, qué falta |
| [docs/CAMBIOS.md](docs/CAMBIOS.md) | Cambios respecto a los mundos y el robot de la práctica (GPS…) y supuestos |

## Estructura del repositorio

```
controllers/fool_controller/   controlador (C++); Makefile de Webots
  fool_controller.cpp            bucle principal y selección de modo
  config/RobotConfig.h           parámetros principales
  sensors/ localization/         lectura de sensores y pose (GPS + brújula)
  perception/                    cámara frontal (FloorSegmenter), esférica (SphericalView), víctimas (VictimDetector.h)
  mapping/ planning/             mapa de ocupación (OccupancyGrid.h, SphericalMapper.h) y A* (PathPlanner.h)
  navigation/                    GoalNavigator: seguir el plan, rodear obstáculos, atascos
  mission/                       RescueMission.h (misión completa)
  control/ calibration/          motores y seguridad, rutina de calibración
  common/                        ángulos y carpeta de resultados
docs/                          guías (tabla anterior)
protos/Pioneer2_Robot.proto    robot que usan los mundos (IR delanteros a 0,4 m, ver docs/CAMBIOS.md)
tools/live_map.py              visor en directo del mapa y de las víctimas
worlds/                        escenarios final_scenario01…10
results/                       salidas de las pruebas (CSV, mapas); no versionado
logs/                          consola de cada ejecución; no versionado
memoria/                       memoria del trabajo (LaTeX, plantilla UC3M/IEEE); no versionada
```

Casi todo el código nuevo son cabeceras (`.h`) que no necesitan tocar el `Makefile`; los `.cpp` están listados en `CXX_SOURCES`.

## Convenios

- **Posición:** `(x, z)` del mundo de Webots, en metros. Salida en `z ≈ −9`, víctimas en `z ≈ +8,3…9,5`.
- **Orientación (yaw):** el ángulo de `rotation 0 1 0 a` de Webots, `atan2(c[2], c[0])` con la brújula. `yaw = π` mira a +Z (víctimas), `yaw = 0` a −Z (salida);
  girar a la izquierda lo aumenta; avance = `(−sin yaw, −cos yaw)`.
- **Infrarrojos:** los 16 tienen una tabla de 0,4 m (los 8 delanteros venían con 1 m en el PROTO; ver [docs/CAMBIOS.md](docs/CAMBIOS.md)). Webots atenúa según el color: con obstáculos cian el alcance real es ≈ 0,21 m. El mapa solo confía en los IR hasta 0,19 m (`kIrMaxRangeM`).
- **GPS:** los mundos usan `resolution -1` (sin cuantizar; el original tenía `resolution 3`). Es un cambio permitido y documentado en [docs/CAMBIOS.md](docs/CAMBIOS.md). Con el GPS original habría que poner `kUseGpsForPosition = false` (solo odometría, no probado con la misión completa).

## Misión completa en una pantalla

`mission/RescueMission.h`: **IDA** (z ≥ 8,2) → **BUSCAR** (barrido del pasillo del fondo si aún no hay víctima confirmada) → **IR_A_VICTIMA** (A* hasta 1,15 m) →
**ACERCAR** (hasta 0,85 m del centro) → **GIRO_360** (+ mensaje `[VICTIMA n] ...`) → de nuevo BUSCAR/IR_A hasta tener las dos → **VUELTA** (A* hasta z ≤ −8,2).
Si el tiempo transcurrido, la reserva de vuelta (0,55 × tiempo de ida + 12 s) y un margen de 12 s no caben en 240 s, se deja de buscar y se vuelve con las
víctimas rescatadas. Detalle en [docs/NAVEGACION.md](docs/NAVEGACION.md).

Supuestos: la vuelta termina al cruzar z ≤ −8,2 (línea amarilla de salida en z = −8); «a menos de 1 m» se interpreta como distancia centro–centro.
Las colisiones reales solo se ven en Webots: el controlador no las mide, solo episodios con algún IR a menos de 3 cm o saturado (`near_contacts`).
