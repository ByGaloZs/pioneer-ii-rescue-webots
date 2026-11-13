# Rescate con Pioneer II (Webots R2020b, C++)

Controlador `fool_controller` del trabajo final de Robótica (UC3M): un Pioneer II debe cruzar una arena de 10 × 20 m con obstáculos,
localizar dos víctimas (cilindros verdes), parar a menos de 1 m de cada una, dar un giro completo de 360° con mensaje en la terminal y
volver a la salida en menos de 240 s. Se construye por pasos; cada paso deja el robot funcionando.

## Estado

| Paso | Contenido | Estado |
|---|---|---|
| 1 | Sensores configurados y calibrados (GPS, brújula, encoders, 16 IR) | **Hecho** (escenario 01) |
| 2 | Ida: mapa + A* (IR + cámara esférica), Bug reactivo, guía con cámara frontal | **Validado en Webots**: 10/10 mundos, 33–100 s (un intento por mundo) |
| 3 | Vuelta a la salida (A* que prefiere lo ya recorrido) | **En pruebas**: completada en Webots en 2 escenarios (s01, s10) |
| 4 | Víctimas: detección, parada a < 1 m, giro 360° y mensaje | **En pruebas**: 2/2 víctimas en esos 2 escenarios; 10/10 solo en el simulador propio |
| 5 | Los 10 escenarios completos en Webots, tabla final para el informe | Pendiente |

Los números y su procedencia (Webots frente a simulador propio) están en [docs/RESULTADOS.md](docs/RESULTADOS.md).
Pendiente conocido: la resolución del GPS en la evaluación (ver [docs/CONFIGURACION.md](docs/CONFIGURACION.md)).

## Inicio rápido

**Requisitos:** Webots **R2020b** (lo fija `protos/Pioneer2_Robot.proto`), compilador C++ y la variable `WEBOTS_HOME` (el `Makefile` incluye
`$(WEBOTS_HOME)/resources/Makefile.include`). Opcional, solo para el visor en directo: Python 3 y `matplotlib`.

1. Abrir `worlds/final_scenarioNN.wbt` (NN = 01…10) en Webots.
2. Compilar `fool_controller` (*Build* en el editor de Webots, o `make` dentro de `controllers/fool_controller/` con `WEBOTS_HOME` definida).
   Tras cambiar cabeceras conviene hacer *Clean* y recompilar. Los archivos nuevos del controlador deben estar en `CXX_SOURCES` del `Makefile`
   (hoy solo `.cpp`; las cabeceras no hacen falta).
3. En el árbol de escena, `Pioneer2` → `controllerArgs`: un argumento por línea. **Vacío = misión completa.** Ejemplos: `tag=s01`; `navigate`
   (solo la ida); `calibrate`; `capture`. Lista completa abajo.
4. Pulsar Play. Mensajes en consola: `[MISION]`, `[VICTIMA n]`, `[RESULT]`.
5. Resultados en `results/` (se crea sola junto a `worlds/`; no se versiona).
6. Visor en directo (opcional): `python3 tools/live_map.py --world worlds/final_scenarioNN.wbt`.

### `controllerArgs`

| Argumento | Efecto |
|---|---|
| *(vacío)* o `mission` | **Misión completa** (modo por defecto, `kStartupMode = kMission`) → `results/mission_results.csv` |
| `navigate` | Solo la ida (paso 2) → `results/outbound_results.csv` |
| `calibrate` / `monitor` / `capture` | Calibración (paso 1) / robot parado imprimiendo sensores / capturas de las cámaras (**`capture` borra las capturas anteriores** de `results/captures/` al empezar) |
| `nocam`, `nomap`, `nosphere` | Sin cámara frontal / sin mapa+A* (Bug puro) / sin cámara esférica (ni en el mapa ni en las observaciones de víctimas del modo `mission`; para comparar) |
| `nolive`, `snap` | Sin volcado del mapa en directo / capturas de cámaras mientras navega |
| `tag=XX` | Etiqueta de la prueba; si falta se detecta el mundo (`s01`…`s10`) o sale `s??` |

## Documentación

| Guía | Contenido |
|---|---|
| [docs/ARQUITECTURA.md](docs/ARQUITECTURA.md) | Qué hace cada módulo, qué recibe y entrega, dónde tocar |
| [docs/NAVEGACION.md](docs/NAVEGACION.md) | Máquina de estados, mapa + A*, uso de las cámaras, misión (ida, víctimas, vuelta), depuración |
| [docs/CONFIGURACION.md](docs/CONFIGURACION.md) | Todos los parámetros de `RobotConfig.h` |
| [docs/CALIBRACION.md](docs/CALIBRACION.md) | Rutina del paso 1 y valores obtenidos |
| [docs/RESULTADOS.md](docs/RESULTADOS.md) | Resultados, columnas de los CSV, mapas ASCII, qué falta |

## Estructura del repositorio

```
controllers/fool_controller/   controlador (C++); Makefile de Webots
  fool_controller.cpp            bucle principal y selección de modo
  config/RobotConfig.h           todos los parámetros
  sensors/ localization/         lectura de sensores y pose (GPS + brújula)
  perception/                    cámara frontal (FloorSegmenter), esférica (SphericalView), víctimas (VictimDetector.h)
  mapping/ planning/             mapa de ocupación (OccupancyGrid.h, SphericalMapper.h) y A* (PathPlanner.h)
  navigation/                    GoalNavigator: seguir el plan, rodear obstáculos, atascos
  mission/                       RescueMission.h (misión completa) y OutboundRun (prueba de la ida)
  control/ calibration/ capture/ motores y seguridad, rutina de calibración, capturas de cámaras
  common/                        ángulos y carpeta de resultados
docs/                          guías (tabla anterior)
protos/Pioneer2_Robot.proto    robot (no se modifica)
tools/live_map.py              visor en directo del mapa y de las víctimas
worlds/                        escenarios final_scenario01…10
results/                       salidas de las pruebas (CSV, mapas, capturas); no versionado
```

Casi todo el código nuevo son cabeceras (`.h`) que no necesitan tocar el `Makefile`; los `.cpp` están listados en `CXX_SOURCES`.

## Convenios

- **Posición:** `(x, z)` del mundo de Webots, en metros. Salida en `z ≈ −9`, víctimas en `z ≈ +8,3…9,5`.
- **Orientación (yaw):** el ángulo de `rotation 0 1 0 a` de Webots, `atan2(c[2], c[0])` con la brújula. `yaw = π` mira a +Z (víctimas), `yaw = 0` a −Z (salida);
  girar a la izquierda lo aumenta; avance = `(−sin yaw, −cos yaw)`.
- **Infrarrojos:** alcance nominal 0,4 m, pero Webots atenúa según el color; con obstáculos cian el alcance real es ≈ 0,21 m.
- **GPS:** los mundos usan `resolution -1` (sin cuantizar). Si la evaluación usara el GPS original (cuantizado a 3 m) habría que poner `kUseGpsForPosition = false`; **sin tratar todavía**.

## Misión completa en una pantalla

`mission/RescueMission.h`: **IDA** (z ≥ 8,2) → **BUSCAR** (barrido del pasillo del fondo si aún no hay víctima confirmada) → **IR_A_VICTIMA** (A* hasta 1,15 m) →
**ACERCAR** (hasta 0,85 m del centro) → **GIRO_360** (+ mensaje `[VICTIMA n] ...`) → de nuevo BUSCAR/IR_A hasta tener las dos → **VUELTA** (A* hasta z ≤ −8,2).
Si el tiempo transcurrido, la reserva de vuelta (0,55 × tiempo de ida + 12 s) y un margen de 12 s no caben en 240 s, se deja de buscar y se vuelve con las
víctimas rescatadas. Detalle en [docs/NAVEGACION.md](docs/NAVEGACION.md).

Supuestos: la vuelta termina al cruzar z ≤ −8,2 (línea amarilla de salida en z = −8); «a menos de 1 m» se interpreta como distancia centro–centro.
Las colisiones reales solo se ven en Webots: el controlador no las mide, solo episodios con algún IR a menos de 3 cm o saturado (`near_contacts`).
