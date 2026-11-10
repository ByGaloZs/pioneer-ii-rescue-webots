# Arquitectura del proyecto

> **Estado:** arquitectura objetivo. Solo están disponibles la inicialización de dispositivos, captura de sensores, calibración de distancias y control básico de motores. Los nuevos algoritmos se implementarán progresivamente.

## Objetivo

El Pioneer II navegará por un escenario desconocido de 10 × 20 m para localizar dos cilindros verdes, detenerse a menos de 1 m de cada víctima, realizar un giro de 360° y regresar a la zona inicial en un máximo de 240 segundos de simulación. La posición y orientación iniciales pueden cambiar. Los archivos WBT son escenarios de prueba, **no mapas precargados para el controlador**.

## Flujo de información

```text
Webots devices
  |
SensorManager -> SensorSnapshot ----------> Localization (GPS/compass/encoders)
  |                                                    |
Perception (camera + proximity)                         |
  |                                                    |
  +---------------------------> Mapping <--------------+
                                     |
                          Occupancy grid (local/global)
                           /                       \
                Exploration (frontier)         Victim detection
                           |                       |
                  Planning (A*) <----------- Mission/time manager
                           |
                 Control / safety override
                           |
                        Wheels
                           |
                     New measurements
```

Las conexiones son conceptuales; no son interfaces ya implementadas.

## Contratos por módulo

| Ruta | Entrada | Salida / función | Estado |
| --- | --- | --- | --- |
| `config/` | Parámetros | Geometría y calibraciones | Existe |
| `sensors/` | Dispositivos Webots | `SensorSnapshot`, cámaras | Existe |
| `control/` | Comandos de rueda | Velocidades y parada | Existe |
| `localization/` | GPS, brújula, encoders | Pose y calidad | Planificado |
| `perception/` | Imagen esférica y 16 sensores | Observaciones con confianza | Planificado |
| `mapping/` | Pose + observaciones | Rejilla local/acumulada | Planificado |
| `exploration/` | Mapa + dirección objetivo | Frontera segura alcanzable | Planificado |
| `planning/` | Mapa libre + objetivo | Ruta A* | Planificado |
| `mission/` | Estado, tiempo, víctimas | Objetivo y modo | Planificado |

El punto de entrada `fool_controller.cpp` coordina los módulos, sin alojar la lógica de cada algoritmo. Los nuevos .cpp se incorporarán explícitamente al Makefile.

## Percepción, mapeo y exploración

1. Transformar las lecturas de la cámara 360° en observaciones radiales solo después de calibrar distancia y error. La cámara no mide distancias directamente.
2. Combinar dichas observaciones con los sensores de distancia según cobertura, rango, saturación e incertidumbre. Ausencia de retorno ≠ espacio libre.
3. Construir un mapa local y acumularlo en coordenadas del mundo mediante la pose estimada.
4. Mantener celdas **libres, ocupadas y desconocidas**, e inflar obstáculos según dimensiones del robot y margen.
5. Seleccionar fronteras accesibles que favorezcan progreso hacia la zona de víctimas, coste bajo y exploración novedosa.
6. Calcular con **A*** rutas hacia fronteras a través de **celdas libres confirmadas**.
7. Priorizar el control de seguridad, registrar bloqueos y replanificar cuando cambie el mapa.
8. Utilizar el mapa acumulado para planificar el regreso, sin requerir cartografía completa.

## Estados previstos

`INITIALIZE` → `INITIAL_SCAN` → `NAVIGATE` / `EXPLORE` → `APPROACH_VICTIM` → `RESCUE` → `RETURN` → `FINISH`.

`AVOID_OBSTACLE` puede interrumpir navegación o regreso. No se fijarán tiempos rígidos de retorno sin mediciones; la prioridad temporal se definirá con experimentos.

## Hitos

1. Validar inicialización y sensores.
2. Validar pseudo-LiDAR visual, rango y fusión.
3. Probar mapa de ocupación incremental.
4. Implementar fronteras, A* y replanificación.
5. Navegar a la zona opuesta evitando bloqueos.
6. Detectar/rescatar víctimas y regresar.
7. Evaluar en los diez escenarios y distintas poses iniciales.
