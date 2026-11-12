# Paso 1: calibración de sensores

Rutina automática (`calibration/SensorCalibration`) que mide con los propios sensores los parámetros de `RobotConfig.h` que dependen del robot y del mundo.
Se ejecutó en `worlds/final_scenario01.wbt` y sus resultados están ya en `RobotConfig.h`. Solo hace falta repetirla si cambia el robot (PROTO),
el sensor de distancia, o el tipo de GPS/brújula de los mundos.

## Cómo ejecutarla

1. Abrir `worlds/final_scenario01.wbt` y compilar `fool_controller`.
2. Comprobar que el robot está en la pose original del archivo, `(0, −9,05)`, mirando a +Z, con espacio libre delante (~1 m) y una pared detrás.
3. Poner `calibrate` en el campo `controllerArgs` de `Pioneer2` (o `kStartupMode = StartupMode::kCalibrate`) y pulsar Play (≈ 40 s simulados).
4. Al terminar, la consola muestra el **INFORME DE CALIBRACIÓN** y se guarda en `results/calibration_report.txt`. Las muestras de los IR quedan en
   `results/calibration_ir_samples.csv`.
5. Copiar los **valores recomendados** a `config/RobotConfig.h`, recompilar y volver a ejecutar: todo debería salir `OK`.

Cada comprobación sale como `OK`, `REVISAR` (desviación moderada) o `FALLO`.

## Fases

| # | Fase | Qué hace | Qué mide |
|---|---|---|---|
| 1 | Reposo (2 s) | Robot quieto | Ruido de GPS y brújula; qué sensores detectan algo sin moverse |
| 2 | Recta (~1 m) | Avanza manteniendo el rumbo | GPS vs encoders → **radio de rueda**; dirección de avance del GPS vs brújula → **convenio/desfase de la brújula**; si el GPS está cuantizado |
| 3 | Giro 360° izquierda y derecha | Gira sobre sí mismo | Brújula vs encoders → **separación efectiva de ruedas** y signo del giro |
| 4 | Media vuelta y aproximación lenta | Se acerca despacio a la pared de detrás | Distancia IR frente a distancia real recorrida (GPS) → **factor de reflexión de los IR** |
| 5 | Retroceso e informe | Se aleja de la pared | Genera el informe y los valores recomendados |

## Valores obtenidos (escenario 01)

| Parámetro | Valor | Resultado |
|---|---|---|
| `kWheelRadiusM` | 0,0825 | GPS 1,0375 m = encoders 1,0375 m. El nominal es correcto |
| `kWheelSeparationM` | 0,3606 | Medida efectiva en giros de ±360°; la geométrica es 0,3206 m |
| `kCompassYawOffsetRad` | 0,0 | Desfase 0,00° entre brújula y dirección de avance |
| `kObstacleReflectionFactor` | 0,52 | Medido 0,527; se mantiene el teórico |
| GPS | sin cuantizar | Los mundos usan `resolution -1` |

## Modo monitor (comprobación manual)

Con `monitor` en `controllerArgs`, el robot queda parado e imprime cada segundo:

```
[MONITOR] t=12.0 pose=(0.000, -9.050) yaw=180.0deg odom=(0.000, -9.050) gps=ok brujula=ok encoders=ok | IR m: F=-- FL=-- FR=-- L=0.12 R=-- BL=-- BR=--
```

Sirve para **arrastrar el robot en Webots** y ver cómo responden pose y sectores IR (`--` = nada detectado). Útil para depurar un mundo nuevo o un cambio en el PROTO.

## Qué comprobar si algo falla en la navegación

| Síntoma | Posible causa | Qué mirar |
|---|---|---|
| El robot gira hacia el lado contrario | Convenio de brújula invertido | `kCompassYawOffsetRad` |
| Recorre más/menos de lo esperado, mapa deformado | Radio o separación de ruedas | `kWheelRadiusM`, `kWheelSeparationM` |
| Los IR "ven" las paredes más lejos/cerca de lo real | Factor de reflexión | `kObstacleReflectionFactor`; recordar que solo vale para obstáculos cian |
| Posición a saltos de 3 m | GPS cuantizado | `kUseGpsForPosition = false` |
