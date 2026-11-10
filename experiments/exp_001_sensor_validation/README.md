# EXP-001 — Sensor validation

**Estado:** accepted. **Fecha:** 2026-11-10. **Hipótesis:** GPS, brújula, encoders, 16 sensores de distancia y ambas cámaras se inicializan y proporcionan lecturas/diagnósticos válidos sin mover el robot.

**Escenarios de EXP-001:** 01 (base), 07 (niebla `EXPONENTIAL2`), 08 (configuración visual distinta), 10 (máxima cantidad de cajas). Se generaron cuatro corridas, una por mundo de la matriz; el logger no capturó el identificador del escenario por corrida. La cobertura de los diez mundos corresponde a una puerta de integración posterior, no al alcance de este experimento.

## Procedimiento
1. Abrir una copia del WBT, compilar `fool_controller` y verificar los logs de inicio (`[STARTUP]`, `[CAMERAS]`, `[START]`, `[SENSORS]`).
2. Registrar pose, orientación, cámara frontal/esférica, rangos válidos y estados de cada sensor; comprobar motores detenidos.
3. Variar orientación/posición de arranque en la zona permitida cuando se haya validado el protocolo S1–S3.
4. Repetir y documentar diagnósticos por escenario sin alterar la calibración.

## Métricas y aceptación
- Nº de dispositivos correctamente inicializados / esperados; disponibilidad de imágenes; lecturas finitas y saturación.
- Error inicial del estimador de pose frente a la pose de prueba; presencia/ausencia de movimiento.
- **Aceptar** si todos los dispositivos necesarios tienen lectura o error explícito, las cámaras producen imágenes y el robot no se mueve; documentar cualquier sensor que falle.

**Salida:** `results/` con métricas y observaciones; no modificar los parámetros calibrados sin evidencia.

## Resultados

**Decisión: accept.** En las cuatro corridas registradas, GPS, brújula, encoders y el conjunto de 16 sensores aparecen válidos; ambas cámaras reportan imagen disponible. La pose muestreada fue `(x=0.000, z=-6.000)`, orientación `3.141593 rad`, encoders en `0.000 rad` y desplazamiento GPS `0.000 m`. El controlador ordenó mantener detenidos los motores.

| Corrida | Muestras CSV | Tiempo de muestra (s) | Resumen |
| --- | ---: | ---: | --- |
| `run_20261110_160303` | 1 | 0.032 | Completo; duración simulada informada: 0.064 s |
| `run_20261110_160418` | 1 | 0.032 | Completo; duración simulada informada: 0.032 s |
| `run_20261110_160437` | 1 | 0.032 | Completo; duración simulada informada: 0.032 s |
| `run_20261110_160458` | 1 | 0.032 | CSV disponible; `summary.txt` quedó incompleto al detenerse la ejecución |

Los datos sostienen la aceptación de la **validación inicial de dispositivos** para las cuatro corridas realizadas. Cada ejecución dejó una sola muestra, por lo que no demuestra estabilidad durante una prueba prolongada. Los registros automáticos no asocian cada carpeta con un número de escenario; esa asociación no se infiere de los mundos.

## Registro automático

Al iniciar el controlador se crea una carpeta única `results/run_YYYYMMDD_HHMMSS/` (con sufijo si coincide el nombre) que contiene:

- `sensor_samples.csv`: una fila por segundo simulado con validez de GPS, brújula, encoders y sensores de distancia; pose, orientación, encoders; disponibilidad de imagen de ambas cámaras; y respuesta cruda, distancia calibrada, detección y saturación de cada `ds0`–`ds15`.
- `summary.txt`: timestep, resoluciones de cámara, comando de parada, duración simulada, número de muestras y desplazamiento GPS observado.

Los campos no válidos quedan vacíos en el CSV; el resumen usa `not_measured` si no se obtuvo una pose válida. El controlador no identifica ni lee la geometría del escenario: anota `scenario=not_provided_to_controller`. Registra el escenario correspondiente al compartir los resultados. La carpeta se crea automáticamente dentro de `results/` al ejecutar cada mundo.
