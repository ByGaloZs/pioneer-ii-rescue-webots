# EXP-001 — Sensor validation

**Estado:** planned. **Hipótesis:** GPS, brújula, encoders, 16 sensores de distancia y ambas cámaras se inicializan y proporcionan lecturas/diagnósticos válidos sin mover el robot.

**Escenarios:** 01 (base), 07 (niebla `EXPONENTIAL2`), 08 (configuración visual distinta), 10 (máxima cantidad de cajas). **Puerta de integración:** al menos un arranque en cada uno de los diez mundos.

## Procedimiento
1. Abrir una copia del WBT, compilar `fool_controller` y verificar los logs de inicio (`[STARTUP]`, `[CAMERAS]`, `[START]`, `[SENSORS]`).
2. Registrar pose, orientación, cámara frontal/esférica, rangos válidos y estados de cada sensor; comprobar motores detenidos.
3. Variar orientación/posición de arranque en la zona permitida cuando se haya validado el protocolo S1–S3.
4. Repetir y documentar diagnósticos por escenario sin alterar la calibración.

## Métricas y aceptación
- Nº de dispositivos correctamente inicializados / esperados; disponibilidad de imágenes; lecturas finitas y saturación.
- Error inicial del estimador de pose frente a la pose de prueba; presencia/ausencia de movimiento.
- **Aceptar** si todos los dispositivos necesarios tienen lectura o error explícito, las cámaras producen imágenes y el robot no se mueve; documentar cualquier sensor que falle. No hay resultados todavía.

**Salida:** `results/` con métricas y observaciones; no modificar los parámetros calibrados sin evidencia.
