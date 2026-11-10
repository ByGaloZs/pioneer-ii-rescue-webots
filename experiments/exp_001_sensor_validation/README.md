# EXP-001 — Sensor validation

**Estado:** planned. No contiene resultados todavía.

## Hipótesis
El controlador actual inicializa de manera consistente GPS, brújula, encoders, los 16 sensores de distancia, cámara frontal y cámara esférica sin mover el Pioneer II.

## Alcance
- Código: `controllers/fool_controller/`.
- Mundos: `worlds/final_scenario01.wbt` a `worlds/final_scenario10.wbt`.
- Inicializaciones: anotar pose de inicio y repetir con orientaciones diferentes permitidas por el enunciado.
- Registrar commit, versión de Webots, tiempo básico del controlador y configuración relevante antes de ejecutar.

## Procedimiento previsto
1. Abrir el mundo en Webots y compilar `fool_controller`.
2. Verificar mensajes `[STARTUP]`, `[CAMERAS]`, `[START]` y `[SENSORS]`.
3. Comprobar la disponibilidad y dimensiones de ambas cámaras, GPS/brújula, encoders y lecturas válidas de proximidad.
4. Confirmar que las ruedas permanecen detenidas durante el diagnóstico.
5. Registrar errores, lecturas fuera de rango, saturación y fallos de inicialización en cada mundo.
6. Guardar resultados en `results/` y actualizar conclusiones.

## Criterios de aceptación propuestos
- Ningún dispositivo obligatorio sin inicialización o diagnóstico explícito.
- Orientación y pose finitas y coherentes con la pose configurada.
- Los sensores de distancia usan rangos calibrados y reportan saturaciones.
- Las cámaras tienen imágenes y dimensiones válidas.
- El robot permanece detenido.
- Resultados documentados para los diez escenarios.

**Pendiente:** ejecución y mediciones. Ninguna condición se declara cumplida todavía.
