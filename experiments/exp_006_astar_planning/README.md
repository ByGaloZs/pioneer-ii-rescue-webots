# EXP-006 — A* and replanning

**Estado:** planned. **Hipótesis:** A* encuentra rutas seguras sobre espacio confirmado como libre y puede replantearlas cuando cambia el mapa.

**Escenarios:** mapas sintéticos unitarios + WBT 01, 03 y 10.

## Procedimiento
1. Probar rejillas sintéticas: pasillo abierto, muro bloqueante, callejón, frontera desconectada, ruta alternativa y margen del robot.
2. Comparar coste de A* con Dijkstra de referencia bajo la misma rejilla y costes; comprobar que no cruza `occupied` ni `unknown`.
3. Integrar con mapas producidos por percepción y provocar cambios de ocupación controlados.
4. Registrar rutas, longitud, motivo de replanificación y tiempos de cómputo.

## Métricas y aceptación
- Integridad de ruta, minimalidad frente a Dijkstra en casos verificables, número de replanteos y tiempo de cálculo.
- **Aceptar** si todos los casos sintéticos son correctos, no aparecen celdas inseguras en rutas y existe replanteo o fallo seguro cuando desaparece el camino.

**Dependencia:** EXP-004. **Salida:** pruebas unitarias y trazas, sin geometría precargada para ejecución autónoma.
