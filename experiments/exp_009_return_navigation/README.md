# EXP-009 — Return to start

**Estado:** planned. **Hipótesis:** el mapa acumulado permite regresar hacia la pose/zona inicial y replanificar sin cartografiar toda la arena.

**Escenarios:** 01, 03, 07, 10; múltiples poses iniciales después de validar S1–S3.

## Procedimiento
1. Registrar pose de arranque y ruta transitada durante ida controlada.
2. Activar `RETURN` desde posiciones al otro extremo con mapa parcial existente.
3. Calcular A* por espacio confirmado como libre; ante bloqueo, replantear o detenerse con seguridad.
4. Comprobar cruce de línea de salida y registrar error respecto a pose de arranque.

## Métricas y aceptación
- Tasa de regreso, distancia al inicio, entrada a región Z < −8, tiempo y colisiones, nº replanteos.
- **Aceptar** si retorna a la zona de partida sin atravesar celdas ocupadas/desconocidas ni colisionar. La tolerancia para considerar `mismo punto` debe fijarse según incertidumbre de localización y enunciado, antes de la prueba final.

**Dependencias:** EXP-004/007. **Salida:** mapa del retorno y mediciones de desviación.
