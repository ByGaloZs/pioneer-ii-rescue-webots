# EXP-008 — Victim detection and rescue

**Estado:** planned. **Hipótesis:** la cámara puede detectar dos cilindros verdes, distinguir víctimas cercanas y lejanas y realizar la maniobra de rescate sin falsos positivos.

**Escenarios:** 01 (base), 07 (niebla), 08 (visibilidad distinta), 09 (víctimas en extremos laterales opuestos), 10 (víctimas separadas ≈ 0.83 m).

## Procedimiento
1. Obtener imágenes en ángulos/distancias controlados y segmentar regiones verdes.
2. Verificar aproximación libre de colisiones y confirmación de distancia < 1 m.
3. Ejecutar detención, giro completo de 360° y un único mensaje por víctima, evitando rescates duplicados.
4. Confirmar la identificación de ambas víctimas en mapas 09 y 10; contrastar posiciones reales solo offline.

## Métricas y aceptación
- Precisión/recall de detección, falsos rescates, distancia al detenerse, giro medido, tiempo de maniobra, víctimas únicas rescatadas.
- **Aceptar** si rescata exactamente dos víctimas válidas, sin duplicados, cumpliendo < 1 m y 360°; cualquier detección incierta debe provocar verificación adicional.

**Dependencias:** EXP-002/003 y control de ruedas. **Salida:** capturas verificables y log de cada rescate.
