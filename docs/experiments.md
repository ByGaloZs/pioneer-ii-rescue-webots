# Protocolo de experimentos

## Objetivo
Aislar hipótesis, conservar resultados reproducibles y evitar que pruebas intermedias contaminen el controlador principal. Cada experimento debe poder explicarse en la memoria final.

## Estructura obligatoria
```text
experiments/
├── _template/README.md
└── exp_NNN_short_name/
    ├── README.md
    ├── src/               # Solo si hay código aislado
    └── results/           # CSV, gráficos pequeños, observaciones
```

Asignar un número incremental por hipótesis. No reutilizar un ID para un experimento distinto. Documentar también los intentos fallidos.

## Procedimiento
1. Escribir hipótesis, unidad de medida y criterio de aceptación **antes de ejecutar**.
2. Registrar versión de Webots, commit, parámetros, mundo, pose inicial `(x,z,yaw)`, método/semilla de inicialización y número de repeticiones.
3. Comenzar en uno o varios mundos controlados. Para afirmar robustez, evaluar los diez `final_scenario01.wbt` a `final_scenario10.wbt` y múltiples poses permitidas.
4. Ejecutar y registrar **tiempo de simulación**, no tiempo real. Limitar la misión a 240 s.
5. Separar valores observados de estimaciones y marcar `not_measured` cuando falten datos; nunca inventar ceros.
6. Analizar fallos, decidir `accept`, `iterate` o `reject`, y actualizar `docs/decisions.md` cuando cambie la arquitectura.
7. Integrar al controlador solo con evidencia y comprobaciones de regresión de sensores y seguridad.

## Métricas por fase

| Fase | Métricas |
| --- | --- |
| Sensores | inicialización válida, rangos, saturaciones, cámara disponible, pose coherente |
| Percepción | error radial, falsos espacios libres/ocupados, cobertura y detección de pasos |
| Mapeo | consistencia, ocupación desconocida, celdas seguras y falsas detecciones |
| Navegación | tiempo simulado, distancia/progreso, colisiones, bloqueos, repeticiones |
| Rescate/retorno | víctimas confirmadas, giro, distancia de rescate, llegada a inicio, éxito < 240 s |

Encabezado sugerido para `results/metrics.csv` (solo cuando existan resultados medidos):

```csv
experiment_id,run_id,scenario,start_x,start_z,start_yaw,elapsed_s,progress_m,collisions,victims_rescued,returned_to_start,mission_complete,notes
```

Guardar CSV y conclusiones pequeñas en Git; excluir compilados, vídeos y logs voluminosos con `.gitignore`. El archivo CSV no es generado automáticamente por el controlador actual.
