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

controllers/
└── experiment_test_controller/ # Único harness C++ runtime compartido

worlds/
└── experiment_test_scenario01.wbt # Única copia de prueba, regenerable desde scenario01
```

Asignar un número incremental por hipótesis. No reutilizar un ID para un experimento distinto. Documentar también los intentos fallidos.

## Procedimiento
1. Escribir hipótesis, unidad de medida y criterio de aceptación **antes de ejecutar**.
2. Registrar versión de Webots, commit, parámetros, mundo, pose inicial `(x,z,yaw)`, método/semilla de inicialización y número de repeticiones.
3. Comenzar siempre en la copia compartida de scenario 01. Después de validar la línea base, evaluar las condiciones representativas y, para afirmar robustez de navegación/misión, los diez `final_scenario01.wbt` a `final_scenario10.wbt` y múltiples poses permitidas.
4. Ejecutar y registrar **tiempo de simulación**, no tiempo real. Limitar la misión a 240 s.
5. Separar valores observados de estimaciones y marcar `not_measured` cuando falten datos; nunca inventar ceros.
6. Analizar fallos, decidir `accept`, `iterate` o `reject`, y actualizar `docs/decisions.md` cuando cambie la arquitectura.
7. Integrar al controlador solo con evidencia y comprobaciones de regresión de sensores y seguridad.

## Aislamiento de código y corridas Webots

- `controllers/fool_controller/` es el controlador estable/final. No se agregan allí loggers ni prototipos ligados a una sola hipótesis.
- Todos los experimentos Webots con código C++ comparten un solo `controllers/experiment_test_controller/`, un punto de entrada y un `Makefile`. Se configura para el experimento activo y reutiliza módulos estables mediante referencias; no se copia el controlador completo ni se reemplaza el binario final.
- Generar una sola copia del escenario 01 en `worlds/experiment_test_scenario01.wbt`. Regenerar ese mismo archivo con pose, sensor y etiqueta del ensayo activo; no crear un WBT por experimento o por pose. Configurar `controller "experiment_test_controller"` y etiqueta explícita `controllerArgs`; no modificar `final_scenario*.wbt`.
- Cada logger escribe solo dentro de `experiments/exp_NNN_short_name/results/`. Si no puede determinar/abrir esa raíz o falta una etiqueta requerida, debe fallar cerrado y no usar un `results/` genérico.
- La corrida piloto valida el comportamiento experimental, no la integración con la misión. Promover solo el módulo que superó criterios previos, junto con pruebas de regresión; documentar el cambio en `docs/decisions.md`.
- Sin controlador experimental, un experimento offline se mantiene en `experiments/exp_NNN_short_name/src/` y no requiere una carpeta bajo `controllers/`.
- Secuencia incremental: validar primero EXP-001–010 en la copia compartida de escenario 01; al cerrar esa línea base, ampliar a los escenarios/condiciones representativos definidos por `docs/test_plan.md`. La validación de navegación de ida y misión completa conserva la matriz de 10 escenarios × 3 poses.

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

## Escenarios seleccionados y casos

La selección por experimento está fijada en [Plan maestro de pruebas](test_plan.md) y justificada mediante el [catálogo de escenarios WBT](scenario_catalog.md). Consultar allí la matriz de 10 experimentos, las poses iniciales S1–S3, las pruebas finales de 30 corridas y el mecanismo de revisión. No interpretar una prueba superada en escenario 01 como éxito demostrado en los diez mundos.
