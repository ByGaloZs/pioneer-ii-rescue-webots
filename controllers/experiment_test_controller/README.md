# Controlador de pruebas compartido

Este es el único controlador de pruebas Webots, reutilizado por los experimentos que requieren ejecución C++. Se selecciona desde la copia única `worlds/experiment_test_scenario01.wbt` mediante `controller "experiment_test_controller"`. El argumento de corrida identifica el experimento activo y su trial, por ejemplo `experiment=002;trial=calib01_tilt30_p1` o `experiment=002;trial=geom01_tilt30_front_r01`.

## Compilación

Desde la raíz del proyecto, con `WEBOTS_HOME` configurado:

```bash
make -C controllers/experiment_test_controller
```

El `Makefile` compila este punto de entrada y enlaza explícitamente los módulos estables de sensores/control y el código runtime que requiere el experimento activo. No copia ni reemplaza el ejecutable de `fool_controller`.

## Ejecución y resultados

Abrir el único mundo de pruebas y ejecutar la simulación. El proceso falla si falta una etiqueta válida o si no puede abrir el directorio del experimento seleccionado. En la configuración actual, acepta EXP-002, captura la cámara esférica y escribe únicamente en `experiments/exp_002_visual_ranging/results/`. Las etiquetas `calib01_*` capturan una imagen al segundo simulado; `geom01_*` guarda una captura inicial y otra al segundo para comparar pose de arranque y pose asentada, y luego finaliza el logger. Las demás etiquetas capturan periódicamente. Al cambiar de experimento se adapta este mismo controlador y su selección explícita; no se crea otro controlador.

El controlador mantiene los motores detenidos y recoge telemetría para el piloto. No lee la geometría del mundo ni usa la referencia de distancia. La configuración runtime actual pertenece a EXP-002; antes de iniciar EXP-003 se adapta este mismo harness a la comparación A/B/C de cámara, proximidad y fusión, con resultados bajo la carpeta de EXP-003. No crear otro controlador. Este harness no forma parte de la misión final; cualquier módulo que se promueva requiere evidencia y regresiones antes de integrarse.
