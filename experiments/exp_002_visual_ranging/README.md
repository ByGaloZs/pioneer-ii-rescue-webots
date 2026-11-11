# EXP-002 — Visual radial ranging

**Estado:** `in_progress` (capturas y prototipo offline disponibles; calibración métrica pendiente). **Hipótesis:** la cámara esférica puede producir observaciones radiales útiles como pseudo-LiDAR, con límites de alcance e incertidumbre explícitos.

**Escenario inicial común:** una copia de `worlds/final_scenario01.wbt`, guardada como `worlds/experiment_test_scenario01.wbt`. Se usa el mismo mundo de prueba para todos los experimentos en la primera fase; se regenera esa copia con los parámetros del ensayo activo. Los mundos canónicos `final_scenario*.wbt` no se modifican.

## Arquitectura compartida de pruebas

Todos los experimentos que requieran ejecución C++ en Webots usan **un único** `controllers/experiment_test_controller/`, con su propio punto de entrada y `Makefile`. Se edita/configura según el experimento activo; no se crea un controlador diferente por experimento y no se añaden loggers de prueba a `fool_controller`. El mundo compartido selecciona ese controlador y recibe una etiqueta explícita `experiment=002;trial=<id>`.

```bash
make -C controllers/experiment_test_controller
```

Webots debe estar disponible en `WEBOTS_HOME`. El logger de esta fase solo escribe en `experiments/exp_002_visual_ranging/results/`; requiere un identificador válido y falla si no encuentra esa ruta. Las corridas de otros experimentos usarán el mismo controlador y su propia raíz de resultados, según la implementación activa.

### Regenerar el único mundo de pruebas

El generador parte siempre de `final_scenario01.wbt`, conserva el escenario fuente y modifica solo la pose/GPS/cámara requeridos por el ensayo, el controlador y la etiqueta. Sin `--overwrite`, rechaza cambios respecto al archivo existente. Cada ensayo mantiene su configuración y etiqueta en el registro experimental antes de volver a generar la misma copia.

```bash
# Comparación inicial de inclinación a pose fija
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id camera_tilt45 --camera-tilt-rad -0.7854 --overwrite

# Cambiar la misma copia a inclinación de 30 grados
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id camera_tilt30 --camera-tilt-rad -0.5236 --overwrite

# Ensayo de pared P1; P2 y P3 usan Z=-8.8 y Z=-8.3 respectivamente
python3 experiments/exp_002_visual_ranging/src/create_pilot_world.py \
  --trial-id calib01_tilt30_p1 --z -9.0 --camera-tilt-rad -0.5236 \
  --strip-camera-debug --overwrite
```

Abrir `worlds/experiment_test_scenario01.wbt`, ejecutar y guardar cada corrida antes de regenerar el mundo para el siguiente ensayo. Los parámetros físicos se registran fuera del controlador; este no lee geometría WBT ni distancias de referencia. La misma copia común se usará primero para EXP-001–010 durante desarrollo; la matriz de escenarios adicionales se ejecutará después de validar el comportamiento base, según `docs/test_plan.md`.

## V3 — observaciones angulares y geometría (prototipo offline)

**Estado:** prototipo implementado, sin validación métrica en Webots ni integración con el mapa. La cámara esférica actual tiene resolución 160×160 y FOV horizontal aproximado de π rad (**180°, no 360°**). Medir radios desde el centro de la imagen no corresponde a un barrido LiDAR.

Módulos principales:
- `src/spherical_geometry.py`: rayos por píxel, cámara con tilt nominal −30°, altura 0,44 m y desplazamiento de montaje. Calcula intersecciones candidatas con suelo plano y descarta rayos al cielo/horizonte.
- `src/angular_observations.py`: sectores por columnas, segmentación HSV conservadora y transición **suelo visible → obstáculo visible**. Autooclusión, desacuerdo o geometría inválida producen `unknown` y distancia nula; nunca se infiere espacio libre.
- `src/segment_image.py`: V3 angular por defecto; el algoritmo circular antiguo solo está disponible como `--mode radial-legacy` para reproducibilidad, no para métricas.
- `src/analyze_runs.py`: analiza corridas etiquetadas y produce salidas V3 separadas, sin modificar los CSV originales.

El eje óptico asumido es `-Z`; píxeles a la derecha se tratan como azimut positivo. El sentido y el montaje deben verificarse con un marcador lateral en Webots antes de usar rumbos en el mapa. La intersección suelo-rayo solo es candidata para contacto visible sobre suelo plano, no distancia a cualquier superficie. La confianza 0,5 es heurística y el alcance [0,2; 4,0] m es un filtro provisional, no un resultado medido.

## Ejecución del piloto y análisis

1. Compilar `experiment_test_controller` y abrir la copia única de prueba en Webots.
2. Ejecutar una sola configuración etiquetada, verificar en consola `trial_id`, pose, cámara y captura guardada.
3. Inspeccionar primero el JPEG bruto. Registrar suelo, pared, carrocería, condiciones y fallos; no modificar datos de una corrida anterior.
4. Para la siguiente pose/inclinación, regenerar **el mismo** `worlds/experiment_test_scenario01.wbt` con `--overwrite`, abrir/reiniciar y guardar la corrida nueva.
5. No calcular metros hasta contar con frontera suelo-pared visible, máscara de autooclusión verificada y referencia independiente.

Las capturas existentes P1/P2/P3 (distancias nominales centro-robot–pared de 0,8/1,0/1,5 m a 1,024 s) mostraron contorno superior y zonas de carrocería, pero no confirmaron una frontera suelo-pared continua. Por tanto **no demuestran rangos métricos ni MAE**.

Las etiquetas `calib01_*` capturan desde 1 s simulado y finalizan tras guardar la primera imagen; las otras etiquetas capturan periódicamente. El identificador es solo metadato, nunca aporta geometría o distancia al controlador.

Pruebas offline del generador y del prototipo:

```bash
python3 -m pip install -r experiments/exp_002_visual_ranging/requirements.txt
python3 -m unittest discover -s experiments/exp_002_visual_ranging/tests -v
```

Procesar una captura con `segment_image.py` o `analyze_runs.py`. Sin una máscara de autooclusión **verificada visualmente para la orientación utilizada**, las salidas deben permanecer `unknown`; no reportar MAE. Los sectores `occupied` son candidatos geométricos, no evidencia aceptada para integrar al mapa. `camera_f` permanece reservada para detección de víctimas, fuera del alcance de EXP-002.

## Métricas y aceptación

- Error absoluto por sector/distancia, proporción de rayos utilizables, falsos libres y sensibilidad a niebla/iluminación.
- Aceptar para integración únicamente con alcance utilizable y errores medidos en referencias independientes, además de tratamiento `unknown` para observaciones dudosas.
- Empezar en escenario 01; solo después de validar allí, confirmar robustez en 03, 07, 08 y 10 antes de conectar observaciones visuales al mapeo.

La captura previa `run_20261110_174034` tenía GPS cuantizado a 3 m y los 16 sectores reportaron `near_field_occluded=true`; sus tres límites visuales pueden corresponder a la silueta del robot. La máscara `config/self_occlusion_pilot.json` sigue sin verificar y corresponde a la cámara mirando hacia abajo. No reutilizarla automáticamente para otras inclinaciones.
