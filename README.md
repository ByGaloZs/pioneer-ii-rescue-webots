# Rescate autónomo con Pioneer II

Controlador de navegación autónoma en **Webots (C++)** para el robot **Pioneer II**. El sistema debe explorar un entorno desconocido, evitar colisiones, localizar a dos víctimas representadas por cilindros verdes y regresar a la zona de salida.

## Objetivo de la misión

El controlador se evaluará en uno de diez escenarios de **10 × 20 m**, con diferentes configuraciones de obstáculos e iluminación. La posición y orientación iniciales del robot pueden variar dentro de la zona situada detrás de la línea amarilla.

La misión consiste en:

1. Salir de la zona inicial y llegar al extremo opuesto de la arena sin colisionar.
2. Detectar los **dos cilindros verdes** que representan a las víctimas.
3. Acercarse a **menos de 1 m** de cada víctima, detenerse, efectuar un giro completo de **360°** e imprimir un mensaje de rescate.
4. Regresar a la zona de partida.
5. Completar la misión en el menor tiempo posible, con un límite de **240 segundos**.

## Enfoque de navegación

Se desarrollará un sistema de **exploración dirigida al objetivo con mapas de ocupación incrementales**. No se requiere identificar cuál de los diez escenarios se está ejecutando ni disponer de un mapa completo al comenzar.

La arquitectura separa cinco responsabilidades:

| Módulo | Responsabilidad |
| --- | --- |
| **Localización** | Estimar pose (posición y orientación) con GPS y brújula; apoyar movimientos cortos con encoders. |
| **Percepción** | Detectar obstáculos y zonas potencialmente transitables mediante los 16 sensores de distancia y las cámaras frontal y esférica. |
| **Mapeo incremental** | Integrar las observaciones en una rejilla de ocupación con celdas **libres**, **ocupadas** y **desconocidas**. |
| **Exploración y planificación** | Seleccionar fronteras que favorezcan el progreso y planificar rutas mediante **A\*** sobre espacio confirmado como transitable. |
| **Control de movimiento** | Seguir rutas locales, ajustar velocidades y priorizar la evasión de obstáculos. |

### Mapa local y mapa acumulado

- El **mapa local** describe el entorno cercano al robot a partir de las lecturas actuales. Sirve para detectar pasos, obstáculos y situaciones de riesgo inmediato.
- El **mapa acumulado** incorpora las observaciones en un marco de coordenadas común, conserva lo descubierto durante la navegación y ayuda a evitar bucles y a planificar el regreso.
- Las observaciones se registrarán considerando la incertidumbre de las mediciones. Una zona no observada **no** se tratará automáticamente como libre.
- Antes de planificar, los obstáculos se ampliarán con un margen de seguridad acorde al tamaño del Pioneer II y al error de percepción.

### Frontier-Based Exploration dirigida al objetivo

Una **frontera** es una celda libre conocida que limita con una región desconocida. En lugar de explorar siempre la frontera más cercana, el robot evaluará las fronteras **alcanzables** según:

- Progreso esperado hacia el extremo opuesto de la arena.
- Longitud o coste de la ruta necesaria para llegar.
- Seguridad y anchura estimada del paso.
- Información nueva que puede descubrirse y penalización por visitas repetidas o rutas bloqueadas.

Una frontera se selecciona como **objetivo intermedio**. **A\*** encuentra una ruta hasta ella a través de celdas verificadas como libres; el controlador local ejecuta el movimiento. Cuando aparecen nuevas observaciones o una ruta deja de ser válida, se actualiza el mapa y se replantea la trayectoria.

Si no existe una ruta conocida hacia el destino final, se sigue explorando mediante fronteras. Si una región está bloqueada, se registra para evitar intentos repetidos.

### Cámara esférica como sensor visual radial

Se investigará el uso de la **cámara de 360° como pseudo-LiDAR visual**, siguiendo el principio de segmentación radial de la práctica de procesamiento de imágenes:

1. Segmentar suelo, paredes y obstáculos visibles.
2. Identificar límites radiales en la imagen esférica.
3. Calibrar la relación entre posición del límite en píxeles y distancia real, cuando sea medible.
4. Fusionar la información visual con los sensores de distancia.

**La cámara no proporciona distancias métricas directamente.** Su utilidad para el mapeo dependerá de pruebas de calibración, iluminación, oclusiones y geometría del escenario. Los sensores de distancia tendrán prioridad en las decisiones inmediatas de seguridad. La detección visual de los cilindros verdes se desarrollará como una función independiente.

## Secuencia de operación prevista

| Estado | Acción |
| --- | --- |
| `INITIALIZE` | Iniciar dispositivos, verificar lecturas y registrar la pose inicial. Mantener los motores detenidos. |
| `INITIAL_SCAN` | Obtener las primeras observaciones y construir el mapa local inicial. |
| `NAVIGATE` | Orientar el movimiento hacia el extremo opuesto y navegar por espacio libre conocido. |
| `EXPLORE` | Seleccionar una frontera y calcular una ruta con A*. |
| `AVOID_OBSTACLE` | Ejecutar maniobras de seguridad y replanificar cuando sea necesario. |
| `APPROACH_VICTIM` | Detectar una víctima, estimar una aproximación segura y comprobar la distancia. |
| `RESCUE` | Detenerse a menos de 1 m, girar 360° y registrar el rescate sin duplicarlo. |
| `RETURN` | Planificar con A* sobre el mapa acumulado para volver al inicio. |
| `FINISH` | Detener el robot y reportar el resultado de la misión. |

Estos nombres describen el **diseño previsto** de la máquina de estados; todavía no están implementados.

## Sensores y configuración

El controlador dispone de:

- **GPS:** posición estimada en el sistema de coordenadas de Webots.
- **Brújula:** orientación del robot.
- **Encoders:** desplazamiento relativo de las ruedas.
- **16 sensores de distancia:** detección de obstáculos con perfiles de respuesta calibrados.
- **Cámara frontal (`camera_f`)** y **cámara esférica (`camera_s`)**: adquisición de imágenes para percepción y detección de víctimas.
- **Dos motores diferenciales:** control independiente de las ruedas.

Los parámetros de los dispositivos y del modelo cinemático están centralizados en `RobotConfig.h`. Las lecturas de distancia se convierten mediante `DistanceSensorCalibration`; sus resultados son **estimaciones** y deben respetarse los rangos válidos y las saturaciones de cada sensor.

## Estructura del proyecto

```text
.
├── AGENTS.md
├── README.md
├── .gitignore
├── controllers/fool_controller/
│   ├── fool_controller.cpp
│   ├── Makefile
│   ├── config/                     # Configuración existente
│   ├── sensors/                    # Lectura y calibración existente
│   ├── control/                    # Actuación existente
│   ├── localization/               # Planificado
│   ├── perception/                 # Planificado
│   ├── mapping/                    # Planificado
│   ├── exploration/                # Planificado
│   ├── planning/                   # Planificado
│   └── mission/                    # Planificado
├── docs/
│   ├── architecture.md
│   ├── decisions.md
│   └── experiments.md
├── experiments/
│   ├── README.md
│   ├── _template/
│   └── exp_001_sensor_validation/
├── tests/
├── protos/
└── worlds/
```

Las carpetas marcadas como *planificadas* están reservadas para futuras implementaciones; no contienen algoritmos todavía. Se conserva el nombre `fool_controller` porque los escenarios disponibles lo referencian.

### Guía de desarrollo

- [AGENTS.md](AGENTS.md): reglas para asistentes y colaboradores.
- [Arquitectura](docs/architecture.md): módulos y flujo de información.
- [Decisiones](docs/decisions.md): justificación y estado de elecciones técnicas.
- [Protocolo experimental](docs/experiments.md): pruebas reproducibles y métricas.
- [Plan maestro de pruebas](docs/test_plan.md): matriz EXP-001–010, escenarios, arranques y criterios.
- [Catálogo de escenarios](docs/scenario_catalog.md): comparación estática de los diez archivos WBT.
- [Experimentos](experiments/README.md): plantillas y registro de cada prueba.

## Estado de desarrollo

**Fase actual: inicialización y lectura de sensores.**

El ejecutable actual:

- Inicializa motores y dispositivos; mantiene las ruedas detenidas.
- Informa del tamaño de las dos cámaras.
- Guarda la primera pose GPS/brújula válida.
- Muestra periódicamente el estado del GPS, brújula, encoders y sensores de distancia.

**Todavía no se han implementado** el mapa de ocupación, la reconstrucción radial de distancias por visión, Frontier-Based Exploration, A*, la navegación autónoma, la detección/rescate de víctimas ni el retorno.

### Plan de implementación y validación

1. **Arranque seguro:** comprobar lecturas y orientación en los diez escenarios con distintas poses iniciales.
2. **Percepción local:** validar rangos y cobertura de los 16 sensores; evaluar experimentalmente la cámara esférica como pseudo-LiDAR.
3. **Mapa incremental:** representar celdas libres, ocupadas y desconocidas; aplicar márgenes de seguridad y validar contra obstáculos observables.
4. **Planificador:** seleccionar fronteras útiles, implementar A* y comprobar replanificación ante bloqueos.
5. **Navegación de ida:** llegar al extremo opuesto en diferentes escenarios sin rescates, registrando tiempo, progreso y colisiones.
6. **Rescate y regreso:** localizar dos víctimas, confirmar cada rescate y volver al área de salida.
7. **Evaluación integral:** ejecutar los diez escenarios y medir tasa de éxito, tiempo, colisiones, víctimas detectadas y recorrido de regreso.

## Ejecutar en Webots

1. Abrir `worlds/final_scenario01.wbt` en Webots.
2. Compilar `controllers/fool_controller/` desde Webots o ejecutar `make` en esa carpeta con `WEBOTS_HOME` configurado.
3. Iniciar la simulación.
4. Comprobar en la consola los mensajes `[STARTUP]`, `[CAMERAS]`, `[START]` y `[SENSORS]`.
5. Confirmar que **el robot permanece detenido**: en esta fase solamente se diagnostican los sensores.

### Prueba independiente de calibración

Desde la raíz del proyecto:

```bash
g++ -std=c++11 -Wall -Wextra -pedantic \
    tests/sensor_calibration_test.cpp \
    controllers/fool_controller/sensors/DistanceSensorCalibration.cpp \
    -o /tmp/sensor_calibration_test
/tmp/sensor_calibration_test
```

> **Validación pendiente:** compilar y ejecutar el controlador completo dentro de Webots para verificar el comportamiento de los dispositivos en los escenarios de evaluación.
