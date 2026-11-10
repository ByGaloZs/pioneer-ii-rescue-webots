# Catálogo de escenarios de evaluación (.wbt)

**Fuente:** los diez archivos `.wbt` originales de `Práctica06_src.zip`, restaurados sin modificaciones en `worlds/` el 10 de noviembre de 2026. Integridad y detalles en [worlds/README.md](../worlds/README.md). **El análisis de geometría es estático, no una medición de rendimiento del controlador.**

## Comparativa objetiva

| Escenario | Cajas grandes | Cajas pequeñas | Total cajas | Solids de estructura* | Niebla declarada | Luces declaradas | Uso sugerido |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| 01 | 0 | 0 | 0 | 30 | No | 18 PointLight | Geometría de paredes sin cajas |
| 02 | 11 | 12 | 23 | 27 | No | 18 PointLight + 3 SpotLight | Cajas e iluminación con focos |
| 03 | 15 | 14 | 29 | 25 | No | 11 PointLight + 5 SpotLight | Cajas y focos en varias zonas |
| 04 | 4 | 5 | 9 | 27 | EXPONENTIAL2, rango 15 | 12 PointLight | Pocos objetos + niebla |
| 05 | 6 | 5 | 11 | 25 | EXPONENTIAL2, rango 25 | 12 PointLight | Niebla más suave según parámetro |
| 06 | 6 | 10 | 16 | 25 | EXPONENTIAL2, rango 9 | 12 PointLight | Niebla de rango reducido |
| 07 | 6 | 10 | 16 | 23 | EXPONENTIAL2, rango 7 | 12 PointLight | Mínimo rango EXPONENTIAL2 |
| 08 | 10 | 10 | 20 | 26 | EXPONENTIAL, rango 50 | 18 PointLight | Niebla y configuración luminosa intensa |
| 09 | 11 | 9 | 20 | 26 | EXPONENTIAL, rango 30 | 18 PointLight | Víctimas muy separadas |
| 10 | 23 | 19 | 42 | 26 | No | 18 PointLight | Máxima cantidad de cajas |

\* **Estructura:** recuento de nodos `Solid` de tipo pared o estructura, excluyendo cajas `DEF LARGE_BOX`/`SMALL_BOX`, cilindros `PERSON1`/`PERSON2`, suelo y líneas. No equivale a segmentos libres, dificultad de navegación ni conectividad. Cada caja `LARGE_BOX` declara tamaño 0.6 × 0.6 × 0.6 m y cada `SMALL_BOX`, 0.4 × 0.4 × 0.4 m en el modelo inspeccionado.

### Diferencias de iluminación

- 01, 08 y 10 incluyen múltiples `PointLight` con intensidad declarada 50 y atenuación `0 0 12.56`.
- 02 utiliza varias luces puntuales de intensidad 0.05 y tres `SpotLight`; 03 contiene cinco `SpotLight`.
- 04–07 emplean niebla `EXPONENTIAL2`; 08–09 emplean `EXPONENTIAL`. **No comparar sus valores `visibilityRange` como si fueran una escala lineal común de visibilidad real.**
- La intensidad declarada de una luz no determina directamente la calidad de la imagen: también influyen atenuación, materiales, cámara y oclusiones.

### Víctimas

Las dos víctimas aparecen como cilindros verdes en el extremo positivo de Z. Para pruebas de identificación:
- **09:** víctimas en X ≈ −4.21 y +4.22; separación horizontal de aproximadamente 8.43 m (búsqueda espacial amplia).
- **10:** víctimas en (X,Z) ≈ (0.29,8.41) y (1.09,8.62); separación en planta ≈ 0.83 m (riesgo de confundir dos rescates cercanos).
- En **06 y 07** las posiciones declaradas de ambas víctimas coinciden, pero otras partes de la geometría difieren. No son pruebas controladas que solo cambien la niebla.

## Inicio y restricciones de los archivos

- El recinto es 10 × 20 m y la línea `STARTLINE` aparece en **Z = −8**; `FINISHLINE` en **Z = +8**.
- Los diez archivos originales declaran al robot en **(X, Z) = (0, −9.05)**, con orientación de eje Y de **π rad**. Es una pose de referencia del archivo, **no** una garantía sobre la pose aleatoria utilizada en competición.
- El enunciado establece inicio aleatorio **detrás de la línea amarilla**, por lo que se necesitarán copias de prueba con poses controladas que cumplan esa restricción.
- Posiciones preliminares para el protocolo: S1 = (−3, −8.8), S2 = (0, −8.8), S3 = (+3, −8.8), coordenadas **(X,Z)**, con orientaciones diferentes. Verificar **en Webots** ausencia de solapamiento inicial, orientación efectiva y sensor válido antes de utilizarlas.
- Las `.wbt` se utilizan como **verdad de referencia offline** y para preparar pruebas; el controlador nunca debe acceder a los nodos geométricos completos, nombres de escenario ni posiciones exactas de víctimas en tiempo de ejecución.

## Selección representativa

Conjunto principal **R = {01, 03, 07, 08, 10}**: distingue paredes sin cajas, focos/cajas, niebla `EXPONENTIAL2`, niebla `EXPONENTIAL` con otra iluminación y densidad alta de cajas. **09** se añade a pruebas visuales por separación de víctimas; **02, 04, 05 y 06** se reservan para cobertura final y diagnóstico de fallos.

La selección se fundamenta en rasgos medibles del archivo, **no** en una clasificación demostrada de dificultad. No sustituye la evaluación final en los diez escenarios.
