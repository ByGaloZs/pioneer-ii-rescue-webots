#ifndef RESCUE_FLOOR_SEGMENTER_H_
#define RESCUE_FLOOR_SEGMENTER_H_

#include <vector>

// Punto del suelo donde termina el espacio libre en una columna de la imagen,
// expresado en el marco del robot (metros desde el centro del robot):
//   forwardM  hacia delante,  leftM  hacia la izquierda.
struct FreeSpacePoint {
  double forwardM;
  double leftM;
  bool closeObstacle;  // el obstáculo llega al borde inferior: está a menos de ~1 m
  bool victim = false; // lo que termina el espacio libre en esta columna es verde
  double rawForwardM = 0.0;  // distancia sin recortar por el alcance de confianza
};

// Víctima (cilindro verde) vista por la cámara frontal: punto de su base más
// cercano, en el marco del robot (desde el centro del robot).
struct VictimBlob {
  double forwardM = 0.0;
  double leftM = 0.0;
  int columns = 0;
};

// Perfil de espacio libre visto por la cámara frontal.
struct FreeSpaceProfile {
  bool valid = false;
  std::vector<FreeSpacePoint> points;  // uno por columna procesada
  std::vector<VictimBlob> victims;
  int victimPixels = 0;                // píxeles verdes (para la fase de víctimas)
  int floorBrightness = 0;             // brillo del suelo de referencia (0-255)

  // Distancia libre (m, desde el centro del robot) a lo largo de una dirección
  // relativa al robot (rad, positivo = izquierda) para un pasillo del ancho del
  // robot. Devuelve maxRangeM si no hay nada.
  double corridorClearance(double relativeHeadingRad, double halfWidthM, double maxRangeM) const;
};

// Segmenta la imagen de la cámara frontal en suelo / obstáculo (cian) /
// víctima (verde) y, recorriendo cada columna de abajo hacia el horizonte,
// obtiene dónde empieza el primer obstáculo. Supone suelo plano y cámara
// horizontal (montaje original del Pioneer2: altura 0.255 m, 0.155 m delante).
//
// Robusto a la iluminación de cada escenario:
//  - En cada imagen aprende el color del suelo con la franja inferior central
//    (mediana), y clasifica cada píxel RELATIVO a ese color (cociente g/r y
//    b/r), no con umbrales fijos.
//  - Si el suelo de referencia es demasiado oscuro, el perfil sale no válido y
//    el navegador funciona solo con infrarrojos.
//  - Píxeles casi negros (no se sabe qué son) y la zona lejana (niebla) se
//    tratan de forma conservadora: no cuentan como espacio libre.
class FloorSegmenter {
public:
  FloorSegmenter(int width, int height, double horizontalFovRad);

  // image: buffer BGRA de webots::Camera::getImage().
  // Procesa una imagen y regresa el perfil de espacio libre por columna.
  FreeSpaceProfile process(const unsigned char *image) const;

private:
  // Convierte una fila de la imagen en distancia al suelo (modelo de cámara plana).
  double groundForwardM(int row) const;  // distancia al suelo visto en esa fila

  int _width;
  int _height;
  double _focalPx;
  // Se procesa una columna de cada _columnStep para ahorrar tiempo (~60 columnas).
  int _columnStep;
};

#endif  // RESCUE_FLOOR_SEGMENTER_H_
