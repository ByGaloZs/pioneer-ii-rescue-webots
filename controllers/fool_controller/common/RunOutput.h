#ifndef RESCUE_RUN_OUTPUT_H_
#define RESCUE_RUN_OUTPUT_H_

#include <sys/stat.h>
#include <unistd.h>

#include <cctype>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <string>

// Dónde se guardan los resultados y cómo se etiqueta cada prueba.
// Webots ejecuta el controlador con la carpeta del controlador como directorio
// actual, así que "../../results" es <proyecto>/results (junto a worlds/).
namespace run_output {

inline std::string directory() {
  const std::string dir = "../../results";
  ::mkdir(dir.c_str(), 0755);  // si ya existe no pasa nada
  return dir;
}

inline std::string path(const std::string &fileName) { return directory() + "/" + fileName; }

// Busca "final_scenarioNN" en un texto (p. ej. la línea de órdenes de Webots).
inline std::string findScenario(const std::string &text) {
  const std::string key = "scenario";
  for (size_t pos = text.find(key); pos != std::string::npos; pos = text.find(key, pos + 1)) {
    size_t i = pos + key.size();
    std::string digits;
    while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])) &&
           digits.size() < 2)
      digits += text[i++];
    if (!digits.empty()) return "s" + digits;
  }
  return "";
}

// Mejor esfuerzo: Webots suele abrirse con la ruta del mundo en su línea de
// órdenes (o como proceso padre del controlador). Devuelve "" si no se encuentra.
inline std::string detectWorld() {
  if (const char *env = std::getenv("WEBOTS_WORLD")) {
    std::string found = findScenario(env);
    if (!found.empty()) return found;
  }
  pid_t pid = ::getppid();
  for (int level = 0; level < 4 && pid > 1; ++level) {
    const std::string base = "/proc/" + std::to_string(pid);
    std::ifstream cmd(base + "/cmdline", std::ios::binary);
    std::string text((std::istreambuf_iterator<char>(cmd)), std::istreambuf_iterator<char>());
    std::string found = findScenario(text);
    if (!found.empty()) return found;
    std::ifstream stat(base + "/stat");
    std::string line;
    std::getline(stat, line);
    const size_t close = line.rfind(')');  // "pid (nombre) estado ppid ..."
    if (close == std::string::npos) break;
    pid = static_cast<pid_t>(std::atoi(line.c_str() + close + 4));
  }
  return "";
}

inline std::string timestamp() {
  char buffer[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(buffer, sizeof buffer, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
  return buffer;
}

// Prioridad: tag= en controllerArgs > mundo detectado > "s??".
inline std::string resolveTag(const std::string &requested) {
  if (!requested.empty()) return requested;
  const std::string world = detectWorld();
  return world.empty() ? "s??" : world;
}

}  // namespace run_output

#endif  // RESCUE_RUN_OUTPUT_H_
