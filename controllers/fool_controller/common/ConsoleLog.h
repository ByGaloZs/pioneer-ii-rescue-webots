#ifndef RESCUE_CONSOLE_LOG_H_
#define RESCUE_CONSOLE_LOG_H_

#include <sys/stat.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

// Copia todo lo que el controlador escribe en consola (std::cout y std::cerr)
// a un archivo <proyecto>/logs/<mundo>_<fecha-hora>.log, sin dejar de mostrarlo
// en la consola de Webots. Así cada ejecución queda guardada para revisarla.
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
namespace console_log {

// streambuf que reenvía cada carácter a dos destinos: la consola original y el
// archivo. El archivo se vacía en cada salto de línea para no perder líneas si
// Webots detiene el controlador de golpe (p. ej. al recargar el mundo).
class TeeBuf : public std::streambuf {
public:
  TeeBuf(std::streambuf *console, std::streambuf *file) : _console(console), _file(file) {}

protected:
  int overflow(int c) override {
    if (c == traits_type::eof()) return traits_type::not_eof(c);
    const char ch = static_cast<char>(c);
    _console->sputc(ch);
    _file->sputc(ch);
    if (ch == '\n') _file->pubsync();
    return c;
  }
  int sync() override {
    _console->pubsync();
    return _file->pubsync();
  }

private:
  std::streambuf *_console;
  std::streambuf *_file;
};

// Mientras existe, std::cout y std::cerr se copian al archivo de log. Al
// destruirse devuelve la consola a su estado original.
class Session {
public:
  // tag: etiqueta del mundo (s01…s10) para el nombre del archivo.
  explicit Session(const std::string &tag) {
    // Webots ejecuta el controlador desde su carpeta: ../../logs = <proyecto>/logs.
    const std::string dir = "../../logs";
    ::mkdir(dir.c_str(), 0755);  // si ya existe no pasa nada
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", std::localtime(&now));
    _path = dir + "/" + tag + "_" + stamp + ".log";
    _file.open(_path);
    if (!_file) return;  // sin archivo: la consola sigue funcionando igual
    _coutOriginal = std::cout.rdbuf();
    _cerrOriginal = std::cerr.rdbuf();
    _coutTee = new TeeBuf(_coutOriginal, _file.rdbuf());
    _cerrTee = new TeeBuf(_cerrOriginal, _file.rdbuf());
    std::cout.rdbuf(_coutTee);
    std::cerr.rdbuf(_cerrTee);
  }
  ~Session() {
    if (_coutTee == nullptr) return;
    std::cout.flush();
    std::cerr.flush();
    std::cout.rdbuf(_coutOriginal);
    std::cerr.rdbuf(_cerrOriginal);
    delete _coutTee;
    delete _cerrTee;
  }
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  bool active() const { return _coutTee != nullptr; }
  // Ruta del archivo, relativa a la carpeta del controlador.
  const std::string &path() const { return _path; }

private:
  std::ofstream _file;
  std::string _path;
  std::streambuf *_coutOriginal = nullptr, *_cerrOriginal = nullptr;
  TeeBuf *_coutTee = nullptr, *_cerrTee = nullptr;
};

}  // namespace console_log

#endif  // RESCUE_CONSOLE_LOG_H_
