#!/usr/bin/env python3
"""Visor en directo del mapa que construye el robot mientras corre la simulación.

El controlador escribe results/live_map.txt cada 0,5 s de simulación (solo si hay
mapa activado; "nolive" en controllerArgs lo desactiva). Este script lo lee y lo
redibuja. No toca el robot ni la simulación: puede abrirse y cerrarse cuando quieras.

Uso (desde cualquier carpeta):
    python3 tools/live_map.py                  # usa matplotlib, tkinter o terminal
    python3 tools/live_map.py --terminal       # fuerza el modo terminal (solo Python)
    python3 tools/live_map.py --world worlds/final_scenario03.wbt   # dibuja las paredes reales

Leyenda: gris oscuro = ocupado (pared vista), blanco = libre visto, azul = ya
recorrido, flecha roja = robot, línea verde = plan A*, línea amarilla = meta.
Con --world se superponen en gris claro las paredes REALES del escenario, para ver
qué ha descubierto ya el robot y si hay paredes fantasma.
"""
import argparse
import math
import os
import re
import sys
import time

# results/ está junto a worlds/, un nivel por encima de tools/ (funciona desde cualquier carpeta).
DEFAULT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "live_map.txt")
X_MIN, X_MAX, Z_MIN, Z_MAX = -5.0, 5.0, -10.0, 10.0


def read_frame(path):
    """Devuelve dict con t, state, x, z, yaw, goalz, plan, rows; o None si no se puede leer."""
    try:
        with open(path) as f:
            lines = f.read().split("\n")
    except OSError:
        return None
    frame = {"t": 0.0, "state": "?", "x": 0.0, "z": 0.0, "yaw": 0.0, "goalz": 8.6, "plan": [], "victims": [], "rows": []}
    try:
        i = 0
        while i < len(lines) and lines[i] != "grid":
            key, _, value = lines[i].partition("=")
            if key == "t":
                frame["t"] = float(value)
            elif key == "state":
                frame["state"] = value
            elif key == "pose":
                frame["x"], frame["z"], frame["yaw"] = (float(v) for v in value.split())
            elif key == "goalz":
                frame["goalz"] = float(value)
            elif key == "victims":
                frame["victims"] = [tuple(float(c) for c in v.split(",")) for v in value.split() if v.count(",") == 2]
            elif key == "plan":
                frame["plan"] = [tuple(float(c) for c in p.split(",")) for p in value.split() if "," in p]
            i += 1
        frame["rows"] = [r for r in lines[i + 1:] if r]
    except ValueError:
        return None
    return frame if len(frame["rows"]) >= 100 else None


def load_world_walls(path):
    """Rectángulos (centro x, z, yaw, tamaño x, z) de las paredes/obstáculos de un .wbt.

    Lectura simplificada: Solid con Box y rotación en torno a Y. Suficiente para ver
    el escenario al lado del mapa; no sustituye a Webots.
    """
    text = open(path, encoding="utf-8", errors="ignore").read()
    boxes = []
    for block in re.split(r"\n(?=(?:DEF \w+ )?Solid \{)", text):
        if not re.match(r"(?:DEF \w+ )?Solid \{", block):
            continue
        tr = re.search(r"^\s*translation ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)", block, re.M)
        box = re.search(r"geometry Box \{\s*size ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)", block)
        if not tr or not box:
            continue
        x, _, z = map(float, tr.groups())
        sx, sy, sz = map(float, box.groups())
        if sy < 0.05 or (sx >= 9.9 and sz >= 19.9):  # líneas del suelo y suelo
            continue
        rot = re.search(r"^\s*rotation ([-\d.e]+) ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)", block, re.M)
        yaw = 0.0
        if rot:
            ax, ay, az, ang = map(float, rot.groups())
            yaw = ang * (1 if ay >= 0 else -1) if abs(ay) > 0.5 else 0.0
        boxes.append((x, z, yaw, sx, sz))
    return boxes


def wall_mask(boxes):
    """Matriz [fila de arriba a abajo][columna] de booleanos a 10 cm."""
    mask = [[False] * 100 for _ in range(200)]
    for r in range(200):
        z = Z_MAX - (r + 0.5) * 0.1
        for c in range(100):
            x = X_MIN + (c + 0.5) * 0.1
            for bx, bz, yaw, sx, sz in boxes:
                dx, dz = x - bx, z - bz
                u = dx * math.cos(yaw) - dz * math.sin(yaw)
                v = dx * math.sin(yaw) + dz * math.cos(yaw)
                if abs(u) <= sx / 2 and abs(v) <= sz / 2:
                    mask[r][c] = True
                    break
    return mask


def heading_vector(yaw):
    return -math.sin(yaw), -math.cos(yaw)  # (dx, dz) del convenio del proyecto


# ---------------------------------------------------------------- matplotlib
def run_matplotlib(args, mask):
    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    # 0 desconocido, 1 libre, 2 recorrido, 3 ocupado, 4 pared real aún sin descubrir
    cmap = ListedColormap(["#f4f1ea", "#ffffff", "#9ec5ee", "#2b2b2b", "#d9a8a8"])
    plt.ion() if not args.save else None
    fig, ax = plt.subplots(figsize=(5.2, 9.2))
    image = ax.imshow([[0] * 100] * 200, cmap=cmap, vmin=0, vmax=4, origin="upper",
                      extent=(X_MIN, X_MAX, Z_MIN, Z_MAX), interpolation="nearest")
    plan_line, = ax.plot([], [], "-", color="#2e9e44", lw=1.8, label="plan A*")
    robot_dot, = ax.plot([], [], "o", color="#d62828", ms=8)
    victim_dots, = ax.plot([], [], "o", color="#2dbd3a", mec="k", ms=11, ls="none", label="víctimas")
    rescued_dots, = ax.plot([], [], "P", color="#2dbd3a", mec="k", ms=12, ls="none")
    arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="#d62828", lw=2))
    goal_line = ax.axhline(8.6, color="#f4b400", lw=1.5, ls="--")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)   arriba = víctimas")
    ax.set_aspect("equal")
    title = ax.set_title("esperando a la simulación...")
    last_mtime = 0.0
    while True:
        try:
            mtime = os.path.getmtime(args.file)
        except OSError:
            mtime = 0.0
        if mtime != last_mtime:
            frame = read_frame(args.file)
            if frame:
                last_mtime = mtime
                code = {"#": 3, "o": 2, ".": 1, " ": 0}
                data = [[code.get(ch, 0) for ch in row[:100].ljust(100)] for row in frame["rows"][:200]]
                if mask:
                    for r in range(200):
                        for c in range(100):
                            if mask[r][c] and data[r][c] != 3:
                                data[r][c] = 4
                image.set_data(data)
                goal_line.set_ydata([frame["goalz"], frame["goalz"]])
                if frame["plan"]:
                    plan_line.set_data([p[0] for p in frame["plan"]], [p[1] for p in frame["plan"]])
                else:
                    plan_line.set_data([], [])
                robot_dot.set_data([frame["x"]], [frame["z"]])
                pend = [v for v in frame["victims"] if v[2] < 2]
                done = [v for v in frame["victims"] if v[2] >= 2]
                victim_dots.set_data([v[0] for v in pend], [v[1] for v in pend])
                rescued_dots.set_data([v[0] for v in done], [v[1] for v in done])
                dx, dz = heading_vector(frame["yaw"])
                arrow.xy = (frame["x"] + 0.7 * dx, frame["z"] + 0.7 * dz)
                arrow.set_position((frame["x"], frame["z"]))
                occupied = sum(row.count("#") for row in frame["rows"])
                title.set_text("t = %.1f s   %s\npos (%.2f, %.2f)   celdas ocupadas: %d"
                               % (frame["t"], frame["state"], frame["x"], frame["z"], occupied))
                if args.save:
                    fig.savefig(args.save, dpi=110)
                    return
                fig.canvas.draw_idle()
        if args.save:
            print("no hay datos en", args.file)
            return
        if not plt.fignum_exists(fig.number):
            return
        plt.pause(0.25)


# ------------------------------------------------------------------ terminal
def run_terminal(args, mask):
    """Sin dependencias: redibuja en la terminal (mitad de resolución: 20 cm por carácter)."""
    last_mtime = 0.0
    print("Esperando a", args.file, "(Ctrl+C para salir)")
    while True:
        try:
            mtime = os.path.getmtime(args.file)
        except OSError:
            mtime = 0.0
        if mtime != last_mtime:
            frame = read_frame(args.file)
            if frame:
                last_mtime = mtime
                rows = frame["rows"]
                canvas = []
                for r in range(0, 200, 4):  # 40 cm de alto por línea (los caracteres son altos)
                    line = ""
                    for c in range(0, 100, 2):
                        block = [rows[rr][cc] if rr < len(rows) and cc < len(rows[rr]) else " "
                                 for rr in range(r, min(r + 4, 200)) for cc in (c, c + 1)]
                        if "#" in block:
                            ch = "#"
                        elif "o" in block:
                            ch = "o"
                        elif "." in block:
                            ch = "."
                        else:
                            ch = " "
                        if ch != "#" and mask and any(mask[rr][cc] for rr in range(r, min(r + 4, 200))
                                                      for cc in (c, c + 1)):
                            ch = "+"
                        line += ch
                    canvas.append(list(line))

                def put(x, z, ch):
                    c = int((x - X_MIN) / 0.2)
                    r = int((Z_MAX - z) / 0.4)
                    if 0 <= r < len(canvas) and 0 <= c < len(canvas[r]):
                        canvas[r][c] = ch

                for px, pz in frame["plan"]:
                    put(px, pz, "*")
                for vx, vz, vs in frame.get("victims", []):
                    put(vx, vz, "V" if vs >= 2 else "v")
                put(frame["x"], frame["z"], "R")
                sys.stdout.write("\x1b[2J\x1b[H")
                print("t=%.1f s  %s  pos=(%.2f, %.2f)   # pared  . libre  o recorrido  * plan  R robot%s"
                      % (frame["t"], frame["state"], frame["x"], frame["z"], "  + pared real sin descubrir" if mask else ""))
                for line in canvas:
                    print("".join(line))
        time.sleep(0.3)


def main():
    parser = argparse.ArgumentParser(description="Visor en directo del mapa del robot")
    parser.add_argument("--file", default=DEFAULT_FILE, help="archivo de mapa (por defecto <proyecto>/results/live_map.txt)")
    parser.add_argument("--world", help="mundo .wbt para superponer las paredes reales")
    parser.add_argument("--terminal", action="store_true", help="dibujar en la terminal (sin librerías)")
    parser.add_argument("--save", help="guardar una imagen PNG del estado actual y salir")
    args = parser.parse_args()

    mask = wall_mask(load_world_walls(args.world)) if args.world else None
    try:
        if args.terminal:
            run_terminal(args, mask)
        else:
            try:
                import matplotlib  # noqa: F401
            except ImportError:
                print("No hay matplotlib: uso el modo terminal (instalar: pip install matplotlib).")
                run_terminal(args, mask)
            else:
                run_matplotlib(args, mask)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
