#!/usr/bin/env python3
"""
2x2scaner.py - Clean, modular web server for scanning a 2x2 Rubik's Cube.
Serves scan/index.html and integrates with robot.solver for BFS action planning.
Zero emojis, high performance, and minimal code.
"""

import http.server
import socketserver
import json
import sys
import time
import tempfile
import multiprocessing as mp
from pathlib import Path

# Add workspace root to system path for imports
workspace_root = Path(__file__).resolve().parent.parent
if str(workspace_root) not in sys.path:
    sys.path.insert(0, str(workspace_root))

try:
    from robot.solver import COLOR_IDX, bfs_solve, print_cube
    from robot.generate_taskfile import generate_manipulation_problem, run_fast_downward
    SOLVER_AVAILABLE = True
except ImportError as e:
    SOLVER_AVAILABLE = False
    print(f"Warning: robot modules not available. Solving locally is disabled. Error: {e}")

PORT = 5000
FACE_ORDER = ["U", "F", "R", "B", "L", "D"]
SOLVER_TIMEOUT_SECONDS = 12

def build_solver_state(face_data):
    """Maps the 6 scanned faces (4 facelets each) to the 24-sticker solver state.

    face_data[face] order: [TL, TR, BL, BR] viewed from outside that face.
    Same convention as SCAN_CORNERS in scan/index.html.
    """
    if not SOLVER_AVAILABLE:
        return None

    color_to_idx = {
        "W": COLOR_IDX["white"],
        "Y": COLOR_IDX["yellow"],
        "R": COLOR_IDX["red"],       # Rosa/Pink is mapped to Red (R)
        "O": COLOR_IDX["orange"],
        "B": COLOR_IDX["blue"],
        "G": COLOR_IDX["green"],
    }

    state = [0] * 24

    # U face
    state[0] = color_to_idx[face_data["U"][3]]   # 'u-ufr' -> U[3]
    state[1] = color_to_idx[face_data["U"][2]]   # 'u-ufl' -> U[2]
    state[2] = color_to_idx[face_data["U"][1]]   # 'u-ubr' -> U[1]
    state[3] = color_to_idx[face_data["U"][0]]   # 'u-ubl' -> U[0]
    
    # D face
    state[4] = color_to_idx[face_data["D"][1]]   # 'd-dfr' -> D[1]
    state[5] = color_to_idx[face_data["D"][0]]   # 'd-dfl' -> D[0]
    state[6] = color_to_idx[face_data["D"][3]]   # 'd-dbr' -> D[3]
    state[7] = color_to_idx[face_data["D"][2]]   # 'd-dbl' -> D[2]
    
    # F face
    state[8] = color_to_idx[face_data["F"][1]]   # 'f-ufr' -> F[1]
    state[9] = color_to_idx[face_data["F"][0]]   # 'f-ufl' -> F[0]
    state[10] = color_to_idx[face_data["F"][3]]  # 'f-dfr' -> F[3]
    state[11] = color_to_idx[face_data["F"][2]]  # 'f-dfl' -> F[2]
    
    # B face
    state[12] = color_to_idx[face_data["B"][0]]  # 'b-ubr' -> B[0]
    state[13] = color_to_idx[face_data["B"][1]]  # 'b-ubl' -> B[1]
    state[14] = color_to_idx[face_data["B"][2]]  # 'b-dbr' -> B[2]
    state[15] = color_to_idx[face_data["B"][3]]  # 'b-dbl' -> B[3]
    
    # L face
    state[16] = color_to_idx[face_data["L"][1]]  # 'l-ufl' -> L[1]
    state[17] = color_to_idx[face_data["L"][0]]  # 'l-ubl' -> L[0]
    state[18] = color_to_idx[face_data["L"][3]]  # 'l-dfl' -> L[3]
    state[19] = color_to_idx[face_data["L"][2]]  # 'l-dbl' -> L[2]
    
    # R face
    state[20] = color_to_idx[face_data["R"][0]]  # 'r-ufr' -> R[0]
    state[21] = color_to_idx[face_data["R"][1]]  # 'r-ubr' -> R[1]
    state[22] = color_to_idx[face_data["R"][2]]  # 'r-dfr' -> R[2]
    state[23] = color_to_idx[face_data["R"][3]]  # 'r-dbr' -> R[3]

    return tuple(state)

def _bfs_worker(state, queue):
    try:
        queue.put({"solution": bfs_solve(state), "error": None})
    except Exception as e:
        queue.put({"solution": None, "error": str(e)})

def solve_with_timeout(state, timeout_seconds=SOLVER_TIMEOUT_SECONDS):
    """Run BFS in a child process so invalid/hard scans cannot hang the server."""
    ctx = mp.get_context("fork")
    queue = ctx.Queue()
    proc = ctx.Process(target=_bfs_worker, args=(state, queue))
    proc.start()
    proc.join(timeout_seconds)
    if proc.is_alive():
        proc.terminate()
        proc.join(1)
        return None, "timeout"
    if queue.empty():
        return None, "empty"
    result = queue.get()
    return result["solution"], result["error"]

class ScannerHTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            html_path = Path(__file__).resolve().parent / "index.html"
            try:
                with open(html_path, "r", encoding="utf-8") as f:
                    content = f.read()
                self.wfile.write(content.encode("utf-8"))
            except FileNotFoundError:
                self.wfile.write(b"Error: index.html not found.")
        elif self.path == "/calibrate.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            html_path = Path(__file__).resolve().parent / "calibrate.html"
            try:
                with open(html_path, "r", encoding="utf-8") as f:
                    content = f.read()
                self.wfile.write(content.encode("utf-8"))
            except FileNotFoundError:
                self.wfile.write(b"Error: calibrate.html not found.")
        elif self.path == "/cube-interp.js":
            self.send_response(200)
            self.send_header("Content-Type", "application/javascript; charset=utf-8")
            self.end_headers()
            js_path = Path(__file__).resolve().parent / "cube-interp.js"
            try:
                with open(js_path, "r", encoding="utf-8") as f:
                    content = f.read()
                self.wfile.write(content.encode("utf-8"))
            except FileNotFoundError:
                self.wfile.write(b"console.error('cube-interp.js not found');")
        elif self.path.startswith("/icons/"):
            from urllib.parse import unquote
            req_path = unquote(self.path.split("?", 1)[0])
            icon_path = Path(__file__).resolve().parent / req_path.lstrip("/")
            icons_dir = Path(__file__).resolve().parent / "icons"
            try:
                icon_path = icon_path.resolve()
                if not str(icon_path).startswith(str(icons_dir.resolve())):
                    self.send_error(403, "Forbidden")
                    return
                if not icon_path.is_file():
                    self.send_error(404, "Not Found")
                    return
                ext = icon_path.suffix.lower()
                ctype = {
                    ".jpg": "image/jpeg",
                    ".jpeg": "image/jpeg",
                    ".svg": "image/svg+xml",
                    ".png": "image/png",
                }.get(ext, "application/octet-stream")
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.end_headers()
                self.wfile.write(icon_path.read_bytes())
            except OSError:
                self.send_error(404, "Not Found")
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        if self.path == "/solve":
            content_length = int(self.headers["Content-Length"])
            post_data = self.rfile.read(content_length)
            
            try:
                face_data = json.loads(post_data.decode("utf-8"))
                
                # Check for completeness of the faces payload
                if not all(f in face_data for f in FACE_ORDER):
                    self.send_json_response({"status": "error", "message": "Missing faces in cube configuration."})
                    return
                
                state_tuple = build_solver_state(face_data)
                
                if not SOLVER_AVAILABLE or not state_tuple:
                    self.send_json_response({"status": "error", "message": "Solver is not available."})
                    return

                print("\n" + "-"*50)
                print("  Received Scrambled Cube Configuration")
                print("-"*50)
                print_cube(state_tuple)

                # Match robot/solver.py's fast path: generate a problem file for
                # inspection, then solve with the in-process bidirectional BFS.
                # The temp file avoids overwriting robot/problem.pddl.
                with tempfile.TemporaryDirectory(prefix="cub_scan_pddl_") as tmp:
                    print("Solving with robot.solver bidirectional BFS...")
                    t0 = time.perf_counter()
                    solution, solver_error = solve_with_timeout(state_tuple)
                    solve_seconds = time.perf_counter() - t0

                if solution is not None:
                    # Nivel 2: Fast Downward para calcular el plan de manipulación
                    print("Solving symbolic manipulation with Fast Downward...")
                    prob_file = workspace_root / "robot" / "manipulation_problem.pddl"
                    dom_file = workspace_root / "robot" / "manipulation_domain.pddl"
                    
                    t0_fd = time.perf_counter()
                    generate_manipulation_problem(solution, filename=str(prob_file))
                    manipulation_plan = run_fast_downward(domain_path=str(dom_file), problem_path=str(prob_file))
                    solve_seconds += (time.perf_counter() - t0_fd)
                    
                    if manipulation_plan is not None:
                        print(f"PDDL Plan found: {len(manipulation_plan)} physical moves in {solve_seconds:.3f}s total.")
                        self.send_json_response({
                            "status": "success",
                            "solution": manipulation_plan,
                            "cube_solution": solution,
                            "solver": "robot.solver.bfs_solve + Fast Downward",
                            "solve_seconds": round(solve_seconds, 3)
                        })
                    else:
                        self.send_json_response({
                            "status": "error",
                            "message": "Fast Downward failed to plan the physical transitions.",
                            "solver": "robot.solver.bfs_solve + Fast Downward",
                            "solve_seconds": round(solve_seconds, 3)
                        })
                else:
                    if solver_error == "timeout":
                        message = (
                            f"Solver timed out after {SOLVER_TIMEOUT_SECONDS}s. "
                            "The scan may be unreachable for the robot model; please verify the 2D cross map."
                        )
                    elif solver_error:
                        message = f"Solver error: {solver_error}"
                    else:
                        message = "Invalid cube scan or unsupported face mapping. Please verify the 2D cross map and face orientation."
                    self.send_json_response({
                        "status": "error",
                        "message": message,
                        "solver": "robot.solver.bfs_solve",
                        "solve_seconds": round(solve_seconds, 3)
                    })

            except Exception as e:
                self.send_json_response({"status": "error", "message": f"Server error: {str(e)}"})
        elif self.path == "/calibrate-log":
            content_length = int(self.headers["Content-Length"])
            post_data = self.rfile.read(content_length)
            try:
                payload = json.loads(post_data.decode("utf-8"))
                log_dir = Path(__file__).resolve().parent / "calibrate_logs"
                log_dir.mkdir(exist_ok=True)
                from datetime import datetime
                fname = f"calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                log_path = log_dir / fname
                with open(log_path, "w", encoding="utf-8") as f:
                    json.dump(payload, f, indent=2, ensure_ascii=False)
                print(f"\n[calibrate] Saved: {log_path}")
                self.send_json_response({"status": "success", "path": str(log_path.relative_to(workspace_root))})
            except Exception as e:
                self.send_json_response({"status": "error", "message": str(e)})
        else:
            self.send_error(404, "Not Found")

    def send_json_response(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode("utf-8"))

    def log_message(self, format, *args):
        # Mute standard access logging for clean output
        pass

def main():
    print("\n" + "-"*60)
    print("  UR3 ROBOT - 2x2 CUBE SCANNER WEB SERVER")
    print("-"*60)
    print(f"  Starting local server on port {PORT}...")
    print("  WSL2 will automatically forward port 5000 to Windows.")
    print("\n  Open your Windows web browser and go to:")
    print(f"  http://localhost:{PORT}")
    print("-"*60 + "\n")

    handler = ScannerHTTPHandler
    socketserver.TCPServer.allow_reuse_address = True
    
    with socketserver.TCPServer(("", PORT), handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer terminated by user.")

if __name__ == "__main__":
    main()