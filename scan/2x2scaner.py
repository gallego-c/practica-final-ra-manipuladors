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
from pathlib import Path

# Add workspace root to system path for imports
workspace_root = Path(__file__).resolve().parent.parent
if str(workspace_root) not in sys.path:
    sys.path.insert(0, str(workspace_root))

try:
    from robot.solver import COLOR_IDX, generate_pddl_problem, bfs_solve, print_cube
    SOLVER_AVAILABLE = True
except ImportError:
    SOLVER_AVAILABLE = False
    print("Warning: robot.solver module is not available. Solving locally is disabled.")

PORT = 5000
FACE_ORDER = ["U", "F", "R", "B", "L", "D"]

def build_solver_state(face_data):
    """Maps the 6 scanned faces (4 facelets each) to the 24-sticker solver state."""
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

                # Write problem.pddl for the solver
                pddl_path = workspace_root / "robot" / "problem.pddl"
                generate_pddl_problem(state_tuple, filename=str(pddl_path))

                # Solve cube optimally using BFS
                print("Solving cube optimally with BFS...")
                solution = bfs_solve(state_tuple)

                if solution is not None:
                    print(f"Solution found: {len(solution)} moves.")
                    self.send_json_response({
                        "status": "success",
                        "solution": solution,
                        "pddl_path": str(pddl_path.relative_to(workspace_root))
                    })
                else:
                    self.send_json_response({"status": "error", "message": "Invalid color mix. Please verify 2D cross map."})

            except Exception as e:
                self.send_json_response({"status": "error", "message": f"Server error: {str(e)}"})
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