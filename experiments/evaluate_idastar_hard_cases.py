#!/usr/bin/env python3
"""
Evaluate the bidirectional BFS solver on fixed hard 2x2 cases.

This script intentionally does not generate a full state tree. The hard cases
come from public God's-number references for the 2x2:
  - HTM antipode: R U' R2 U' F' R' U F2 R U' F
  - QTM antipode: U R U F' R' F R U F' R F U R' F

The project solver uses quarter turns only, so half turns are expanded as two
quarter turns. Results are written to experiments/idastar_hard_cases.csv.
"""

from __future__ import annotations

import argparse
import csv
import multiprocessing as mp
import os
import sys
import time
from dataclasses import dataclass


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ROBOT_DIR = os.path.join(REPO_ROOT, "robot")
CSV_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "idastar_hard_cases.csv")

sys.path.insert(0, ROBOT_DIR)

from solver import (  # noqa: E402
    ROBOT_MOVES,
    apply_move,
    bfs_solve,
    has_valid_color_counts,
    is_reachable_state,
    is_solved_monochromatic,
    scramble,
)


MOVE_ALIASES = {
    "U": ["U"],
    "U'": ["U_PRIME"],
    "U2": ["U", "U"],
    "D": ["D"],
    "D'": ["D_PRIME"],
    "D2": ["D", "D"],
    "R": ["R"],
    "R'": ["R_PRIME"],
    "R2": ["R", "R"],
    "L": ["L"],
    "L'": ["L_PRIME"],
    "L2": ["L", "L"],
    "F": ["F"],
    "F'": ["F_PRIME"],
    "F2": ["F", "F"],
    "B": ["B"],
    "B'": ["B_PRIME"],
    "B2": ["B", "B"],
}


KNOWN_HARD_SEQUENCES = {
    "gods_number_htm_11_expanded_qtm": "R U' R2 U' F' R' U F2 R U' F",
    "gods_number_qtm_14": "U R U F' R' F R U F' R F U R' F",
}


SCANNED_TERMINAL_STATE = (
    # Reconstructed from the terminal cross:
    #          OR / GR
    # WR WG YY BG / BR WO GW OY
    #          BY / OB
    2, 5, 2, 3,
    1, 4, 4, 3,
    5, 0, 3, 0,
    4, 5, 3, 1,
    2, 0, 2, 4,
    1, 1, 5, 0,
)


@dataclass(frozen=True)
class Case:
    name: str
    source: str
    state: tuple[int, ...]
    scramble: str


def parse_sequence(sequence: str) -> list[str]:
    moves: list[str] = []
    for token in sequence.split():
        if token not in MOVE_ALIASES:
            raise ValueError(f"Unsupported move token: {token}")
        moves.extend(MOVE_ALIASES[token])
    return moves


def invert_moves(moves: list[str]) -> list[str]:
    inverse = {
        "U": "U_PRIME",
        "U_PRIME": "U",
        "D": "D_PRIME",
        "D_PRIME": "D",
        "R": "R_PRIME",
        "R_PRIME": "R",
        "L": "L_PRIME",
        "L_PRIME": "L",
        "F": "F_PRIME",
        "F_PRIME": "F",
        "B": "B_PRIME",
        "B_PRIME": "B",
    }
    return [inverse[move] for move in reversed(moves)]


def build_cases(include_scanned: bool) -> list[Case]:
    cases = []
    for name, sequence in KNOWN_HARD_SEQUENCES.items():
        moves = parse_sequence(sequence)
        cases.append(Case(name, "known_public_hard_sequence", scramble(moves), " ".join(moves)))

        inverse_moves = invert_moves(moves)
        cases.append(Case(
            f"{name}_inverse",
            "known_public_hard_sequence_inverse",
            scramble(inverse_moves),
            " ".join(inverse_moves),
        ))

    if include_scanned:
        cases.append(Case("terminal_scanned_state", "scan_reconstruction", SCANNED_TERMINAL_STATE, ""))
    return cases


def solve_worker(state, queue):
    try:
        queue.put(("ok", bfs_solve(state), None))
    except Exception as exc:  # pragma: no cover - diagnostic path
        queue.put(("error", None, repr(exc)))


def solve_with_timeout(state: tuple[int, ...], timeout_seconds: float):
    queue = mp.Queue()
    proc = mp.Process(target=solve_worker, args=(state, queue))
    started = time.perf_counter()
    proc.start()
    proc.join(timeout_seconds)
    elapsed = time.perf_counter() - started

    if proc.is_alive():
        proc.terminate()
        proc.join()
        return "solver_timeout", None, elapsed, None
    if queue.empty():
        return "solver_error", None, elapsed, "solver exited without result"

    status, solution, error = queue.get()
    if status == "error":
        return "solver_error", None, elapsed, error
    if solution is None:
        return "solver_none", None, elapsed, None
    return "solver_ok", solution, elapsed, None


def verify_solution(state: tuple[int, ...], solution: list[str] | None) -> bool:
    if solution is None:
        return False
    current = state
    for move in solution:
        current = apply_move(current, ROBOT_MOVES[move])
    return is_solved_monochromatic(current)


def run_fast_downward_if_requested(solution: list[str], enabled: bool):
    if not enabled:
        return "skipped", 0, 0.0

    from generate_taskfile import generate_manipulation_problem, run_fast_downward

    problem_path = os.path.join(ROBOT_DIR, "manipulation_problem.pddl")
    domain_path = os.path.join(ROBOT_DIR, "manipulation_domain.pddl")
    generate_manipulation_problem(solution, filename=problem_path)

    started = time.perf_counter()
    plan = run_fast_downward(domain_path=domain_path, problem_path=problem_path)
    elapsed = time.perf_counter() - started
    if plan is None:
        return "fd_failed", 0, elapsed
    return "fd_ok", len(plan), elapsed


def write_rows(rows):
    headers = [
        "case",
        "source",
        "scramble_qtm",
        "valid_color_counts",
        "reachable",
        "solver_status",
        "solver_time_s",
        "solver_solution_len",
        "solver_solution",
        "solver_verified",
        "fd_status",
        "fd_time_s",
        "fd_plan_len",
        "error",
    ]
    with open(CSV_OUTPUT_PATH, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--skip-scanned", action="store_true")
    parser.add_argument("--run-fast-downward", action="store_true")
    args = parser.parse_args()

    print("=" * 72)
    print("  BIDIRECTIONAL BFS HARD-CASE EVALUATION FOR 2x2")
    print("=" * 72)

    cases = build_cases(include_scanned=not args.skip_scanned)
    rows = []
    for idx, case in enumerate(cases, start=1):
        valid_counts = has_valid_color_counts(case.state)
        reachable = is_reachable_state(case.state)
        if not reachable:
            solver_status, solution, ida_time, error = "invalid_unreachable", None, 0.0, "not reachable"
        else:
            solver_status, solution, ida_time, error = solve_with_timeout(case.state, args.timeout)

        verified = verify_solution(case.state, solution)
        fd_status, fd_plan_len, fd_time = ("skipped", 0, 0.0)
        if solver_status == "solver_ok" and verified:
            fd_status, fd_plan_len, fd_time = run_fast_downward_if_requested(
                solution,
                args.run_fast_downward,
            )

        rows.append({
            "case": case.name,
            "source": case.source,
            "scramble_qtm": case.scramble,
            "valid_color_counts": valid_counts,
            "reachable": reachable,
            "solver_status": solver_status,
            "solver_time_s": f"{ida_time:.4f}",
            "solver_solution_len": len(solution) if solution is not None else "",
            "solver_solution": " ".join(solution or []),
            "solver_verified": verified,
            "fd_status": fd_status,
            "fd_time_s": f"{fd_time:.4f}",
            "fd_plan_len": fd_plan_len,
            "error": error or "",
        })
        print(
            f"{idx:2d}. {case.name}: reachable={reachable} "
            f"status={solver_status} time={ida_time:.4f}s "
            f"len={len(solution) if solution else '-'} fd={fd_status}"
        )

    write_rows(rows)
    failures = [row for row in rows if row["solver_status"] != "solver_ok"]
    print("\n" + "=" * 72)
    print(f"CSV written to: {CSV_OUTPUT_PATH}")
    print(f"Solver failures/timeouts: {len(failures)}/{len(rows)}")
    print("=" * 72)


if __name__ == "__main__":
    main()
