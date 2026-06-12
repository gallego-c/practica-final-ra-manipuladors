#!/usr/bin/env python3

import sys
import math


def normalize_joint(angle_deg, joint_idx):
    """
    joint_idx: 1..6

    Joints 1,2,4,5,6:
        q_rad in [-2π, 2π]
        q_norm = (q_rad + 2π)/(4π)

    Joint 3 (elbow):
        q_rad in [-π, π]
        q_norm = (q_rad + π)/(2π)
    """

    q_rad = math.radians(angle_deg)

    if joint_idx == 3:
        # Wrap to [-π, π]
        q_rad = math.atan2(math.sin(q_rad), math.cos(q_rad))
        q_norm = (q_rad + math.pi) / (2 * math.pi)
    else:
        # Wrap to [-2π, 2π]
        q_rad = ((q_rad + 2 * math.pi) % (4 * math.pi)) - 2 * math.pi
        q_norm = (q_rad + 2 * math.pi) / (4 * math.pi)

    return q_rad, q_norm


def main():
    if len(sys.argv) != 7:
        print(
            f"Usage: {sys.argv[0]} angle1 angle2 angle3 angle4 angle5 angle6",
            file=sys.stderr,
        )
        sys.exit(1)

    angles_deg = [float(x) for x in sys.argv[1:]]

    print("Joint | Degrees | Radians | Normalized")
    print("----------------------------------------")

    for i, angle_deg in enumerate(angles_deg, start=1):
        q_rad, q_norm = normalize_joint(angle_deg, i)
        print(f"{i:5d} | {angle_deg:7.3f} | {q_rad:8.6f} | {q_norm:.6f}")


if __name__ == "__main__":
    main()
