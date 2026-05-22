# UR3 Robot Arm 2×2 Rubik's Cube Solver (PDDL + BFS & Kautham Simulation)

Welcome to the **UR3 Robot Arm 2×2 Rubik's Cube Solver** repository. This project provides a complete, elegant PDDL domain and an optimal Python BFS solver modeling a **Universal Robots UR3** arm solving a 2×2 Rubik's cube under realistic physical constraints. It also includes configuration files to visualize, plan, and verify the physical movements in **Kautham**, a powerful motion planning simulation tool.

---

## 📌 1. Physical Setup & Manipulation Constraints

Standard Rubik's cube solvers assume arbitrary face turns ($U, D, R, L, F, B$). However, when a robotic manipulator solves a cube, it is bound by physical and kinematical constraints:
1. **Vertical Grasp Constraint**: The UR3 robot arm is mounted above the table and can only grasp the cube **vertically from above** (top-down grasp) using its **Robotiq 85 gripper**.
2. **Base Fixture**: The cube rests on a secure, form-fitting **square base fixture** on the table, preventing the bottom layers from turning when placed.
3. **Layer Turns ($U$ moves)**: When the robot is holding the cube, spinning the wrist/end-effector rotates only the **top layer** (the $U$ face) while the bottom layer remains static.
4. **Whole-Cube Tilts (Reorientation)**: To rotate other faces or make them accessible to the top layer, the robot must lift the cube, tilt/roll it 90° around the horizontal $X$ or $Y$ axis (effectively changing which face is on top), and place it back into the base fixture.

This physical constraints system restricts the robot to exactly **6 atomic manipulation actions**:
* 🔄 **`rotate_top_cw`** / **`rotate_top_ccw`**: Spin the wrist while holding the cube, turning only the top layer.
* ↕️ **`tilt_x_pos`** / **`tilt_x_neg`**: Pick the cube, roll/tip it forward or backward ($90^\circ$ around the $X$-axis), and place it back on the fixture.
* ↔️ **`tilt_y_pos`** / **`tilt_y_neg`**: Pick the cube, roll/tip it rightward or leftward ($90^\circ$ around the $Y$-axis), and place it back on the fixture.

---

## 📂 2. Repository Structure

This repository is designed to be clean, self-contained, and highly structured:

```
robotica/cub/
├── README.md                          # Comprehensive project documentation
├── robot/
│   ├── domain.pddl                    # PDDL domain representing physical UR3 cube actions
│   ├── problem.pddl                   # Dynamically generated PDDL problem instance
│   └── solver.py                      # Optimal bidirectional BFS solver & PDDL generator
└── kautham/
    ├── ur3_rubik_kautham.xml          # Kautham XML problem scene loading UR3, table, and models
    ├── rubik_cube_2x2.urdf            # Native 3D model for the 2x2 Rubik's cube obstacle
    └── square_fixture.urdf            # Native 3D model for the table square base fixture
```

---

## 🛠️ 3. PDDL Modeling (`robot/domain.pddl`)

The PDDL domain defines the state representation and actions mapping directly to physical robot trajectories.

### State Representation
* **Propositions**:
  * `(color-at ?p - position ?c - color)`: Specifies which color sticker is on a particular facelet position.
  * `(robot-holding)`: `True` if the UR3 gripper is holding the cube.
  * `(cube-on-fixture)`: `True` if the cube is securely placed in the table base fixture.
* **Positions**: 24 facelet positions labeled `<face>-<corner>` (e.g., `u-ufr` for the Up sticker of the Up-Front-Right corner).

### Robot Actions
* **`pick`**: Lifts the cube from the fixture.
  * *Precondition*: `(cube-on-fixture)` and `(not (robot-holding))`
  * *Effect*: `(robot-holding)` and `(not (cube-on-fixture))`
* **`place`**: Releases the cube onto the fixture.
  * *Precondition*: `(robot-holding)`
  * *Effect*: `(cube-on-fixture)` and `(not (robot-holding))`
* **`rotate_top_cw` / `rotate_top_ccw`**: Rotates the wrist clockwise/counter-clockwise.
  * *Precondition*: `(robot-holding)`
  * *Effect*: Permutes the stickers of the top layer using conditional effects.
* **`tilt_*`**: Combined macro-actions representing picking, reorienting in the air, and placing the cube back.
  * *Precondition*: `(cube-on-fixture)` and `(not (robot-holding))`
  * *Effect*: Full 3D coordinate frame transformation of all 24 stickers (conditional effects).

---

## 🚀 4. Running the Optimal Python Solver

The script `solver.py` contains a high-performance **bidirectional Breadth-First Search (BFS)** solver. It finds the **absolute minimum number of robot actions** to solve the cube from any starting scramble, and automatically outputs the corresponding PDDL `problem.pddl` file.

### How to Run:
You can run the solver with the default scramble or supply your own custom scramble sequence using the terminal:

1. **Run with the Default Scramble**:
   ```bash
   python3 robot/solver.py
   ```
2. **Run with a Custom Scramble Sequence**:
   ```bash
   python3 robot/solver.py tilt_x_pos rotate_top_cw tilt_y_pos rotate_top_ccw
   ```

### Solver Output:
The solver prints a pretty cross-layout visualization of the cube state, outputs the optimal solution, and generates a valid PDDL problem file:
```
============================================================
  UR3 Robot — 2×2 Rubik's Cube Solver
============================================================

Scramble sequence (4 moves):
  1. tilt_x_pos
  2. rotate_top_cw
  3. tilt_y_pos
  4. rotate_top_ccw
✓ PDDL problem file written to robot/problem.pddl

── Scrambled Cube ──
         ┌────┐
         │WW  │
         │GG  │
┌────┬───┴┬───┴┬────┐
│WB  │RR  │YG  │OO  │
│RR  │YG  │OO  │WB  │
└────┴───┬┴───┬┴────┘
         │YB  │
         │YB  │
         └────┘

Solving with bidirectional BFS...

✓ OPTIMAL SOLUTION  (4 robot actions)  [0.001s]

  Step  1: rotate_top_cw
  Step  2: tilt_y_neg
  Step  3: rotate_top_ccw
  Step  4: tilt_x_neg

✓ Verification passed — cube is solved.
```

---

## 🤖 5. Kautham GUI Simulation & Verification

**Kautham** is a motion planning tool that allows you to load robots and obstacles, define queries, and compute obstacle-free trajectories using OMPL algorithms (like RRT-Connect).

We have provided a custom Kautham scene XML file: `kautham/ur3_rubik_kautham.xml`.

### Step-by-Step Instructions to Verify in Kautham GUI:

1. **Launch the Kautham GUI**:
   Open a terminal and run the Kautham GUI application installed on your environment:
   ```bash
   kautham-gui
   ```

2. **Load the Problem Scene**:
   * Click on **`File`** $\rightarrow$ **`Open Problem`** in the top menu bar.
   * Navigate to your workspace directory and open:
     `kautham/ur3_rubik_kautham.xml`
   * This will load the beautiful 3D workspace containing:
     * A **UR3 manipulator** equipped with a **Robotiq 85 gripper**.
     * A table base representing the workspace surface.
     * A **square fixture** (the block holding the cube, modeled from `square_fixture.urdf`).
     * The **2×2 Rubik's cube** (modeled from `rubik_cube_2x2.urdf`) placed precisely on the fixture.

### 📦 Native 2×2 Rubik's Cube & Fixture URDF Modeling

The 2×2 Rubik's cube and its base holder are modeled locally using native, parameter-based URDF box primitives rather than scaled generic files:
1. **`rubik_cube_2x2.urdf`**: Models the 2×2 Rubik's cube as a single rigid obstacle (without internal joints) of exactly **$5.0 \text{ cm} \times 5.0 \text{ cm} \times 5.0 \text{ cm}$** ($0.05 \text{ m}$ side length). It is colored matte carbon black.
2. **`square_fixture.urdf`**: Models the table-mounted base fixture that securely holds the cube. Dimensions are exactly **$7.0 \text{ cm} \times 7.0 \text{ cm} \times 2.0 \text{ cm}$** ($0.07 \text{ m} \times 0.07 \text{ m} \times 0.02 \text{ m}$), colored metallic gray.

By avoiding complex multi-joint models inside Kautham for the cube itself, the planner focuses entirely on the UR3 robot arm's collision-free kinematics to pick, place, and reorient the rigid block, perfectly matching the PDDL state abstractions.

3. **Verify the Degrees of Freedom (DoFs)**:
   * Select the **`Controls`** or **`Robot`** tab on the left/right panel.
   * You will see the sliders representing the 6 joints of the UR3 robot arm (`shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`) and the `gripper` control.
   * Drag the sliders to see the UR3 arm move in 3D and see the gripper fingers open and close.

4. **Verify Motion Planning (Queries)**:
   * To verify that the robot can successfully reach, grasp, and move the cube without colliding with the environment or itself, go to the **`Planner`** tab.
   * Under the **`Queries`** list, select the default query (which defines the motion from home above the cube to the grasp position).
   * Choose the planner **`omplRRTConnect`** from the planner dropdown.
   * Click **`Plan`** / **`Solve`**. You will see green planning trees branch out in C-space as the RRT-Connect algorithm finds a collision-free path.
   * Once solved, click **`Play`** or **`Animate`** to watch the UR3 robot smoothly lower its gripper, align perfectly with the top face of the Rubik's cube, and simulate a physical action!

Using this setup, each step returned by the PDDL planner or the BFS Python solver (such as `pick`, `rotate_top_cw`, `tilt_x_pos`, `place`) can be mapped directly to a collision-free joint trajectory in Kautham!

---
Developed for Robotic Manipulation and Planning with PDDL & Kautham. Enjoy solving! 🚀
