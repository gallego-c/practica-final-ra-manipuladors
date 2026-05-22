;;; ============================================================
;;; DOMAIN: rubik-robot
;;; ============================================================
;;;
;;; Models a UR3 robot arm solving a 2x2 Rubik's cube placed on
;;; a square fixture/base in front of the robot.
;;;
;;; Physical setup:
;;;   - The cube sits on a fixed square base (fixture) on a table.
;;;   - The robot grasps the cube vertically from ABOVE (top-down grasp).
;;;   - While holding the cube, the end-effector can:
;;;       (a) Rotate the wrist → rotates the WHOLE cube (tilt in X or Y).
;;;       (b) Rotate only the TOP LAYER → U-face rotation.
;;;   - After any tilt, the robot places the cube back on the fixture.
;;;
;;; Available robot actions → PDDL cube moves:
;;;
;;;   pick            – lift cube off fixture (robot grabs from top)
;;;   place           – set cube back on fixture
;;;   rotate_top_cw   – spin top layer CW  (= U  move, while holding)
;;;   rotate_top_ccw  – spin top layer CCW (= U' move, while holding)
;;;   rotate_top_180  – spin top layer 180 (= U2 move, while holding)
;;;   tilt_x_pos      – tilt whole cube +90° around X-axis (F→top)
;;;   tilt_x_neg      – tilt whole cube -90° around X-axis (B→top)
;;;   tilt_y_pos      – tilt whole cube +90° around Y-axis (L→top)
;;;   tilt_y_neg      – tilt whole cube -90° around Y-axis (R→top)
;;;
;;; State representation:
;;;   24 sticker positions (6 faces × 4 stickers each).
;;;   Position naming: <face>-<corner>
;;;     Faces: u(p), d(own), f(ront), b(ack), l(eft), r(ight)
;;;     Corners: ufr, ufl, ubr, ubl, dfr, dfl, dbr, dbl
;;;   Predicate: (color-at ?pos - position ?col - color)
;;;
;;; Coordinate system (looking from above):
;;;   +X = toward front (F face direction)
;;;   +Y = toward right (R face direction)
;;;   +Z = up
;;;
;;; ============================================================

(define (domain rubik-robot)

  (:requirements :strips :typing :conditional-effects)

  (:types
    color
    position
  )

  (:predicates
    (color-at ?p - position ?c - color)  ; sticker at position has color
    (robot-holding)                       ; robot is currently holding the cube
    (cube-on-fixture)                     ; cube is resting on the base fixture
  )

  ;; -----------------------------------------------------------
  ;; PICK – lift cube off fixture
  ;; Precondition: cube is on the fixture, robot is not holding
  ;; Effect:       robot holds cube, cube leaves fixture
  ;; -----------------------------------------------------------
  (:action pick
    :parameters ()
    :precondition (and
      (cube-on-fixture)
      (not (robot-holding))
    )
    :effect (and
      (robot-holding)
      (not (cube-on-fixture))
    )
  )

  ;; -----------------------------------------------------------
  ;; PLACE – set cube back on fixture
  ;; Precondition: robot is holding cube
  ;; Effect:       cube on fixture, robot releases
  ;; -----------------------------------------------------------
  (:action place
    :parameters ()
    :precondition (and
      (robot-holding)
    )
    :effect (and
      (cube-on-fixture)
      (not (robot-holding))
    )
  )

  ;; ===========================================================
  ;; TOP-LAYER ROTATIONS  (robot holds cube and spins wrist)
  ;; These correspond to standard U, U', U2 moves.
  ;; ===========================================================

  ;; -----------------------------------------------------------
  ;; ROTATE_TOP_CW  (= U move: top layer clockwise viewed from above)
  ;; Corner cycle on U face:  UBL → UBR → UFR → UFL → UBL
  ;; Side sticker cycle 1:    b-ubl → r-ubr → f-ufr → l-ufl → b-ubl
  ;; Side sticker cycle 2:    l-ubl → b-ubr → r-ufr → f-ufl → l-ubl
  ;; -----------------------------------------------------------
  (:action rotate_top_cw
    :parameters ()
    :precondition (and (robot-holding))
    :effect (and
      ;; U face stickers cycle: UBL→UBR→UFR→UFL→UBL
      (when (color-at u-ubl white)   (and (color-at u-ubr white)   (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow)  (and (color-at u-ubr yellow)  (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)     (and (color-at u-ubr red)     (not (color-at u-ubl red))))
      (when (color-at u-ubl orange)  (and (color-at u-ubr orange)  (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)    (and (color-at u-ubr blue)    (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)   (and (color-at u-ubr green)   (not (color-at u-ubl green))))

      (when (color-at u-ubr white)   (and (color-at u-ufr white)   (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow)  (and (color-at u-ufr yellow)  (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)     (and (color-at u-ufr red)     (not (color-at u-ubr red))))
      (when (color-at u-ubr orange)  (and (color-at u-ufr orange)  (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)    (and (color-at u-ufr blue)    (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)   (and (color-at u-ufr green)   (not (color-at u-ubr green))))

      (when (color-at u-ufr white)   (and (color-at u-ufl white)   (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow)  (and (color-at u-ufl yellow)  (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)     (and (color-at u-ufl red)     (not (color-at u-ufr red))))
      (when (color-at u-ufr orange)  (and (color-at u-ufl orange)  (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)    (and (color-at u-ufl blue)    (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)   (and (color-at u-ufl green)   (not (color-at u-ufr green))))

      (when (color-at u-ufl white)   (and (color-at u-ubl white)   (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow)  (and (color-at u-ubl yellow)  (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)     (and (color-at u-ubl red)     (not (color-at u-ufl red))))
      (when (color-at u-ufl orange)  (and (color-at u-ubl orange)  (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)    (and (color-at u-ubl blue)    (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)   (and (color-at u-ubl green)   (not (color-at u-ufl green))))

      ;; Side sticker cycle 1:  b-ubl → r-ubr → f-ufr → l-ufl → b-ubl
      (when (color-at b-ubl white)   (and (color-at r-ubr white)   (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow)  (and (color-at r-ubr yellow)  (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)     (and (color-at r-ubr red)     (not (color-at b-ubl red))))
      (when (color-at b-ubl orange)  (and (color-at r-ubr orange)  (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)    (and (color-at r-ubr blue)    (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)   (and (color-at r-ubr green)   (not (color-at b-ubl green))))

      (when (color-at r-ubr white)   (and (color-at f-ufr white)   (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow)  (and (color-at f-ufr yellow)  (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)     (and (color-at f-ufr red)     (not (color-at r-ubr red))))
      (when (color-at r-ubr orange)  (and (color-at f-ufr orange)  (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)    (and (color-at f-ufr blue)    (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)   (and (color-at f-ufr green)   (not (color-at r-ubr green))))

      (when (color-at f-ufr white)   (and (color-at l-ufl white)   (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow)  (and (color-at l-ufl yellow)  (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)     (and (color-at l-ufl red)     (not (color-at f-ufr red))))
      (when (color-at f-ufr orange)  (and (color-at l-ufl orange)  (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)    (and (color-at l-ufl blue)    (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)   (and (color-at l-ufl green)   (not (color-at f-ufr green))))

      (when (color-at l-ufl white)   (and (color-at b-ubl white)   (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow)  (and (color-at b-ubl yellow)  (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)     (and (color-at b-ubl red)     (not (color-at l-ufl red))))
      (when (color-at l-ufl orange)  (and (color-at b-ubl orange)  (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)    (and (color-at b-ubl blue)    (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)   (and (color-at b-ubl green)   (not (color-at l-ufl green))))

      ;; Side sticker cycle 2:  l-ubl → b-ubr → r-ufr → f-ufl → l-ubl
      (when (color-at l-ubl white)   (and (color-at b-ubr white)   (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow)  (and (color-at b-ubr yellow)  (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)     (and (color-at b-ubr red)     (not (color-at l-ubl red))))
      (when (color-at l-ubl orange)  (and (color-at b-ubr orange)  (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)    (and (color-at b-ubr blue)    (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)   (and (color-at b-ubr green)   (not (color-at l-ubl green))))

      (when (color-at b-ubr white)   (and (color-at r-ufr white)   (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow)  (and (color-at r-ufr yellow)  (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)     (and (color-at r-ufr red)     (not (color-at b-ubr red))))
      (when (color-at b-ubr orange)  (and (color-at r-ufr orange)  (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)    (and (color-at r-ufr blue)    (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)   (and (color-at r-ufr green)   (not (color-at b-ubr green))))

      (when (color-at r-ufr white)   (and (color-at f-ufl white)   (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow)  (and (color-at f-ufl yellow)  (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)     (and (color-at f-ufl red)     (not (color-at r-ufr red))))
      (when (color-at r-ufr orange)  (and (color-at f-ufl orange)  (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)    (and (color-at f-ufl blue)    (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)   (and (color-at f-ufl green)   (not (color-at r-ufr green))))

      (when (color-at f-ufl white)   (and (color-at l-ubl white)   (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow)  (and (color-at l-ubl yellow)  (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)     (and (color-at l-ubl red)     (not (color-at f-ufl red))))
      (when (color-at f-ufl orange)  (and (color-at l-ubl orange)  (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)    (and (color-at l-ubl blue)    (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)   (and (color-at l-ubl green)   (not (color-at f-ufl green))))
    )
  )

  ;; -----------------------------------------------------------
  ;; ROTATE_TOP_CCW  (= U' move: top layer counter-clockwise from above)
  ;; Reverse of rotate_top_cw
  ;; -----------------------------------------------------------
  (:action rotate_top_ccw
    :parameters ()
    :precondition (and (robot-holding))
    :effect (and
      ;; U face stickers cycle reversed: UFL→UFR→UBR→UBL→UFL
      (when (color-at u-ufl white)   (and (color-at u-ufr white)   (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow)  (and (color-at u-ufr yellow)  (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)     (and (color-at u-ufr red)     (not (color-at u-ufl red))))
      (when (color-at u-ufl orange)  (and (color-at u-ufr orange)  (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)    (and (color-at u-ufr blue)    (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)   (and (color-at u-ufr green)   (not (color-at u-ufl green))))

      (when (color-at u-ufr white)   (and (color-at u-ubr white)   (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow)  (and (color-at u-ubr yellow)  (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)     (and (color-at u-ubr red)     (not (color-at u-ufr red))))
      (when (color-at u-ufr orange)  (and (color-at u-ubr orange)  (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)    (and (color-at u-ubr blue)    (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)   (and (color-at u-ubr green)   (not (color-at u-ufr green))))

      (when (color-at u-ubr white)   (and (color-at u-ubl white)   (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow)  (and (color-at u-ubl yellow)  (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)     (and (color-at u-ubl red)     (not (color-at u-ubr red))))
      (when (color-at u-ubr orange)  (and (color-at u-ubl orange)  (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)    (and (color-at u-ubl blue)    (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)   (and (color-at u-ubl green)   (not (color-at u-ubr green))))

      (when (color-at u-ubl white)   (and (color-at u-ufl white)   (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow)  (and (color-at u-ufl yellow)  (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)     (and (color-at u-ufl red)     (not (color-at u-ubl red))))
      (when (color-at u-ubl orange)  (and (color-at u-ufl orange)  (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)    (and (color-at u-ufl blue)    (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)   (and (color-at u-ufl green)   (not (color-at u-ubl green))))

      ;; Side sticker cycle 1 reversed:  l-ufl → f-ufr → r-ubr → b-ubl → l-ufl
      (when (color-at l-ufl white)   (and (color-at f-ufr white)   (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow)  (and (color-at f-ufr yellow)  (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)     (and (color-at f-ufr red)     (not (color-at l-ufl red))))
      (when (color-at l-ufl orange)  (and (color-at f-ufr orange)  (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)    (and (color-at f-ufr blue)    (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)   (and (color-at f-ufr green)   (not (color-at l-ufl green))))

      (when (color-at f-ufr white)   (and (color-at r-ubr white)   (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow)  (and (color-at r-ubr yellow)  (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)     (and (color-at r-ubr red)     (not (color-at f-ufr red))))
      (when (color-at f-ufr orange)  (and (color-at r-ubr orange)  (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)    (and (color-at r-ubr blue)    (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)   (and (color-at r-ubr green)   (not (color-at f-ufr green))))

      (when (color-at r-ubr white)   (and (color-at b-ubl white)   (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow)  (and (color-at b-ubl yellow)  (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)     (and (color-at b-ubl red)     (not (color-at r-ubr red))))
      (when (color-at r-ubr orange)  (and (color-at b-ubl orange)  (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)    (and (color-at b-ubl blue)    (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)   (and (color-at b-ubl green)   (not (color-at r-ubr green))))

      (when (color-at b-ubl white)   (and (color-at l-ufl white)   (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow)  (and (color-at l-ufl yellow)  (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)     (and (color-at l-ufl red)     (not (color-at b-ubl red))))
      (when (color-at b-ubl orange)  (and (color-at l-ufl orange)  (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)    (and (color-at l-ufl blue)    (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)   (and (color-at l-ufl green)   (not (color-at b-ubl green))))

      ;; Side sticker cycle 2 reversed:  f-ufl → r-ufr → b-ubr → l-ubl → f-ufl
      (when (color-at f-ufl white)   (and (color-at r-ufr white)   (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow)  (and (color-at r-ufr yellow)  (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)     (and (color-at r-ufr red)     (not (color-at f-ufl red))))
      (when (color-at f-ufl orange)  (and (color-at r-ufr orange)  (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)    (and (color-at r-ufr blue)    (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)   (and (color-at r-ufr green)   (not (color-at f-ufl green))))

      (when (color-at r-ufr white)   (and (color-at b-ubr white)   (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow)  (and (color-at b-ubr yellow)  (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)     (and (color-at b-ubr red)     (not (color-at r-ufr red))))
      (when (color-at r-ufr orange)  (and (color-at b-ubr orange)  (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)    (and (color-at b-ubr blue)    (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)   (and (color-at b-ubr green)   (not (color-at r-ufr green))))

      (when (color-at b-ubr white)   (and (color-at l-ubl white)   (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow)  (and (color-at l-ubl yellow)  (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)     (and (color-at l-ubl red)     (not (color-at b-ubr red))))
      (when (color-at b-ubr orange)  (and (color-at l-ubl orange)  (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)    (and (color-at l-ubl blue)    (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)   (and (color-at l-ubl green)   (not (color-at b-ubr green))))

      (when (color-at l-ubl white)   (and (color-at f-ufl white)   (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow)  (and (color-at f-ufl yellow)  (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)     (and (color-at f-ufl red)     (not (color-at l-ubl red))))
      (when (color-at l-ubl orange)  (and (color-at f-ufl orange)  (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)    (and (color-at f-ufl blue)    (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)   (and (color-at f-ufl green)   (not (color-at l-ubl green))))
    )
  )

  ;; ===========================================================
  ;; WHOLE-CUBE TILTS  (robot lifts cube and repositions it)
  ;; The cube must be ON the fixture – robot picks it, tilts it,
  ;; then places it back. Modeled as a single combined action.
  ;;
  ;; Tilt around X-axis (+90°): Front face goes UP
  ;;   U → B,  F → U,  D → F,  B → D  (L and R unchanged)
  ;;   Sticker mapping per corner:
  ;;     UFR→BUR, UFL→BUL, UBR→DBR, UBL→DBL
  ;;     FFR→UFR, FFL→UFL, FDR→FBR→... (full permutation below)
  ;; ===========================================================

  ;; -----------------------------------------------------------
  ;; TILT_X_POS  (+90° around X: Front→Top, Down→Front, Back→Down, Top→Back)
  ;; Pick, tilt forward, place.
  ;; Physical meaning: robot tips cube so F face becomes new U face.
  ;;
  ;; Complete sticker permutation (all 24):
  ;;  U face → B face positions:  u-ufr→b-dbr, u-ufl→b-dbl, u-ubr→b-ubr*, u-ubl→b-ubl*
  ;;   NOTE: B face is viewed from behind, so orientation flips.
  ;;
  ;; Cleaner cycle description (face → face, where they go):
  ;;   New U = old F:  u-ufr←f-ufr, u-ufl←f-ufl, u-ubr←f-dfr, u-ubl←f-dfl
  ;;   New F = old D:  f-ufr←d-dfr, f-ufl←d-dfl, f-dfr←d-dbr, f-dfl←d-dbl
  ;;   New D = old B:  d-dfr←b-dbr, d-dfl←b-dbl, d-dbr←b-ubr, d-dbl←b-ubl
  ;;   New B = old U:  b-dbr←u-ufr, b-dbl←u-ufl, b-ubr←u-ubr, b-ubl←u-ubl
  ;;   L face rotates in-place (CW from left side view):
  ;;     l-ufl←l-ubl, l-ubl←l-dbl, l-dbl←l-dfl, l-dfl←l-ufl
  ;;   R face rotates in-place (CCW from right side view):
  ;;     r-ufr←r-dfr, r-dfr←r-dbr, r-dbr←r-ubr, r-ubr←r-ufr
  ;; -----------------------------------------------------------
  (:action tilt_x_pos
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and
      ;; New U ← old F
      (when (color-at f-ufr white)  (and (color-at u-ufr white)  (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow) (and (color-at u-ufr yellow) (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)    (and (color-at u-ufr red)    (not (color-at f-ufr red))))
      (when (color-at f-ufr orange) (and (color-at u-ufr orange) (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)   (and (color-at u-ufr blue)   (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)  (and (color-at u-ufr green)  (not (color-at f-ufr green))))

      (when (color-at f-ufl white)  (and (color-at u-ufl white)  (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow) (and (color-at u-ufl yellow) (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)    (and (color-at u-ufl red)    (not (color-at f-ufl red))))
      (when (color-at f-ufl orange) (and (color-at u-ufl orange) (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)   (and (color-at u-ufl blue)   (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)  (and (color-at u-ufl green)  (not (color-at f-ufl green))))

      (when (color-at f-dfr white)  (and (color-at u-ubr white)  (not (color-at f-dfr white))))
      (when (color-at f-dfr yellow) (and (color-at u-ubr yellow) (not (color-at f-dfr yellow))))
      (when (color-at f-dfr red)    (and (color-at u-ubr red)    (not (color-at f-dfr red))))
      (when (color-at f-dfr orange) (and (color-at u-ubr orange) (not (color-at f-dfr orange))))
      (when (color-at f-dfr blue)   (and (color-at u-ubr blue)   (not (color-at f-dfr blue))))
      (when (color-at f-dfr green)  (and (color-at u-ubr green)  (not (color-at f-dfr green))))

      (when (color-at f-dfl white)  (and (color-at u-ubl white)  (not (color-at f-dfl white))))
      (when (color-at f-dfl yellow) (and (color-at u-ubl yellow) (not (color-at f-dfl yellow))))
      (when (color-at f-dfl red)    (and (color-at u-ubl red)    (not (color-at f-dfl red))))
      (when (color-at f-dfl orange) (and (color-at u-ubl orange) (not (color-at f-dfl orange))))
      (when (color-at f-dfl blue)   (and (color-at u-ubl blue)   (not (color-at f-dfl blue))))
      (when (color-at f-dfl green)  (and (color-at u-ubl green)  (not (color-at f-dfl green))))

      ;; New F ← old D
      (when (color-at d-dfr white)  (and (color-at f-ufr white)  (not (color-at d-dfr white))))
      (when (color-at d-dfr yellow) (and (color-at f-ufr yellow) (not (color-at d-dfr yellow))))
      (when (color-at d-dfr red)    (and (color-at f-ufr red)    (not (color-at d-dfr red))))
      (when (color-at d-dfr orange) (and (color-at f-ufr orange) (not (color-at d-dfr orange))))
      (when (color-at d-dfr blue)   (and (color-at f-ufr blue)   (not (color-at d-dfr blue))))
      (when (color-at d-dfr green)  (and (color-at f-ufr green)  (not (color-at d-dfr green))))

      (when (color-at d-dfl white)  (and (color-at f-ufl white)  (not (color-at d-dfl white))))
      (when (color-at d-dfl yellow) (and (color-at f-ufl yellow) (not (color-at d-dfl yellow))))
      (when (color-at d-dfl red)    (and (color-at f-ufl red)    (not (color-at d-dfl red))))
      (when (color-at d-dfl orange) (and (color-at f-ufl orange) (not (color-at d-dfl orange))))
      (when (color-at d-dfl blue)   (and (color-at f-ufl blue)   (not (color-at d-dfl blue))))
      (when (color-at d-dfl green)  (and (color-at f-ufl green)  (not (color-at d-dfl green))))

      (when (color-at d-dbr white)  (and (color-at f-dfr white)  (not (color-at d-dbr white))))
      (when (color-at d-dbr yellow) (and (color-at f-dfr yellow) (not (color-at d-dbr yellow))))
      (when (color-at d-dbr red)    (and (color-at f-dfr red)    (not (color-at d-dbr red))))
      (when (color-at d-dbr orange) (and (color-at f-dfr orange) (not (color-at d-dbr orange))))
      (when (color-at d-dbr blue)   (and (color-at f-dfr blue)   (not (color-at d-dbr blue))))
      (when (color-at d-dbr green)  (and (color-at f-dfr green)  (not (color-at d-dbr green))))

      (when (color-at d-dbl white)  (and (color-at f-dfl white)  (not (color-at d-dbl white))))
      (when (color-at d-dbl yellow) (and (color-at f-dfl yellow) (not (color-at d-dbl yellow))))
      (when (color-at d-dbl red)    (and (color-at f-dfl red)    (not (color-at d-dbl red))))
      (when (color-at d-dbl orange) (and (color-at f-dfl orange) (not (color-at d-dbl orange))))
      (when (color-at d-dbl blue)   (and (color-at f-dfl blue)   (not (color-at d-dbl blue))))
      (when (color-at d-dbl green)  (and (color-at f-dfl green)  (not (color-at d-dbl green))))

      ;; New D ← old B  (B face viewed from behind → corners swap L/R)
      (when (color-at b-dbr white)  (and (color-at d-dfr white)  (not (color-at b-dbr white))))
      (when (color-at b-dbr yellow) (and (color-at d-dfr yellow) (not (color-at b-dbr yellow))))
      (when (color-at b-dbr red)    (and (color-at d-dfr red)    (not (color-at b-dbr red))))
      (when (color-at b-dbr orange) (and (color-at d-dfr orange) (not (color-at b-dbr orange))))
      (when (color-at b-dbr blue)   (and (color-at d-dfr blue)   (not (color-at b-dbr blue))))
      (when (color-at b-dbr green)  (and (color-at d-dfr green)  (not (color-at b-dbr green))))

      (when (color-at b-dbl white)  (and (color-at d-dfl white)  (not (color-at b-dbl white))))
      (when (color-at b-dbl yellow) (and (color-at d-dfl yellow) (not (color-at b-dbl yellow))))
      (when (color-at b-dbl red)    (and (color-at d-dfl red)    (not (color-at b-dbl red))))
      (when (color-at b-dbl orange) (and (color-at d-dfl orange) (not (color-at b-dbl orange))))
      (when (color-at b-dbl blue)   (and (color-at d-dfl blue)   (not (color-at b-dbl blue))))
      (when (color-at b-dbl green)  (and (color-at d-dfl green)  (not (color-at b-dbl green))))

      (when (color-at b-ubr white)  (and (color-at d-dbr white)  (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow) (and (color-at d-dbr yellow) (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)    (and (color-at d-dbr red)    (not (color-at b-ubr red))))
      (when (color-at b-ubr orange) (and (color-at d-dbr orange) (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)   (and (color-at d-dbr blue)   (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)  (and (color-at d-dbr green)  (not (color-at b-ubr green))))

      (when (color-at b-ubl white)  (and (color-at d-dbl white)  (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow) (and (color-at d-dbl yellow) (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)    (and (color-at d-dbl red)    (not (color-at b-ubl red))))
      (when (color-at b-ubl orange) (and (color-at d-dbl orange) (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)   (and (color-at d-dbl blue)   (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)  (and (color-at d-dbl green)  (not (color-at b-ubl green))))

      ;; New B ← old U  (goes to back, orientation flips)
      (when (color-at u-ufr white)  (and (color-at b-dbr white)  (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow) (and (color-at b-dbr yellow) (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)    (and (color-at b-dbr red)    (not (color-at u-ufr red))))
      (when (color-at u-ufr orange) (and (color-at b-dbr orange) (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)   (and (color-at b-dbr blue)   (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)  (and (color-at b-dbr green)  (not (color-at u-ufr green))))

      (when (color-at u-ufl white)  (and (color-at b-dbl white)  (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow) (and (color-at b-dbl yellow) (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)    (and (color-at b-dbl red)    (not (color-at u-ufl red))))
      (when (color-at u-ufl orange) (and (color-at b-dbl orange) (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)   (and (color-at b-dbl blue)   (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)  (and (color-at b-dbl green)  (not (color-at u-ufl green))))

      (when (color-at u-ubr white)  (and (color-at b-ubr white)  (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow) (and (color-at b-ubr yellow) (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)    (and (color-at b-ubr red)    (not (color-at u-ubr red))))
      (when (color-at u-ubr orange) (and (color-at b-ubr orange) (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)   (and (color-at b-ubr blue)   (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)  (and (color-at b-ubr green)  (not (color-at u-ubr green))))

      (when (color-at u-ubl white)  (and (color-at b-ubl white)  (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow) (and (color-at b-ubl yellow) (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)    (and (color-at b-ubl red)    (not (color-at u-ubl red))))
      (when (color-at u-ubl orange) (and (color-at b-ubl orange) (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)   (and (color-at b-ubl blue)   (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)  (and (color-at b-ubl green)  (not (color-at u-ubl green))))

      ;; L face rotates CW (viewed from left):  l-ufl←l-ubl←l-dbl←l-dfl←l-ufl
      (when (color-at l-ubl white)  (and (color-at l-ufl white)  (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow) (and (color-at l-ufl yellow) (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)    (and (color-at l-ufl red)    (not (color-at l-ubl red))))
      (when (color-at l-ubl orange) (and (color-at l-ufl orange) (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)   (and (color-at l-ufl blue)   (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)  (and (color-at l-ufl green)  (not (color-at l-ubl green))))

      (when (color-at l-dbl white)  (and (color-at l-ubl white)  (not (color-at l-dbl white))))
      (when (color-at l-dbl yellow) (and (color-at l-ubl yellow) (not (color-at l-dbl yellow))))
      (when (color-at l-dbl red)    (and (color-at l-ubl red)    (not (color-at l-dbl red))))
      (when (color-at l-dbl orange) (and (color-at l-ubl orange) (not (color-at l-dbl orange))))
      (when (color-at l-dbl blue)   (and (color-at l-ubl blue)   (not (color-at l-dbl blue))))
      (when (color-at l-dbl green)  (and (color-at l-ubl green)  (not (color-at l-dbl green))))

      (when (color-at l-dfl white)  (and (color-at l-dbl white)  (not (color-at l-dfl white))))
      (when (color-at l-dfl yellow) (and (color-at l-dbl yellow) (not (color-at l-dfl yellow))))
      (when (color-at l-dfl red)    (and (color-at l-dbl red)    (not (color-at l-dfl red))))
      (when (color-at l-dfl orange) (and (color-at l-dbl orange) (not (color-at l-dfl orange))))
      (when (color-at l-dfl blue)   (and (color-at l-dbl blue)   (not (color-at l-dfl blue))))
      (when (color-at l-dfl green)  (and (color-at l-dbl green)  (not (color-at l-dfl green))))

      (when (color-at l-ufl white)  (and (color-at l-dfl white)  (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow) (and (color-at l-dfl yellow) (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)    (and (color-at l-dfl red)    (not (color-at l-ufl red))))
      (when (color-at l-ufl orange) (and (color-at l-dfl orange) (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)   (and (color-at l-dfl blue)   (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)  (and (color-at l-dfl green)  (not (color-at l-ufl green))))

      ;; R face rotates CCW (viewed from right):  r-ufr←r-dfr←r-dbr←r-ubr←r-ufr
      (when (color-at r-dfr white)  (and (color-at r-ufr white)  (not (color-at r-dfr white))))
      (when (color-at r-dfr yellow) (and (color-at r-ufr yellow) (not (color-at r-dfr yellow))))
      (when (color-at r-dfr red)    (and (color-at r-ufr red)    (not (color-at r-dfr red))))
      (when (color-at r-dfr orange) (and (color-at r-ufr orange) (not (color-at r-dfr orange))))
      (when (color-at r-dfr blue)   (and (color-at r-ufr blue)   (not (color-at r-dfr blue))))
      (when (color-at r-dfr green)  (and (color-at r-ufr green)  (not (color-at r-dfr green))))

      (when (color-at r-dbr white)  (and (color-at r-dfr white)  (not (color-at r-dbr white))))
      (when (color-at r-dbr yellow) (and (color-at r-dfr yellow) (not (color-at r-dbr yellow))))
      (when (color-at r-dbr red)    (and (color-at r-dfr red)    (not (color-at r-dbr red))))
      (when (color-at r-dbr orange) (and (color-at r-dfr orange) (not (color-at r-dbr orange))))
      (when (color-at r-dbr blue)   (and (color-at r-dfr blue)   (not (color-at r-dbr blue))))
      (when (color-at r-dbr green)  (and (color-at r-dfr green)  (not (color-at r-dbr green))))

      (when (color-at r-ubr white)  (and (color-at r-dbr white)  (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow) (and (color-at r-dbr yellow) (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)    (and (color-at r-dbr red)    (not (color-at r-ubr red))))
      (when (color-at r-ubr orange) (and (color-at r-dbr orange) (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)   (and (color-at r-dbr blue)   (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)  (and (color-at r-dbr green)  (not (color-at r-ubr green))))

      (when (color-at r-ufr white)  (and (color-at r-ubr white)  (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow) (and (color-at r-ubr yellow) (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)    (and (color-at r-ubr red)    (not (color-at r-ufr red))))
      (when (color-at r-ufr orange) (and (color-at r-ubr orange) (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)   (and (color-at r-ubr blue)   (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)  (and (color-at r-ubr green)  (not (color-at r-ufr green))))
    )
  )

  ;; -----------------------------------------------------------
  ;; TILT_X_NEG  (-90° around X: Back→Top, Top→Front, Front→Down, Down→Back)
  ;; Inverse of tilt_x_pos.
  ;; Physical meaning: robot tips cube backward so B face becomes new U face.
  ;; -----------------------------------------------------------
  (:action tilt_x_neg
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and
      ;; New U ← old B
      (when (color-at b-dbr white)  (and (color-at u-ufr white)  (not (color-at b-dbr white))))
      (when (color-at b-dbr yellow) (and (color-at u-ufr yellow) (not (color-at b-dbr yellow))))
      (when (color-at b-dbr red)    (and (color-at u-ufr red)    (not (color-at b-dbr red))))
      (when (color-at b-dbr orange) (and (color-at u-ufr orange) (not (color-at b-dbr orange))))
      (when (color-at b-dbr blue)   (and (color-at u-ufr blue)   (not (color-at b-dbr blue))))
      (when (color-at b-dbr green)  (and (color-at u-ufr green)  (not (color-at b-dbr green))))

      (when (color-at b-dbl white)  (and (color-at u-ufl white)  (not (color-at b-dbl white))))
      (when (color-at b-dbl yellow) (and (color-at u-ufl yellow) (not (color-at b-dbl yellow))))
      (when (color-at b-dbl red)    (and (color-at u-ufl red)    (not (color-at b-dbl red))))
      (when (color-at b-dbl orange) (and (color-at u-ufl orange) (not (color-at b-dbl orange))))
      (when (color-at b-dbl blue)   (and (color-at u-ufl blue)   (not (color-at b-dbl blue))))
      (when (color-at b-dbl green)  (and (color-at u-ufl green)  (not (color-at b-dbl green))))

      (when (color-at b-ubr white)  (and (color-at u-ubr white)  (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow) (and (color-at u-ubr yellow) (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)    (and (color-at u-ubr red)    (not (color-at b-ubr red))))
      (when (color-at b-ubr orange) (and (color-at u-ubr orange) (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)   (and (color-at u-ubr blue)   (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)  (and (color-at u-ubr green)  (not (color-at b-ubr green))))

      (when (color-at b-ubl white)  (and (color-at u-ubl white)  (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow) (and (color-at u-ubl yellow) (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)    (and (color-at u-ubl red)    (not (color-at b-ubl red))))
      (when (color-at b-ubl orange) (and (color-at u-ubl orange) (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)   (and (color-at u-ubl blue)   (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)  (and (color-at u-ubl green)  (not (color-at b-ubl green))))

      ;; New B ← old D
      (when (color-at d-dfr white)  (and (color-at b-dbr white)  (not (color-at d-dfr white))))
      (when (color-at d-dfr yellow) (and (color-at b-dbr yellow) (not (color-at d-dfr yellow))))
      (when (color-at d-dfr red)    (and (color-at b-dbr red)    (not (color-at d-dfr red))))
      (when (color-at d-dfr orange) (and (color-at b-dbr orange) (not (color-at d-dfr orange))))
      (when (color-at d-dfr blue)   (and (color-at b-dbr blue)   (not (color-at d-dfr blue))))
      (when (color-at d-dfr green)  (and (color-at b-dbr green)  (not (color-at d-dfr green))))

      (when (color-at d-dfl white)  (and (color-at b-dbl white)  (not (color-at d-dfl white))))
      (when (color-at d-dfl yellow) (and (color-at b-dbl yellow) (not (color-at d-dfl yellow))))
      (when (color-at d-dfl red)    (and (color-at b-dbl red)    (not (color-at d-dfl red))))
      (when (color-at d-dfl orange) (and (color-at b-dbl orange) (not (color-at d-dfl orange))))
      (when (color-at d-dfl blue)   (and (color-at b-dbl blue)   (not (color-at d-dfl blue))))
      (when (color-at d-dfl green)  (and (color-at b-dbl green)  (not (color-at d-dfl green))))

      (when (color-at d-dbr white)  (and (color-at b-ubr white)  (not (color-at d-dbr white))))
      (when (color-at d-dbr yellow) (and (color-at b-ubr yellow) (not (color-at d-dbr yellow))))
      (when (color-at d-dbr red)    (and (color-at b-ubr red)    (not (color-at d-dbr red))))
      (when (color-at d-dbr orange) (and (color-at b-ubr orange) (not (color-at d-dbr orange))))
      (when (color-at d-dbr blue)   (and (color-at b-ubr blue)   (not (color-at d-dbr blue))))
      (when (color-at d-dbr green)  (and (color-at b-ubr green)  (not (color-at d-dbr green))))

      (when (color-at d-dbl white)  (and (color-at b-ubl white)  (not (color-at d-dbl white))))
      (when (color-at d-dbl yellow) (and (color-at b-ubl yellow) (not (color-at d-dbl yellow))))
      (when (color-at d-dbl red)    (and (color-at b-ubl red)    (not (color-at d-dbl red))))
      (when (color-at d-dbl orange) (and (color-at b-ubl orange) (not (color-at d-dbl orange))))
      (when (color-at d-dbl blue)   (and (color-at b-ubl blue)   (not (color-at d-dbl blue))))
      (when (color-at d-dbl green)  (and (color-at b-ubl green)  (not (color-at d-dbl green))))

      ;; New D ← old F
      (when (color-at f-ufr white)  (and (color-at d-dfr white)  (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow) (and (color-at d-dfr yellow) (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)    (and (color-at d-dfr red)    (not (color-at f-ufr red))))
      (when (color-at f-ufr orange) (and (color-at d-dfr orange) (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)   (and (color-at d-dfr blue)   (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)  (and (color-at d-dfr green)  (not (color-at f-ufr green))))

      (when (color-at f-ufl white)  (and (color-at d-dfl white)  (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow) (and (color-at d-dfl yellow) (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)    (and (color-at d-dfl red)    (not (color-at f-ufl red))))
      (when (color-at f-ufl orange) (and (color-at d-dfl orange) (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)   (and (color-at d-dfl blue)   (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)  (and (color-at d-dfl green)  (not (color-at f-ufl green))))

      (when (color-at f-dfr white)  (and (color-at d-dbr white)  (not (color-at f-dfr white))))
      (when (color-at f-dfr yellow) (and (color-at d-dbr yellow) (not (color-at f-dfr yellow))))
      (when (color-at f-dfr red)    (and (color-at d-dbr red)    (not (color-at f-dfr red))))
      (when (color-at f-dfr orange) (and (color-at d-dbr orange) (not (color-at f-dfr orange))))
      (when (color-at f-dfr blue)   (and (color-at d-dbr blue)   (not (color-at f-dfr blue))))
      (when (color-at f-dfr green)  (and (color-at d-dbr green)  (not (color-at f-dfr green))))

      (when (color-at f-dfl white)  (and (color-at d-dbl white)  (not (color-at f-dfl white))))
      (when (color-at f-dfl yellow) (and (color-at d-dbl yellow) (not (color-at f-dfl yellow))))
      (when (color-at f-dfl red)    (and (color-at d-dbl red)    (not (color-at f-dfl red))))
      (when (color-at f-dfl orange) (and (color-at d-dbl orange) (not (color-at f-dfl orange))))
      (when (color-at f-dfl blue)   (and (color-at d-dbl blue)   (not (color-at f-dfl blue))))
      (when (color-at f-dfl green)  (and (color-at d-dbl green)  (not (color-at f-dfl green))))

      ;; New F ← old U
      (when (color-at u-ufr white)  (and (color-at f-ufr white)  (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow) (and (color-at f-ufr yellow) (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)    (and (color-at f-ufr red)    (not (color-at u-ufr red))))
      (when (color-at u-ufr orange) (and (color-at f-ufr orange) (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)   (and (color-at f-ufr blue)   (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)  (and (color-at f-ufr green)  (not (color-at u-ufr green))))

      (when (color-at u-ufl white)  (and (color-at f-ufl white)  (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow) (and (color-at f-ufl yellow) (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)    (and (color-at f-ufl red)    (not (color-at u-ufl red))))
      (when (color-at u-ufl orange) (and (color-at f-ufl orange) (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)   (and (color-at f-ufl blue)   (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)  (and (color-at f-ufl green)  (not (color-at u-ufl green))))

      (when (color-at u-ubr white)  (and (color-at f-dfr white)  (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow) (and (color-at f-dfr yellow) (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)    (and (color-at f-dfr red)    (not (color-at u-ubr red))))
      (when (color-at u-ubr orange) (and (color-at f-dfr orange) (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)   (and (color-at f-dfr blue)   (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)  (and (color-at f-dfr green)  (not (color-at u-ubr green))))

      (when (color-at u-ubl white)  (and (color-at f-dfl white)  (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow) (and (color-at f-dfl yellow) (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)    (and (color-at f-dfl red)    (not (color-at u-ubl red))))
      (when (color-at u-ubl orange) (and (color-at f-dfl orange) (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)   (and (color-at f-dfl blue)   (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)  (and (color-at f-dfl green)  (not (color-at u-ubl green))))

      ;; L face rotates CCW (viewed from left):  l-ufl←l-dfl←l-dbl←l-ubl←l-ufl
      (when (color-at l-dfl white)  (and (color-at l-ufl white)  (not (color-at l-dfl white))))
      (when (color-at l-dfl yellow) (and (color-at l-ufl yellow) (not (color-at l-dfl yellow))))
      (when (color-at l-dfl red)    (and (color-at l-ufl red)    (not (color-at l-dfl red))))
      (when (color-at l-dfl orange) (and (color-at l-ufl orange) (not (color-at l-dfl orange))))
      (when (color-at l-dfl blue)   (and (color-at l-ufl blue)   (not (color-at l-dfl blue))))
      (when (color-at l-dfl green)  (and (color-at l-ufl green)  (not (color-at l-dfl green))))

      (when (color-at l-dbl white)  (and (color-at l-dfl white)  (not (color-at l-dbl white))))
      (when (color-at l-dbl yellow) (and (color-at l-dfl yellow) (not (color-at l-dbl yellow))))
      (when (color-at l-dbl red)    (and (color-at l-dfl red)    (not (color-at l-dbl red))))
      (when (color-at l-dbl orange) (and (color-at l-dfl orange) (not (color-at l-dbl orange))))
      (when (color-at l-dbl blue)   (and (color-at l-dfl blue)   (not (color-at l-dbl blue))))
      (when (color-at l-dbl green)  (and (color-at l-dfl green)  (not (color-at l-dbl green))))

      (when (color-at l-ubl white)  (and (color-at l-dbl white)  (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow) (and (color-at l-dbl yellow) (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)    (and (color-at l-dbl red)    (not (color-at l-ubl red))))
      (when (color-at l-ubl orange) (and (color-at l-dbl orange) (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)   (and (color-at l-dbl blue)   (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)  (and (color-at l-dbl green)  (not (color-at l-ubl green))))

      (when (color-at l-ufl white)  (and (color-at l-ubl white)  (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow) (and (color-at l-ubl yellow) (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)    (and (color-at l-ubl red)    (not (color-at l-ufl red))))
      (when (color-at l-ufl orange) (and (color-at l-ubl orange) (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)   (and (color-at l-ubl blue)   (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)  (and (color-at l-ubl green)  (not (color-at l-ufl green))))

      ;; R face rotates CW (viewed from right):  r-ufr←r-ubr←r-dbr←r-dfr←r-ufr
      (when (color-at r-ubr white)  (and (color-at r-ufr white)  (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow) (and (color-at r-ufr yellow) (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)    (and (color-at r-ufr red)    (not (color-at r-ubr red))))
      (when (color-at r-ubr orange) (and (color-at r-ufr orange) (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)   (and (color-at r-ufr blue)   (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)  (and (color-at r-ufr green)  (not (color-at r-ubr green))))

      (when (color-at r-dbr white)  (and (color-at r-ubr white)  (not (color-at r-dbr white))))
      (when (color-at r-dbr yellow) (and (color-at r-ubr yellow) (not (color-at r-dbr yellow))))
      (when (color-at r-dbr red)    (and (color-at r-ubr red)    (not (color-at r-dbr red))))
      (when (color-at r-dbr orange) (and (color-at r-ubr orange) (not (color-at r-dbr orange))))
      (when (color-at r-dbr blue)   (and (color-at r-ubr blue)   (not (color-at r-dbr blue))))
      (when (color-at r-dbr green)  (and (color-at r-ubr green)  (not (color-at r-dbr green))))

      (when (color-at r-dfr white)  (and (color-at r-dbr white)  (not (color-at r-dfr white))))
      (when (color-at r-dfr yellow) (and (color-at r-dbr yellow) (not (color-at r-dfr yellow))))
      (when (color-at r-dfr red)    (and (color-at r-dbr red)    (not (color-at r-dfr red))))
      (when (color-at r-dfr orange) (and (color-at r-dbr orange) (not (color-at r-dfr orange))))
      (when (color-at r-dfr blue)   (and (color-at r-dbr blue)   (not (color-at r-dfr blue))))
      (when (color-at r-dfr green)  (and (color-at r-dbr green)  (not (color-at r-dfr green))))

      (when (color-at r-ufr white)  (and (color-at r-dfr white)  (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow) (and (color-at r-dfr yellow) (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)    (and (color-at r-dfr red)    (not (color-at r-ufr red))))
      (when (color-at r-ufr orange) (and (color-at r-dfr orange) (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)   (and (color-at r-dfr blue)   (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)  (and (color-at r-dfr green)  (not (color-at r-ufr green))))
    )
  )

  ;; -----------------------------------------------------------
  ;; TILT_Y_POS  (+90° around Y: Right→Top, Top→Left, Left→Down, Down→Right)
  ;; Physical meaning: robot tips cube rightward so R face becomes new U face.
  ;;
  ;; Face mapping:  U→L, L→D, D→R, R→U  (F and B stay, but rotate in-place)
  ;;   New U ← old R:  u-ufr←r-ufr, u-ubr←r-ubr, u-ufl←r-dfr, u-ubl←r-dbr
  ;;   New R ← old D:  r-ufr←d-dfr, r-ubr←d-dbr, r-dfr←d-dfl, r-dbr←d-dbl
  ;;   New D ← old L:  d-dfr←l-ufl, d-dfl←l-dfl, d-dbr←l-ubl, d-dbl←l-dbl
  ;;   New L ← old U:  l-ufl←u-ufr, l-ubl←u-ubr, l-dfl←u-ufl, l-dbl←u-ubl
  ;;   Wait — need exact corner mapping. Let me use cycle notation.
  ;;
  ;; Cycle (Y+ tilt, R→top): for corner (c) identity map:
  ;;   UFR corner: r-ufr→u-ufr, u-ufr→l-dfl*, l-dfl→d-dfr*, d-dfr→r-ufr
  ;;   Actually, simpler: the 4 cycles for Y+ are:
  ;;     [r-ufr, u-ufr, l-dfl, d-dfr]  – corners going: R-face→U,U→L,L→D,D→R
  ;;     [r-ubr, u-ubr, l-dbl, d-dbr]
  ;;     [r-dfr, u-ufl, l-ufl, d-dfl]
  ;;     [r-dbr, u-ubl, l-ubl, d-dbl]
  ;; F face rotates CW (from front): f-ufl→f-ufr→f-dfr→f-dfl→f-ufl
  ;; B face rotates CCW (from back):  b-ubr→b-ubl→b-dbl→b-dbr→b-ubr
  ;; -----------------------------------------------------------
  (:action tilt_y_pos
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and
      ;; Cycle [r-ufr → u-ufr → l-dfl → d-dfr → r-ufr]  (content flows CW)
      (when (color-at r-ufr white)  (and (color-at u-ufr white)  (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow) (and (color-at u-ufr yellow) (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)    (and (color-at u-ufr red)    (not (color-at r-ufr red))))
      (when (color-at r-ufr orange) (and (color-at u-ufr orange) (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)   (and (color-at u-ufr blue)   (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)  (and (color-at u-ufr green)  (not (color-at r-ufr green))))

      (when (color-at u-ufr white)  (and (color-at l-dfl white)  (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow) (and (color-at l-dfl yellow) (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)    (and (color-at l-dfl red)    (not (color-at u-ufr red))))
      (when (color-at u-ufr orange) (and (color-at l-dfl orange) (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)   (and (color-at l-dfl blue)   (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)  (and (color-at l-dfl green)  (not (color-at u-ufr green))))

      (when (color-at l-dfl white)  (and (color-at d-dfr white)  (not (color-at l-dfl white))))
      (when (color-at l-dfl yellow) (and (color-at d-dfr yellow) (not (color-at l-dfl yellow))))
      (when (color-at l-dfl red)    (and (color-at d-dfr red)    (not (color-at l-dfl red))))
      (when (color-at l-dfl orange) (and (color-at d-dfr orange) (not (color-at l-dfl orange))))
      (when (color-at l-dfl blue)   (and (color-at d-dfr blue)   (not (color-at l-dfl blue))))
      (when (color-at l-dfl green)  (and (color-at d-dfr green)  (not (color-at l-dfl green))))

      (when (color-at d-dfr white)  (and (color-at r-ufr white)  (not (color-at d-dfr white))))
      (when (color-at d-dfr yellow) (and (color-at r-ufr yellow) (not (color-at d-dfr yellow))))
      (when (color-at d-dfr red)    (and (color-at r-ufr red)    (not (color-at d-dfr red))))
      (when (color-at d-dfr orange) (and (color-at r-ufr orange) (not (color-at d-dfr orange))))
      (when (color-at d-dfr blue)   (and (color-at r-ufr blue)   (not (color-at d-dfr blue))))
      (when (color-at d-dfr green)  (and (color-at r-ufr green)  (not (color-at d-dfr green))))

      ;; Cycle [r-ubr → u-ubr → l-dbl → d-dbr → r-ubr]
      (when (color-at r-ubr white)  (and (color-at u-ubr white)  (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow) (and (color-at u-ubr yellow) (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)    (and (color-at u-ubr red)    (not (color-at r-ubr red))))
      (when (color-at r-ubr orange) (and (color-at u-ubr orange) (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)   (and (color-at u-ubr blue)   (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)  (and (color-at u-ubr green)  (not (color-at r-ubr green))))

      (when (color-at u-ubr white)  (and (color-at l-dbl white)  (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow) (and (color-at l-dbl yellow) (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)    (and (color-at l-dbl red)    (not (color-at u-ubr red))))
      (when (color-at u-ubr orange) (and (color-at l-dbl orange) (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)   (and (color-at l-dbl blue)   (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)  (and (color-at l-dbl green)  (not (color-at u-ubr green))))

      (when (color-at l-dbl white)  (and (color-at d-dbr white)  (not (color-at l-dbl white))))
      (when (color-at l-dbl yellow) (and (color-at d-dbr yellow) (not (color-at l-dbl yellow))))
      (when (color-at l-dbl red)    (and (color-at d-dbr red)    (not (color-at l-dbl red))))
      (when (color-at l-dbl orange) (and (color-at d-dbr orange) (not (color-at l-dbl orange))))
      (when (color-at l-dbl blue)   (and (color-at d-dbr blue)   (not (color-at l-dbl blue))))
      (when (color-at l-dbl green)  (and (color-at d-dbr green)  (not (color-at l-dbl green))))

      (when (color-at d-dbr white)  (and (color-at r-ubr white)  (not (color-at d-dbr white))))
      (when (color-at d-dbr yellow) (and (color-at r-ubr yellow) (not (color-at d-dbr yellow))))
      (when (color-at d-dbr red)    (and (color-at r-ubr red)    (not (color-at d-dbr red))))
      (when (color-at d-dbr orange) (and (color-at r-ubr orange) (not (color-at d-dbr orange))))
      (when (color-at d-dbr blue)   (and (color-at r-ubr blue)   (not (color-at d-dbr blue))))
      (when (color-at d-dbr green)  (and (color-at r-ubr green)  (not (color-at d-dbr green))))

      ;; Cycle [r-dfr → u-ufl → l-ufl → d-dfl → r-dfr]
      (when (color-at r-dfr white)  (and (color-at u-ufl white)  (not (color-at r-dfr white))))
      (when (color-at r-dfr yellow) (and (color-at u-ufl yellow) (not (color-at r-dfr yellow))))
      (when (color-at r-dfr red)    (and (color-at u-ufl red)    (not (color-at r-dfr red))))
      (when (color-at r-dfr orange) (and (color-at u-ufl orange) (not (color-at r-dfr orange))))
      (when (color-at r-dfr blue)   (and (color-at u-ufl blue)   (not (color-at r-dfr blue))))
      (when (color-at r-dfr green)  (and (color-at u-ufl green)  (not (color-at r-dfr green))))

      (when (color-at u-ufl white)  (and (color-at l-ufl white)  (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow) (and (color-at l-ufl yellow) (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)    (and (color-at l-ufl red)    (not (color-at u-ufl red))))
      (when (color-at u-ufl orange) (and (color-at l-ufl orange) (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)   (and (color-at l-ufl blue)   (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)  (and (color-at l-ufl green)  (not (color-at u-ufl green))))

      (when (color-at l-ufl white)  (and (color-at d-dfl white)  (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow) (and (color-at d-dfl yellow) (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)    (and (color-at d-dfl red)    (not (color-at l-ufl red))))
      (when (color-at l-ufl orange) (and (color-at d-dfl orange) (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)   (and (color-at d-dfl blue)   (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)  (and (color-at d-dfl green)  (not (color-at l-ufl green))))

      (when (color-at d-dfl white)  (and (color-at r-dfr white)  (not (color-at d-dfl white))))
      (when (color-at d-dfl yellow) (and (color-at r-dfr yellow) (not (color-at d-dfl yellow))))
      (when (color-at d-dfl red)    (and (color-at r-dfr red)    (not (color-at d-dfl red))))
      (when (color-at d-dfl orange) (and (color-at r-dfr orange) (not (color-at d-dfl orange))))
      (when (color-at d-dfl blue)   (and (color-at r-dfr blue)   (not (color-at d-dfl blue))))
      (when (color-at d-dfl green)  (and (color-at r-dfr green)  (not (color-at d-dfl green))))

      ;; Cycle [r-dbr → u-ubl → l-ubl → d-dbl → r-dbr]
      (when (color-at r-dbr white)  (and (color-at u-ubl white)  (not (color-at r-dbr white))))
      (when (color-at r-dbr yellow) (and (color-at u-ubl yellow) (not (color-at r-dbr yellow))))
      (when (color-at r-dbr red)    (and (color-at u-ubl red)    (not (color-at r-dbr red))))
      (when (color-at r-dbr orange) (and (color-at u-ubl orange) (not (color-at r-dbr orange))))
      (when (color-at r-dbr blue)   (and (color-at u-ubl blue)   (not (color-at r-dbr blue))))
      (when (color-at r-dbr green)  (and (color-at u-ubl green)  (not (color-at r-dbr green))))

      (when (color-at u-ubl white)  (and (color-at l-ubl white)  (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow) (and (color-at l-ubl yellow) (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)    (and (color-at l-ubl red)    (not (color-at u-ubl red))))
      (when (color-at u-ubl orange) (and (color-at l-ubl orange) (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)   (and (color-at l-ubl blue)   (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)  (and (color-at l-ubl green)  (not (color-at u-ubl green))))

      (when (color-at l-ubl white)  (and (color-at d-dbl white)  (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow) (and (color-at d-dbl yellow) (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)    (and (color-at d-dbl red)    (not (color-at l-ubl red))))
      (when (color-at l-ubl orange) (and (color-at d-dbl orange) (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)   (and (color-at d-dbl blue)   (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)  (and (color-at d-dbl green)  (not (color-at l-ubl green))))

      (when (color-at d-dbl white)  (and (color-at r-dbr white)  (not (color-at d-dbl white))))
      (when (color-at d-dbl yellow) (and (color-at r-dbr yellow) (not (color-at d-dbl yellow))))
      (when (color-at d-dbl red)    (and (color-at r-dbr red)    (not (color-at d-dbl red))))
      (when (color-at d-dbl orange) (and (color-at r-dbr orange) (not (color-at d-dbl orange))))
      (when (color-at d-dbl blue)   (and (color-at r-dbr blue)   (not (color-at d-dbl blue))))
      (when (color-at d-dbl green)  (and (color-at r-dbr green)  (not (color-at d-dbl green))))

      ;; F face rotates CW (from front view): f-ufl→f-ufr→f-dfr→f-dfl→f-ufl
      (when (color-at f-ufl white)  (and (color-at f-ufr white)  (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow) (and (color-at f-ufr yellow) (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)    (and (color-at f-ufr red)    (not (color-at f-ufl red))))
      (when (color-at f-ufl orange) (and (color-at f-ufr orange) (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)   (and (color-at f-ufr blue)   (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)  (and (color-at f-ufr green)  (not (color-at f-ufl green))))

      (when (color-at f-ufr white)  (and (color-at f-dfr white)  (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow) (and (color-at f-dfr yellow) (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)    (and (color-at f-dfr red)    (not (color-at f-ufr red))))
      (when (color-at f-ufr orange) (and (color-at f-dfr orange) (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)   (and (color-at f-dfr blue)   (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)  (and (color-at f-dfr green)  (not (color-at f-ufr green))))

      (when (color-at f-dfr white)  (and (color-at f-dfl white)  (not (color-at f-dfr white))))
      (when (color-at f-dfr yellow) (and (color-at f-dfl yellow) (not (color-at f-dfr yellow))))
      (when (color-at f-dfr red)    (and (color-at f-dfl red)    (not (color-at f-dfr red))))
      (when (color-at f-dfr orange) (and (color-at f-dfl orange) (not (color-at f-dfr orange))))
      (when (color-at f-dfr blue)   (and (color-at f-dfl blue)   (not (color-at f-dfr blue))))
      (when (color-at f-dfr green)  (and (color-at f-dfl green)  (not (color-at f-dfr green))))

      (when (color-at f-dfl white)  (and (color-at f-ufl white)  (not (color-at f-dfl white))))
      (when (color-at f-dfl yellow) (and (color-at f-ufl yellow) (not (color-at f-dfl yellow))))
      (when (color-at f-dfl red)    (and (color-at f-ufl red)    (not (color-at f-dfl red))))
      (when (color-at f-dfl orange) (and (color-at f-ufl orange) (not (color-at f-dfl orange))))
      (when (color-at f-dfl blue)   (and (color-at f-ufl blue)   (not (color-at f-dfl blue))))
      (when (color-at f-dfl green)  (and (color-at f-ufl green)  (not (color-at f-dfl green))))

      ;; B face rotates CCW (from back view): b-ubr→b-ubl→b-dbl→b-dbr→b-ubr
      (when (color-at b-ubl white)  (and (color-at b-ubr white)  (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow) (and (color-at b-ubr yellow) (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)    (and (color-at b-ubr red)    (not (color-at b-ubl red))))
      (when (color-at b-ubl orange) (and (color-at b-ubr orange) (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)   (and (color-at b-ubr blue)   (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)  (and (color-at b-ubr green)  (not (color-at b-ubl green))))

      (when (color-at b-dbl white)  (and (color-at b-ubl white)  (not (color-at b-dbl white))))
      (when (color-at b-dbl yellow) (and (color-at b-ubl yellow) (not (color-at b-dbl yellow))))
      (when (color-at b-dbl red)    (and (color-at b-ubl red)    (not (color-at b-dbl red))))
      (when (color-at b-dbl orange) (and (color-at b-ubl orange) (not (color-at b-dbl orange))))
      (when (color-at b-dbl blue)   (and (color-at b-ubl blue)   (not (color-at b-dbl blue))))
      (when (color-at b-dbl green)  (and (color-at b-ubl green)  (not (color-at b-dbl green))))

      (when (color-at b-dbr white)  (and (color-at b-dbl white)  (not (color-at b-dbr white))))
      (when (color-at b-dbr yellow) (and (color-at b-dbl yellow) (not (color-at b-dbr yellow))))
      (when (color-at b-dbr red)    (and (color-at b-dbl red)    (not (color-at b-dbr red))))
      (when (color-at b-dbr orange) (and (color-at b-dbl orange) (not (color-at b-dbr orange))))
      (when (color-at b-dbr blue)   (and (color-at b-dbl blue)   (not (color-at b-dbr blue))))
      (when (color-at b-dbr green)  (and (color-at b-dbl green)  (not (color-at b-dbr green))))

      (when (color-at b-ubr white)  (and (color-at b-dbr white)  (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow) (and (color-at b-dbr yellow) (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)    (and (color-at b-dbr red)    (not (color-at b-ubr red))))
      (when (color-at b-ubr orange) (and (color-at b-dbr orange) (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)   (and (color-at b-dbr blue)   (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)  (and (color-at b-dbr green)  (not (color-at b-ubr green))))
    )
  )

  ;; -----------------------------------------------------------
  ;; TILT_Y_NEG  (-90° around Y: Left→Top, Top→Right, Right→Down, Down→Left)
  ;; Inverse of tilt_y_pos.
  ;; Physical meaning: robot tips cube leftward so L face becomes new U face.
  ;; -----------------------------------------------------------
  (:action tilt_y_neg
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and
      ;; Inverse of tilt_y_pos: cycle goes in reverse
      ;; Cycle [r-ufr ← u-ufr ← l-dfl ← d-dfr ← r-ufr] reversed
      (when (color-at u-ufr white)  (and (color-at r-ufr white)  (not (color-at u-ufr white))))
      (when (color-at u-ufr yellow) (and (color-at r-ufr yellow) (not (color-at u-ufr yellow))))
      (when (color-at u-ufr red)    (and (color-at r-ufr red)    (not (color-at u-ufr red))))
      (when (color-at u-ufr orange) (and (color-at r-ufr orange) (not (color-at u-ufr orange))))
      (when (color-at u-ufr blue)   (and (color-at r-ufr blue)   (not (color-at u-ufr blue))))
      (when (color-at u-ufr green)  (and (color-at r-ufr green)  (not (color-at u-ufr green))))

      (when (color-at l-dfl white)  (and (color-at u-ufr white)  (not (color-at l-dfl white))))
      (when (color-at l-dfl yellow) (and (color-at u-ufr yellow) (not (color-at l-dfl yellow))))
      (when (color-at l-dfl red)    (and (color-at u-ufr red)    (not (color-at l-dfl red))))
      (when (color-at l-dfl orange) (and (color-at u-ufr orange) (not (color-at l-dfl orange))))
      (when (color-at l-dfl blue)   (and (color-at u-ufr blue)   (not (color-at l-dfl blue))))
      (when (color-at l-dfl green)  (and (color-at u-ufr green)  (not (color-at l-dfl green))))

      (when (color-at d-dfr white)  (and (color-at l-dfl white)  (not (color-at d-dfr white))))
      (when (color-at d-dfr yellow) (and (color-at l-dfl yellow) (not (color-at d-dfr yellow))))
      (when (color-at d-dfr red)    (and (color-at l-dfl red)    (not (color-at d-dfr red))))
      (when (color-at d-dfr orange) (and (color-at l-dfl orange) (not (color-at d-dfr orange))))
      (when (color-at d-dfr blue)   (and (color-at l-dfl blue)   (not (color-at d-dfr blue))))
      (when (color-at d-dfr green)  (and (color-at l-dfl green)  (not (color-at d-dfr green))))

      (when (color-at r-ufr white)  (and (color-at d-dfr white)  (not (color-at r-ufr white))))
      (when (color-at r-ufr yellow) (and (color-at d-dfr yellow) (not (color-at r-ufr yellow))))
      (when (color-at r-ufr red)    (and (color-at d-dfr red)    (not (color-at r-ufr red))))
      (when (color-at r-ufr orange) (and (color-at d-dfr orange) (not (color-at r-ufr orange))))
      (when (color-at r-ufr blue)   (and (color-at d-dfr blue)   (not (color-at r-ufr blue))))
      (when (color-at r-ufr green)  (and (color-at d-dfr green)  (not (color-at r-ufr green))))

      ;; Cycle [r-ubr ← u-ubr ← l-dbl ← d-dbr] reversed
      (when (color-at u-ubr white)  (and (color-at r-ubr white)  (not (color-at u-ubr white))))
      (when (color-at u-ubr yellow) (and (color-at r-ubr yellow) (not (color-at u-ubr yellow))))
      (when (color-at u-ubr red)    (and (color-at r-ubr red)    (not (color-at u-ubr red))))
      (when (color-at u-ubr orange) (and (color-at r-ubr orange) (not (color-at u-ubr orange))))
      (when (color-at u-ubr blue)   (and (color-at r-ubr blue)   (not (color-at u-ubr blue))))
      (when (color-at u-ubr green)  (and (color-at r-ubr green)  (not (color-at u-ubr green))))

      (when (color-at l-dbl white)  (and (color-at u-ubr white)  (not (color-at l-dbl white))))
      (when (color-at l-dbl yellow) (and (color-at u-ubr yellow) (not (color-at l-dbl yellow))))
      (when (color-at l-dbl red)    (and (color-at u-ubr red)    (not (color-at l-dbl red))))
      (when (color-at l-dbl orange) (and (color-at u-ubr orange) (not (color-at l-dbl orange))))
      (when (color-at l-dbl blue)   (and (color-at u-ubr blue)   (not (color-at l-dbl blue))))
      (when (color-at l-dbl green)  (and (color-at u-ubr green)  (not (color-at l-dbl green))))

      (when (color-at d-dbr white)  (and (color-at l-dbl white)  (not (color-at d-dbr white))))
      (when (color-at d-dbr yellow) (and (color-at l-dbl yellow) (not (color-at d-dbr yellow))))
      (when (color-at d-dbr red)    (and (color-at l-dbl red)    (not (color-at d-dbr red))))
      (when (color-at d-dbr orange) (and (color-at l-dbl orange) (not (color-at d-dbr orange))))
      (when (color-at d-dbr blue)   (and (color-at l-dbl blue)   (not (color-at d-dbr blue))))
      (when (color-at d-dbr green)  (and (color-at l-dbl green)  (not (color-at d-dbr green))))

      (when (color-at r-ubr white)  (and (color-at d-dbr white)  (not (color-at r-ubr white))))
      (when (color-at r-ubr yellow) (and (color-at d-dbr yellow) (not (color-at r-ubr yellow))))
      (when (color-at r-ubr red)    (and (color-at d-dbr red)    (not (color-at r-ubr red))))
      (when (color-at r-ubr orange) (and (color-at d-dbr orange) (not (color-at r-ubr orange))))
      (when (color-at r-ubr blue)   (and (color-at d-dbr blue)   (not (color-at r-ubr blue))))
      (when (color-at r-ubr green)  (and (color-at d-dbr green)  (not (color-at r-ubr green))))

      ;; Cycle [r-dfr ← u-ufl ← l-ufl ← d-dfl] reversed
      (when (color-at u-ufl white)  (and (color-at r-dfr white)  (not (color-at u-ufl white))))
      (when (color-at u-ufl yellow) (and (color-at r-dfr yellow) (not (color-at u-ufl yellow))))
      (when (color-at u-ufl red)    (and (color-at r-dfr red)    (not (color-at u-ufl red))))
      (when (color-at u-ufl orange) (and (color-at r-dfr orange) (not (color-at u-ufl orange))))
      (when (color-at u-ufl blue)   (and (color-at r-dfr blue)   (not (color-at u-ufl blue))))
      (when (color-at u-ufl green)  (and (color-at r-dfr green)  (not (color-at u-ufl green))))

      (when (color-at l-ufl white)  (and (color-at u-ufl white)  (not (color-at l-ufl white))))
      (when (color-at l-ufl yellow) (and (color-at u-ufl yellow) (not (color-at l-ufl yellow))))
      (when (color-at l-ufl red)    (and (color-at u-ufl red)    (not (color-at l-ufl red))))
      (when (color-at l-ufl orange) (and (color-at u-ufl orange) (not (color-at l-ufl orange))))
      (when (color-at l-ufl blue)   (and (color-at u-ufl blue)   (not (color-at l-ufl blue))))
      (when (color-at l-ufl green)  (and (color-at u-ufl green)  (not (color-at l-ufl green))))

      (when (color-at d-dfl white)  (and (color-at l-ufl white)  (not (color-at d-dfl white))))
      (when (color-at d-dfl yellow) (and (color-at l-ufl yellow) (not (color-at d-dfl yellow))))
      (when (color-at d-dfl red)    (and (color-at l-ufl red)    (not (color-at d-dfl red))))
      (when (color-at d-dfl orange) (and (color-at l-ufl orange) (not (color-at d-dfl orange))))
      (when (color-at d-dfl blue)   (and (color-at l-ufl blue)   (not (color-at d-dfl blue))))
      (when (color-at d-dfl green)  (and (color-at l-ufl green)  (not (color-at d-dfl green))))

      (when (color-at r-dfr white)  (and (color-at d-dfl white)  (not (color-at r-dfr white))))
      (when (color-at r-dfr yellow) (and (color-at d-dfl yellow) (not (color-at r-dfr yellow))))
      (when (color-at r-dfr red)    (and (color-at d-dfl red)    (not (color-at r-dfr red))))
      (when (color-at r-dfr orange) (and (color-at d-dfl orange) (not (color-at r-dfr orange))))
      (when (color-at r-dfr blue)   (and (color-at d-dfl blue)   (not (color-at r-dfr blue))))
      (when (color-at r-dfr green)  (and (color-at d-dfl green)  (not (color-at r-dfr green))))

      ;; Cycle [r-dbr ← u-ubl ← l-ubl ← d-dbl] reversed
      (when (color-at u-ubl white)  (and (color-at r-dbr white)  (not (color-at u-ubl white))))
      (when (color-at u-ubl yellow) (and (color-at r-dbr yellow) (not (color-at u-ubl yellow))))
      (when (color-at u-ubl red)    (and (color-at r-dbr red)    (not (color-at u-ubl red))))
      (when (color-at u-ubl orange) (and (color-at r-dbr orange) (not (color-at u-ubl orange))))
      (when (color-at u-ubl blue)   (and (color-at r-dbr blue)   (not (color-at u-ubl blue))))
      (when (color-at u-ubl green)  (and (color-at r-dbr green)  (not (color-at u-ubl green))))

      (when (color-at l-ubl white)  (and (color-at u-ubl white)  (not (color-at l-ubl white))))
      (when (color-at l-ubl yellow) (and (color-at u-ubl yellow) (not (color-at l-ubl yellow))))
      (when (color-at l-ubl red)    (and (color-at u-ubl red)    (not (color-at l-ubl red))))
      (when (color-at l-ubl orange) (and (color-at u-ubl orange) (not (color-at l-ubl orange))))
      (when (color-at l-ubl blue)   (and (color-at u-ubl blue)   (not (color-at l-ubl blue))))
      (when (color-at l-ubl green)  (and (color-at u-ubl green)  (not (color-at l-ubl green))))

      (when (color-at d-dbl white)  (and (color-at l-ubl white)  (not (color-at d-dbl white))))
      (when (color-at d-dbl yellow) (and (color-at l-ubl yellow) (not (color-at d-dbl yellow))))
      (when (color-at d-dbl red)    (and (color-at l-ubl red)    (not (color-at d-dbl red))))
      (when (color-at d-dbl orange) (and (color-at l-ubl orange) (not (color-at d-dbl orange))))
      (when (color-at d-dbl blue)   (and (color-at l-ubl blue)   (not (color-at d-dbl blue))))
      (when (color-at d-dbl green)  (and (color-at l-ubl green)  (not (color-at d-dbl green))))

      (when (color-at r-dbr white)  (and (color-at d-dbl white)  (not (color-at r-dbr white))))
      (when (color-at r-dbr yellow) (and (color-at d-dbl yellow) (not (color-at r-dbr yellow))))
      (when (color-at r-dbr red)    (and (color-at d-dbl red)    (not (color-at r-dbr red))))
      (when (color-at r-dbr orange) (and (color-at d-dbl orange) (not (color-at r-dbr orange))))
      (when (color-at r-dbr blue)   (and (color-at d-dbl blue)   (not (color-at r-dbr blue))))
      (when (color-at r-dbr green)  (and (color-at d-dbl green)  (not (color-at r-dbr green))))

      ;; F face rotates CCW: f-ufr→f-ufl→f-dfl→f-dfr→f-ufr
      (when (color-at f-ufr white)  (and (color-at f-ufl white)  (not (color-at f-ufr white))))
      (when (color-at f-ufr yellow) (and (color-at f-ufl yellow) (not (color-at f-ufr yellow))))
      (when (color-at f-ufr red)    (and (color-at f-ufl red)    (not (color-at f-ufr red))))
      (when (color-at f-ufr orange) (and (color-at f-ufl orange) (not (color-at f-ufr orange))))
      (when (color-at f-ufr blue)   (and (color-at f-ufl blue)   (not (color-at f-ufr blue))))
      (when (color-at f-ufr green)  (and (color-at f-ufl green)  (not (color-at f-ufr green))))

      (when (color-at f-ufl white)  (and (color-at f-dfl white)  (not (color-at f-ufl white))))
      (when (color-at f-ufl yellow) (and (color-at f-dfl yellow) (not (color-at f-ufl yellow))))
      (when (color-at f-ufl red)    (and (color-at f-dfl red)    (not (color-at f-ufl red))))
      (when (color-at f-ufl orange) (and (color-at f-dfl orange) (not (color-at f-ufl orange))))
      (when (color-at f-ufl blue)   (and (color-at f-dfl blue)   (not (color-at f-ufl blue))))
      (when (color-at f-ufl green)  (and (color-at f-dfl green)  (not (color-at f-ufl green))))

      (when (color-at f-dfl white)  (and (color-at f-dfr white)  (not (color-at f-dfl white))))
      (when (color-at f-dfl yellow) (and (color-at f-dfr yellow) (not (color-at f-dfl yellow))))
      (when (color-at f-dfl red)    (and (color-at f-dfr red)    (not (color-at f-dfl red))))
      (when (color-at f-dfl orange) (and (color-at f-dfr orange) (not (color-at f-dfl orange))))
      (when (color-at f-dfl blue)   (and (color-at f-dfr blue)   (not (color-at f-dfl blue))))
      (when (color-at f-dfl green)  (and (color-at f-dfr green)  (not (color-at f-dfl green))))

      (when (color-at f-dfr white)  (and (color-at f-ufr white)  (not (color-at f-dfr white))))
      (when (color-at f-dfr yellow) (and (color-at f-ufr yellow) (not (color-at f-dfr yellow))))
      (when (color-at f-dfr red)    (and (color-at f-ufr red)    (not (color-at f-dfr red))))
      (when (color-at f-dfr orange) (and (color-at f-ufr orange) (not (color-at f-dfr orange))))
      (when (color-at f-dfr blue)   (and (color-at f-ufr blue)   (not (color-at f-dfr blue))))
      (when (color-at f-dfr green)  (and (color-at f-ufr green)  (not (color-at f-dfr green))))

      ;; B face rotates CW: b-ubr→b-dbr→b-dbl→b-ubl→b-ubr
      (when (color-at b-ubr white)  (and (color-at b-dbr white)  (not (color-at b-ubr white))))
      (when (color-at b-ubr yellow) (and (color-at b-dbr yellow) (not (color-at b-ubr yellow))))
      (when (color-at b-ubr red)    (and (color-at b-dbr red)    (not (color-at b-ubr red))))
      (when (color-at b-ubr orange) (and (color-at b-dbr orange) (not (color-at b-ubr orange))))
      (when (color-at b-ubr blue)   (and (color-at b-dbr blue)   (not (color-at b-ubr blue))))
      (when (color-at b-ubr green)  (and (color-at b-dbr green)  (not (color-at b-ubr green))))

      (when (color-at b-dbr white)  (and (color-at b-dbl white)  (not (color-at b-dbr white))))
      (when (color-at b-dbr yellow) (and (color-at b-dbl yellow) (not (color-at b-dbr yellow))))
      (when (color-at b-dbr red)    (and (color-at b-dbl red)    (not (color-at b-dbr red))))
      (when (color-at b-dbr orange) (and (color-at b-dbl orange) (not (color-at b-dbr orange))))
      (when (color-at b-dbr blue)   (and (color-at b-dbl blue)   (not (color-at b-dbr blue))))
      (when (color-at b-dbr green)  (and (color-at b-dbl green)  (not (color-at b-dbr green))))

      (when (color-at b-dbl white)  (and (color-at b-ubl white)  (not (color-at b-dbl white))))
      (when (color-at b-dbl yellow) (and (color-at b-ubl yellow) (not (color-at b-dbl yellow))))
      (when (color-at b-dbl red)    (and (color-at b-ubl red)    (not (color-at b-dbl red))))
      (when (color-at b-dbl orange) (and (color-at b-ubl orange) (not (color-at b-dbl orange))))
      (when (color-at b-dbl blue)   (and (color-at b-ubl blue)   (not (color-at b-dbl blue))))
      (when (color-at b-dbl green)  (and (color-at b-ubl green)  (not (color-at b-dbl green))))

      (when (color-at b-ubl white)  (and (color-at b-ubr white)  (not (color-at b-ubl white))))
      (when (color-at b-ubl yellow) (and (color-at b-ubr yellow) (not (color-at b-ubl yellow))))
      (when (color-at b-ubl red)    (and (color-at b-ubr red)    (not (color-at b-ubl red))))
      (when (color-at b-ubl orange) (and (color-at b-ubr orange) (not (color-at b-ubl orange))))
      (when (color-at b-ubl blue)   (and (color-at b-ubr blue)   (not (color-at b-ubl blue))))
      (when (color-at b-ubl green)  (and (color-at b-ubr green)  (not (color-at b-ubl green))))
    )
  )

)
