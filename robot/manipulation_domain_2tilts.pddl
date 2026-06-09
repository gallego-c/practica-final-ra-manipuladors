;;; ============================================================
;;; DOMAIN: robot-manipulation (2-Tilts Variant)
;;; ============================================================
;;; Advanced TAMP domain where the high-level Rubik's cube solver
;;; is agnostic, and Fast Downward plans the physical tilts and grasps
;;; to bring the correct face of the cube to the top for wrist rotation.
;;; This version uses only 2 tilt actions (tilt_x_pos and tilt_y_neg).
;;; ============================================================

(define (domain robot-manipulation)
  (:requirements :strips :typing :conditional-effects)
  (:types step face)
  
  (:constants
    face_u face_d face_f face_b face_l face_r - face
  )

  (:predicates
    (robot-holding)
    (holding-x)
    (holding-y)
    (cube-on-fixture)
    (current-step ?s - step)
    (next-step ?s1 ?s2 - step)
    (step-completed ?s - step)
    
    ;; 3D Orientation of the cube in the fixture
    (top-face ?f - face)
    (bottom-face ?f - face)
    (front-face ?f - face)
    (back-face ?f - face)
    (left-face ?f - face)
    (right-face ?f - face)
    
    ;; Types of standard Rubik moves for each step
    (step-type-U ?s - step)
    (step-type-U-prime ?s - step)
    (step-type-U2 ?s - step)
    (step-type-D ?s - step)
    (step-type-D-prime ?s - step)
    (step-type-D2 ?s - step)
    (step-type-R ?s - step)
    (step-type-R-prime ?s - step)
    (step-type-R2 ?s - step)
    (step-type-L ?s - step)
    (step-type-L-prime ?s - step)
    (step-type-L2 ?s - step)
    (step-type-F ?s - step)
    (step-type-F-prime ?s - step)
    (step-type-F2 ?s - step)
    (step-type-B ?s - step)
    (step-type-B-prime ?s - step)
    (step-type-B2 ?s - step)
  )

  ;; -----------------------------------------------------------
  ;; PICK_X – lift cube off fixture squeezing along X axis
  ;; -----------------------------------------------------------
  (:action pick_x
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and (robot-holding) (holding-x) (not (cube-on-fixture)))
  )

  ;; -----------------------------------------------------------
  ;; PICK_Y – lift cube off fixture squeezing along Y axis
  ;; -----------------------------------------------------------
  (:action pick_y
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and (robot-holding) (holding-y) (not (cube-on-fixture)))
  )

  ;; -----------------------------------------------------------
  ;; PLACE – set cube back on fixture (requires holding cube)
  ;; -----------------------------------------------------------
  (:action place
    :parameters ()
    :precondition (robot-holding)
    :effect (and (cube-on-fixture) (not (robot-holding)) (not (holding-x)) (not (holding-y)))
  )

  ;; -----------------------------------------------------------
  ;; CHANGE_PICK – place on fixture, rotate gripper 90°, and regrasp
  ;; -----------------------------------------------------------
  (:action change_pick
    :parameters ()
    :precondition (robot-holding)
    :effect (and
      (when (holding-x) (and (not (holding-x)) (holding-y)))
      (when (holding-y) (and (not (holding-y)) (holding-x)))
    )
  )

  ;; -----------------------------------------------------------
  ;; TILT_X_POS (+90° around X: Front→Top, Down→Front, Back→Down, Top→Back)
  ;; -----------------------------------------------------------
  (:action tilt_x_pos
    :parameters ()
    :precondition (holding-y)
    :effect (and
      (cube-on-fixture)
      (not (robot-holding))
      (not (holding-y))
      (forall (?f - face)
        (and
          (when (front-face ?f) (and (top-face ?f) (not (front-face ?f))))
          (when (top-face ?f) (and (back-face ?f) (not (top-face ?f))))
          (when (back-face ?f) (and (bottom-face ?f) (not (back-face ?f))))
          (when (bottom-face ?f) (and (front-face ?f) (not (bottom-face ?f))))
        )
      )
    )
  )

  ;; -----------------------------------------------------------
  ;; TILT_Y_NEG (-90° around Y: Left→Top, Top→Right, Right→Down, Down→Left)
  ;; -----------------------------------------------------------
  (:action tilt_y_neg
    :parameters ()
    :precondition (holding-x)
    :effect (and
      (cube-on-fixture)
      (not (robot-holding))
      (not (holding-x))
      (forall (?f - face)
        (and
          (when (left-face ?f) (and (top-face ?f) (not (left-face ?f))))
          (when (top-face ?f) (and (right-face ?f) (not (top-face ?f))))
          (when (right-face ?f) (and (bottom-face ?f) (not (right-face ?f))))
          (when (bottom-face ?f) (and (left-face ?f) (not (bottom-face ?f))))
        )
      )
    )
  )

  ;; ===========================================================
  ;; ROBOT OPERATIONS CORRESPONDING TO STANDARD CUBE MOVEMENTS
  ;; ===========================================================

  (:action execute_U
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-U ?s) (next-step ?s ?next) (top-face face_u))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_U_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-U-prime ?s) (next-step ?s ?next) (top-face face_u))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_U2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-U2 ?s) (next-step ?s ?next) (top-face face_u))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  (:action execute_D
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-D ?s) (next-step ?s ?next) (top-face face_d))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_D_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-D-prime ?s) (next-step ?s ?next) (top-face face_d))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_D2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-D2 ?s) (next-step ?s ?next) (top-face face_d))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  (:action execute_R
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-R ?s) (next-step ?s ?next) (top-face face_r))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_R_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-R-prime ?s) (next-step ?s ?next) (top-face face_r))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_R2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-R2 ?s) (next-step ?s ?next) (top-face face_r))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  (:action execute_L
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-L ?s) (next-step ?s ?next) (top-face face_l))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_L_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-L-prime ?s) (next-step ?s ?next) (top-face face_l))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_L2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-L2 ?s) (next-step ?s ?next) (top-face face_l))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  (:action execute_F
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-F ?s) (next-step ?s ?next) (top-face face_f))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_F_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-F-prime ?s) (next-step ?s ?next) (top-face face_f))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_F2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-F2 ?s) (next-step ?s ?next) (top-face face_f))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  (:action execute_B
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-B ?s) (next-step ?s ?next) (top-face face_b))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_B_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-B-prime ?s) (next-step ?s ?next) (top-face face_b))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
            )
  )

  (:action execute_B2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-B2 ?s) (next-step ?s ?next) (top-face face_b))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )
)
