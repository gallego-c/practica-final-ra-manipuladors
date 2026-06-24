;;; ============================================================
;;; DOMAIN: robot-manipulation
;;; ============================================================
;;; Advanced TAMP domain where the high-level Rubik's cube solver
;;; is agnostic, and Fast Downward plans the physical tilts and grasps
;;; to bring the correct face of the cube to the top for wrist rotation.
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
  ;; TILT_X (Left→Top, Top→Right, Right→Bottom, Bottom→Left)
  ;; -----------------------------------------------------------
  (:action tilt_x
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

  ;; -----------------------------------------------------------
  ;; TILT_Y (Front→Top, Top→Back, Back→Bottom, Bottom→Front)
  ;; -----------------------------------------------------------
  (:action tilt_y
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

  ;; ===========================================================
  ;; ROBOT OPERATIONS CORRESPONDING TO STANDARD CUBE MOVEMENTS
  ;; ===========================================================

  (:action execute_U
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_u) (or (step-type-U ?s) (step-type-D ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-D ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-D ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-D ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-D ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_U_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_u) (or (step-type-U-prime ?s) (step-type-D-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-D-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-D-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-D-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-D-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_U2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_u) (or (step-type-U2 ?s) (step-type-D2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-D2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-D2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-D2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-D2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_D
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_d) (or (step-type-D ?s) (step-type-U ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-U ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-U ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-U ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-U ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_D_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_d) (or (step-type-D-prime ?s) (step-type-U-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-U-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-U-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-U-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-U-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_D2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_d) (or (step-type-D2 ?s) (step-type-U2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-U2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-U2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-U2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-U2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_R
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_r) (or (step-type-R ?s) (step-type-L ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-L ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-L ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-L ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-L ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_R_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_r) (or (step-type-R-prime ?s) (step-type-L-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-L-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-L-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-L-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-L-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_R2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_r) (or (step-type-R2 ?s) (step-type-L2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-L2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-L2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-L2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-L2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_L
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_l) (or (step-type-L ?s) (step-type-R ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-R ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-R ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-R ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-R ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_L_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_l) (or (step-type-L-prime ?s) (step-type-R-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-R-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-R-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-R-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-R-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_L2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_l) (or (step-type-L2 ?s) (step-type-R2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-R2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-R2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-R2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-R2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_F
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_f) (or (step-type-F ?s) (step-type-B ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-B ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-B ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-B ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-B ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_F_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_f) (or (step-type-F-prime ?s) (step-type-B-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-B-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-B-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-B-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-B-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_F2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_f) (or (step-type-F2 ?s) (step-type-B2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-B2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-B2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-B2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-B2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_B
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_b) (or (step-type-B ?s) (step-type-F ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-F ?s) (front-face ?f)) (and (left-face ?f) (not (front-face ?f))))
                     (when (and (step-type-F ?s) (left-face ?f)) (and (back-face ?f) (not (left-face ?f))))
                     (when (and (step-type-F ?s) (back-face ?f)) (and (right-face ?f) (not (back-face ?f))))
                     (when (and (step-type-F ?s) (right-face ?f)) (and (front-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )

  (:action execute_B_prime
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_b) (or (step-type-B-prime ?s) (step-type-F-prime ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (when (holding-x) (and (not (holding-x)) (holding-y)))
                 (when (holding-y) (and (not (holding-y)) (holding-x)))
                 (forall (?f - face)
                   (and
                     (when (and (step-type-F-prime ?s) (front-face ?f)) (and (right-face ?f) (not (front-face ?f))))
                     (when (and (step-type-F-prime ?s) (right-face ?f)) (and (back-face ?f) (not (right-face ?f))))
                     (when (and (step-type-F-prime ?s) (back-face ?f)) (and (left-face ?f) (not (back-face ?f))))
                     (when (and (step-type-F-prime ?s) (left-face ?f)) (and (front-face ?f) (not (left-face ?f))))
                   )
                 )
            )
  )

  (:action execute_B2
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (next-step ?s ?next) (top-face face_b) (or (step-type-B2 ?s) (step-type-F2 ?s)))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next)
                 (forall (?f - face)
                   (and
                     (when (and (step-type-F2 ?s) (front-face ?f)) (and (back-face ?f) (not (front-face ?f))))
                     (when (and (step-type-F2 ?s) (back-face ?f)) (and (front-face ?f) (not (back-face ?f))))
                     (when (and (step-type-F2 ?s) (left-face ?f)) (and (right-face ?f) (not (left-face ?f))))
                     (when (and (step-type-F2 ?s) (right-face ?f)) (and (left-face ?f) (not (right-face ?f))))
                   )
                 )
            )
  )
)
