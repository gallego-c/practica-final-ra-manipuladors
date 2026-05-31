;;; ============================================================
;;; DOMAIN: robot-manipulation
;;; ============================================================
;;; Simplified domain that plans physical robot actions (pick, place, rotate, tilt)
;;; to execute a high-level recipe of Rubik's cube steps sequentially.
;;; ============================================================

(define (domain robot-manipulation)
  (:requirements :strips :typing)
  (:types step)
  (:predicates
    (robot-holding)
    (cube-on-fixture)
    (current-step ?s - step)
    (next-step ?s1 ?s2 - step)
    (step-completed ?s - step)
    (step-type-rotate-cw ?s - step)
    (step-type-rotate-ccw ?s - step)
    (step-type-tilt-x-pos ?s - step)
    (step-type-tilt-x-neg ?s - step)
    (step-type-tilt-y-pos ?s - step)
    (step-type-tilt-y-neg ?s - step)
  )

  ;; -----------------------------------------------------------
  ;; PICK – lift cube off fixture (requires cube on fixture)
  ;; -----------------------------------------------------------
  (:action pick
    :parameters ()
    :precondition (and (cube-on-fixture) (not (robot-holding)))
    :effect (and (robot-holding) (not (cube-on-fixture)))
  )

  ;; -----------------------------------------------------------
  ;; PLACE – set cube back on fixture (requires holding cube)
  ;; -----------------------------------------------------------
  (:action place
    :parameters ()
    :precondition (robot-holding)
    :effect (and (cube-on-fixture) (not (robot-holding)))
  )

  ;; -----------------------------------------------------------
  ;; ROTATE_TOP_CW – requires robot holding the cube
  ;; -----------------------------------------------------------
  (:action execute_rotate_top_cw
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-rotate-cw ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  ;; -----------------------------------------------------------
  ;; ROTATE_TOP_CCW – requires robot holding the cube
  ;; -----------------------------------------------------------
  (:action execute_rotate_top_ccw
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (robot-holding) (step-type-rotate-ccw ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  ;; -----------------------------------------------------------
  ;; TILT_X_POS – requires cube on fixture (robot not holding)
  ;; -----------------------------------------------------------
  (:action execute_tilt_x_pos
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (cube-on-fixture) (not (robot-holding)) (step-type-tilt-x-pos ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  ;; -----------------------------------------------------------
  ;; TILT_X_NEG – requires cube on fixture (robot not holding)
  ;; -----------------------------------------------------------
  (:action execute_tilt_x_neg
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (cube-on-fixture) (not (robot-holding)) (step-type-tilt-x-neg ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  ;; -----------------------------------------------------------
  ;; TILT_Y_POS – requires cube on fixture (robot not holding)
  ;; -----------------------------------------------------------
  (:action execute_tilt_y_pos
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (cube-on-fixture) (not (robot-holding)) (step-type-tilt-y-pos ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )

  ;; -----------------------------------------------------------
  ;; TILT_Y_NEG – requires cube on fixture (robot not holding)
  ;; -----------------------------------------------------------
  (:action execute_tilt_y_neg
    :parameters (?s - step ?next - step)
    :precondition (and (current-step ?s) (cube-on-fixture) (not (robot-holding)) (step-type-tilt-y-neg ?s) (next-step ?s ?next))
    :effect (and (step-completed ?s) (not (current-step ?s)) (current-step ?next))
  )
)
