;;; ============================================================
;;; PROBLEM: solve-manipulation-sequence
;;; ============================================================
(define (problem solve-manipulation-sequence)
  (:domain robot-manipulation)
  (:objects
    step1 step2 step3 step4 step5 - step
  )
  (:init
    ;; Estado físico inicial del robot y el cubo
    (cube-on-fixture)
    (not (robot-holding))

    ;; Orientación 3D inicial del cubo en la base
    (top-face face_u)
    (bottom-face face_d)
    (front-face face_f)
    (back-face face_b)
    (left-face face_l)
    (right-face face_r)

    ;; Paso inicial de la receta
    (current-step step1)

    (next-step step1 step2)
    (step-type-R step1)
    (next-step step2 step3)
    (step-type-F step2)
    (next-step step3 step4)
    (step-type-R step3)
    (next-step step4 step5)
    (step-type-U step4)
  )
  (:goal (and
    (step-completed step1)
    (step-completed step2)
    (step-completed step3)
    (step-completed step4)
    (cube-on-fixture)
    (not (robot-holding))
  ))
)