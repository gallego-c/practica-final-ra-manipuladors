;;; ============================================================
;;; PROBLEM: solve-manipulation-sequence
;;; ============================================================
(define (problem solve-manipulation-sequence)
  (:domain robot-manipulation)
  (:objects
    step1 step2 step3 step4 step5 step6 step7 step8 step9 - step
  )
  (:init
    ;; Estado físico inicial del robot y el cubo
    (cube-on-fixture)
    (not (robot-holding))

    ;; Paso inicial de la receta
    (current-step step1)

    (next-step step1 step2)
    (step-type-tilt-x-pos step1)
    (next-step step2 step3)
    (step-type-tilt-y-pos step2)
    (next-step step3 step4)
    (step-type-tilt-y-pos step3)
    (next-step step4 step5)
    (step-type-tilt-x-pos step4)
    (next-step step5 step6)
    (step-type-rotate-cw step5)
    (next-step step6 step7)
    (step-type-tilt-x-pos step6)
    (next-step step7 step8)
    (step-type-tilt-y-pos step7)
    (next-step step8 step9)
    (step-type-rotate-ccw step8)
  )
  (:goal (and
    (step-completed step1)
    (step-completed step2)
    (step-completed step3)
    (step-completed step4)
    (step-completed step5)
    (step-completed step6)
    (step-completed step7)
    (step-completed step8)
    (cube-on-fixture)
    (not (robot-holding))
  ))
)