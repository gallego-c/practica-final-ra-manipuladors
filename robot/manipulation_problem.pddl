;;; ============================================================
;;; PROBLEM: solve-manipulation-sequence
;;; ============================================================
(define (problem solve-manipulation-sequence)
  (:domain robot-manipulation)
  (:objects
    step1 step2 step3 step4 step5 step6 step7 step8 step9 step10 - step
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
    (step-type-D-prime step1)
    (next-step step2 step3)
    (step-type-B2 step2)
    (next-step step3 step4)
    (step-type-R step3)
    (next-step step4 step5)
    (step-type-D2 step4)
    (next-step step5 step6)
    (step-type-B-prime step5)
    (next-step step6 step7)
    (step-type-D2 step6)
    (next-step step7 step8)
    (step-type-B step7)
    (next-step step8 step9)
    (step-type-D2 step8)
    (next-step step9 step10)
    (step-type-R-prime step9)
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
    (step-completed step9)
  ))
)