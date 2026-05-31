;;; ============================================================
;;; PROBLEM: rubik-robot-scrambled
;;; ============================================================
(define (problem rubik-robot-scrambled)
  (:domain rubik-robot)
  (:objects
    white yellow red orange blue green - color
    u-ufr u-ufl u-ubr u-ubl d-dfr d-dfl d-dbr d-dbl f-ufr f-ufl f-dfr f-dfl b-ubr b-ubl b-dbr b-dbl l-ufl l-ubl l-dfl l-dbl r-ufr r-ubr r-dfr r-dbr - position
  )
  (:init
    (cube-on-fixture)
    (color-at u-ufr white)
    (color-at u-ufl red)
    (color-at u-ubr red)
    (color-at u-ubl orange)
    (color-at d-dfr white)
    (color-at d-dfl yellow)
    (color-at d-dbr red)
    (color-at d-dbl blue)
    (color-at f-ufr orange)
    (color-at f-ufl white)
    (color-at f-dfr orange)
    (color-at f-dfl red)
    (color-at b-ubr blue)
    (color-at b-ubl yellow)
    (color-at b-dbr white)
    (color-at b-dbl orange)
    (color-at l-ufl blue)
    (color-at l-ubl green)
    (color-at l-dfl green)
    (color-at l-dbl yellow)
    (color-at r-ufr green)
    (color-at r-ubr yellow)
    (color-at r-dfr blue)
    (color-at r-dbr green)
  )
  (:goal (and
    (cube-on-fixture)
    ;; Face U must be monochromatic
    (or
      (and (color-at u-ufr white) (color-at u-ufl white) (color-at u-ubr white) (color-at u-ubl white))
      (and (color-at u-ufr yellow) (color-at u-ufl yellow) (color-at u-ubr yellow) (color-at u-ubl yellow))
      (and (color-at u-ufr red) (color-at u-ufl red) (color-at u-ubr red) (color-at u-ubl red))
      (and (color-at u-ufr orange) (color-at u-ufl orange) (color-at u-ubr orange) (color-at u-ubl orange))
      (and (color-at u-ufr blue) (color-at u-ufl blue) (color-at u-ubr blue) (color-at u-ubl blue))
      (and (color-at u-ufr green) (color-at u-ufl green) (color-at u-ubr green) (color-at u-ubl green))
    )
    ;; Face D must be monochromatic
    (or
      (and (color-at d-dfr white) (color-at d-dfl white) (color-at d-dbr white) (color-at d-dbl white))
      (and (color-at d-dfr yellow) (color-at d-dfl yellow) (color-at d-dbr yellow) (color-at d-dbl yellow))
      (and (color-at d-dfr red) (color-at d-dfl red) (color-at d-dbr red) (color-at d-dbl red))
      (and (color-at d-dfr orange) (color-at d-dfl orange) (color-at d-dbr orange) (color-at d-dbl orange))
      (and (color-at d-dfr blue) (color-at d-dfl blue) (color-at d-dbr blue) (color-at d-dbl blue))
      (and (color-at d-dfr green) (color-at d-dfl green) (color-at d-dbr green) (color-at d-dbl green))
    )
    ;; Face F must be monochromatic
    (or
      (and (color-at f-ufr white) (color-at f-ufl white) (color-at f-dfr white) (color-at f-dfl white))
      (and (color-at f-ufr yellow) (color-at f-ufl yellow) (color-at f-dfr yellow) (color-at f-dfl yellow))
      (and (color-at f-ufr red) (color-at f-ufl red) (color-at f-dfr red) (color-at f-dfl red))
      (and (color-at f-ufr orange) (color-at f-ufl orange) (color-at f-dfr orange) (color-at f-dfl orange))
      (and (color-at f-ufr blue) (color-at f-ufl blue) (color-at f-dfr blue) (color-at f-dfl blue))
      (and (color-at f-ufr green) (color-at f-ufl green) (color-at f-dfr green) (color-at f-dfl green))
    )
    ;; Face B must be monochromatic
    (or
      (and (color-at b-ubr white) (color-at b-ubl white) (color-at b-dbr white) (color-at b-dbl white))
      (and (color-at b-ubr yellow) (color-at b-ubl yellow) (color-at b-dbr yellow) (color-at b-dbl yellow))
      (and (color-at b-ubr red) (color-at b-ubl red) (color-at b-dbr red) (color-at b-dbl red))
      (and (color-at b-ubr orange) (color-at b-ubl orange) (color-at b-dbr orange) (color-at b-dbl orange))
      (and (color-at b-ubr blue) (color-at b-ubl blue) (color-at b-dbr blue) (color-at b-dbl blue))
      (and (color-at b-ubr green) (color-at b-ubl green) (color-at b-dbr green) (color-at b-dbl green))
    )
    ;; Face L must be monochromatic
    (or
      (and (color-at l-ufl white) (color-at l-ubl white) (color-at l-dfl white) (color-at l-dbl white))
      (and (color-at l-ufl yellow) (color-at l-ubl yellow) (color-at l-dfl yellow) (color-at l-dbl yellow))
      (and (color-at l-ufl red) (color-at l-ubl red) (color-at l-dfl red) (color-at l-dbl red))
      (and (color-at l-ufl orange) (color-at l-ubl orange) (color-at l-dfl orange) (color-at l-dbl orange))
      (and (color-at l-ufl blue) (color-at l-ubl blue) (color-at l-dfl blue) (color-at l-dbl blue))
      (and (color-at l-ufl green) (color-at l-ubl green) (color-at l-dfl green) (color-at l-dbl green))
    )
    ;; Face R must be monochromatic
    (or
      (and (color-at r-ufr white) (color-at r-ubr white) (color-at r-dfr white) (color-at r-dbr white))
      (and (color-at r-ufr yellow) (color-at r-ubr yellow) (color-at r-dfr yellow) (color-at r-dbr yellow))
      (and (color-at r-ufr red) (color-at r-ubr red) (color-at r-dfr red) (color-at r-dbr red))
      (and (color-at r-ufr orange) (color-at r-ubr orange) (color-at r-dfr orange) (color-at r-dbr orange))
      (and (color-at r-ufr blue) (color-at r-ubr blue) (color-at r-dfr blue) (color-at r-dbr blue))
      (and (color-at r-ufr green) (color-at r-ubr green) (color-at r-dfr green) (color-at r-dbr green))
    )
  ))
)