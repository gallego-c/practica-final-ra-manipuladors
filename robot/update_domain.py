#!/usr/bin/env python3
import os
import re

domain_path = "/home/barrendeiro/robotica/cub/robot/manipulation_domain.pddl"

with open(domain_path, 'r') as f:
    content = f.read()

# Define the action replacements
replacements = {
    # execute_U
    r'\(:action execute_U\s+.*?execute_U_prime': """(:action execute_U
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

  (:action execute_U_prime""",

    # execute_U_prime
    r'\(:action execute_U_prime\s+.*?execute_U2': """(:action execute_U_prime
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

  (:action execute_U2""",

    # execute_D
    r'\(:action execute_D\s+.*?execute_D_prime': """(:action execute_D
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

  (:action execute_D_prime""",

    # execute_D_prime
    r'\(:action execute_D_prime\s+.*?execute_D2': """(:action execute_D_prime
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

  (:action execute_D2""",

    # execute_R
    r'\(:action execute_R\s+.*?execute_R_prime': """(:action execute_R
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

  (:action execute_R_prime""",

    # execute_R_prime
    r'\(:action execute_R_prime\s+.*?execute_R2': """(:action execute_R_prime
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

  (:action execute_R2""",

    # execute_L
    r'\(:action execute_L\s+.*?execute_L_prime': """(:action execute_L
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

  (:action execute_L_prime""",

    # execute_L_prime
    r'\(:action execute_L_prime\s+.*?execute_L2': """(:action execute_L_prime
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

  (:action execute_L2""",

    # execute_F
    r'\(:action execute_F\s+.*?execute_F_prime': """(:action execute_F
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

  (:action execute_F_prime""",

    # execute_F_prime
    r'\(:action execute_F_prime\s+.*?execute_F2': """(:action execute_F_prime
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

  (:action execute_F2""",

    # execute_B
    r'\(:action execute_B\s+.*?execute_B_prime': """(:action execute_B
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

  (:action execute_B_prime""",

    # execute_B_prime
    r'\(:action execute_B_prime\s+.*?execute_B2': """(:action execute_B_prime
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

  (:action execute_B2""",
}

# Apply replacements
for pattern, replacement in replacements.items():
    content, count = re.subn(pattern, replacement, content, flags=re.DOTALL)
    print(f"Replaced {pattern}: {count} occurrences")

# Write back
with open(domain_path, 'w') as f:
    f.write(content)

print("Domain file successfully updated!")
