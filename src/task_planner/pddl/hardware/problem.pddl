; Example symbolic starting state. Replace it with the colleagues' real task.
; No coordinates are defined here: an external publisher supplies the named poses.
(define (problem hardware-transfer)
  (:domain hardware-pick-place)
  (:objects home pick_approach pick place_approach place - location)
  (:init
    (arm-at home)
    (hand-empty)
    (pick-location pick)
    (place-location place)
    (connected home pick_approach)
    (connected pick_approach pick)
    (connected pick pick_approach)
    (connected pick_approach place_approach)
    (connected place_approach place)
    (connected place place_approach))
  (:goal (placed))
)
