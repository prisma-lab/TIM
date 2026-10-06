; Hardware services: move takes a source and destination; pick/place only close/open the gripper.
(define (domain hardware-pick-place)
  (:requirements :adl :typing)
  (:types location)
  (:predicates
    (arm-at ?where - location)
    (connected ?from ?to - location)
    (pick-location ?where - location)
    (place-location ?where - location)
    (hand-empty)
    (holding)
    (placed))

  (:action move
    :parameters (?from ?to - location)
    :precondition (and
      (arm-at ?from)
      (not (arm-at ?to))
      (connected ?from ?to))
    :effect (and
      (not (arm-at ?from))
      (arm-at ?to)))

  (:action pick
    :parameters (?where - location)
    :precondition (and (hand-empty) (not (placed))
      (arm-at ?where)
      (pick-location ?where))
    :effect (and (holding) (not (hand-empty))))

  (:action place
    :parameters (?where - location)
    :precondition (and (holding)
      (arm-at ?where)
      (place-location ?where))
    :effect (and (placed) (hand-empty) (not (holding))))
)
