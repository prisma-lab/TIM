; Hardware services: move takes a destination; pick/place only close/open the gripper.
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
    :parameters (?to - location)
    :precondition (and
      (not (arm-at ?to))
      (exists (?from - location)
        (and (arm-at ?from) (connected ?from ?to))))
    :effect (and
      (forall (?from - location)
        (when (arm-at ?from) (not (arm-at ?from))))
      (arm-at ?to)))

  (:action pick
    :parameters ()
    :precondition (and (hand-empty) (not (placed))
      (exists (?where - location)
        (and (arm-at ?where) (pick-location ?where))))
    :effect (and (holding) (not (hand-empty))))

  (:action place
    :parameters ()
    :precondition (and (holding)
      (exists (?where - location)
        (and (arm-at ?where) (place-location ?where))))
    :effect (and (placed) (hand-empty) (not (holding))))
)
