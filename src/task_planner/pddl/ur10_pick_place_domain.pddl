(define (domain ur10-pick-place)
  (:requirements :strips :typing)
  (:types item location)
  (:predicates
    (arm-at ?loc - location)
    (connected ?from ?to - location)
    (object-at ?obj - item ?loc - location)
    (holding ?obj - item)
    (hand-empty)
    (pick-location ?loc - location)
    (place-location ?loc - location))

  (:action move-a-b
    :parameters (?from ?to - location)
    :precondition (and (arm-at ?from) (connected ?from ?to))
    :effect (and (not (arm-at ?from)) (arm-at ?to)))

  (:action pick
    :parameters (?obj - item ?loc - location)
    :precondition (and (arm-at ?loc) (pick-location ?loc)
                       (object-at ?obj ?loc) (hand-empty))
    :effect (and (holding ?obj) (not (object-at ?obj ?loc)) (not (hand-empty))))

  (:action place
    :parameters (?obj - item ?loc - location)
    :precondition (and (arm-at ?loc) (place-location ?loc) (holding ?obj))
    :effect (and (object-at ?obj ?loc) (hand-empty) (not (holding ?obj)))))
