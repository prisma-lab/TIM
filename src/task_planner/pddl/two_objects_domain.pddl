(define (domain two-object-transfer)
  (:requirements :adl :typing)
  (:types item location)
  (:predicates
    (arm-at ?where - location)
    (object-at ?object - item ?where - location)
    (holding ?object - item)
    (hand-empty)
    (destination ?object - item ?where - location)
    (before ?earlier ?later - item)
    (completed ?object - item))

  (:action move_a_b
    :parameters (?from ?to - location)
    :precondition (arm-at ?from)
    :effect (and (not (arm-at ?from)) (arm-at ?to)))

  (:action pick
    :parameters (?object - item ?where - location)
    :precondition (and
      (arm-at ?where) (object-at ?object ?where) (hand-empty)
      ; Every predecessor must have been placed before this pick can start.
      (forall (?earlier - item)
        (imply (before ?earlier ?object) (completed ?earlier))))
    :effect (and (holding ?object) (not (hand-empty))
                 (not (object-at ?object ?where))))

  (:action place
    :parameters (?object - item ?where - location)
    :precondition (and (arm-at ?where) (holding ?object)
                       (destination ?object ?where))
    :effect (and (object-at ?object ?where) (hand-empty)
                 (not (holding ?object)) (completed ?object)))
)
