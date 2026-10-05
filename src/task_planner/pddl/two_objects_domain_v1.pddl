(define (domain manipulator-pick-place)

  (:requirements
    :strips
    :typing
  )

  (:types
    object
    location
  )

  (:predicates
    (eeAt ?l - location)
    (objectAt ?o - object ?l - location)
    (gripperHolding ?o - object)
    (gripperEmpty)
    (clearLoc ?l - location)
  )

  (:action move_a_b
    :parameters (
      ?from - location
      ?to - location
    )

    :precondition (and
      (eeAt ?from)
    )

    :effect (and
      (eeAt ?to)
      (not (eeAt ?from))
    )
  )

  (:action pick
    :parameters (
      ?o - object
      ?l - location
    )

    :precondition (and
      (eeAt ?l)
      (objectAt ?o ?l)
      (gripperEmpty )
    )

    :effect (and
      (gripperHolding ?o)
      (clearLoc ?l)
      (not (objectAt ?o ?l))
      (not (gripperEmpty))
    )
  )

  (:action place
    :parameters (
      ?o - object
      ?l - location
    )

    :precondition (and
      (eeAt ?l)
      (gripperHolding ?o)
      (clearLoc ?l)
    )

    :effect (and
      (objectAt ?o ?l)
      (gripperEmpty)
      (not (gripperHolding ?o))
      (not (clearLoc ?l))
    )
  )
)