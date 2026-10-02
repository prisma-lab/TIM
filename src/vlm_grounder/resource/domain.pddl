(define (domain manipulator-pick-place)

  (:requirements
    :strips
    :typing
  )

  (:types
    robot
    object
    location
  )

  (:predicates
    (eeAt ?r - robot ?l - location)
    (objectAt ?o - object ?l - location)
    (gripperHolding ?r - robot ?o - object)
    (gripperEmpty ?r - robot)
    (clearLoc ?l - location)
  )

  (:action move_a_b
    :parameters (
      ?r - robot
      ?from - location
      ?to - location
    )

    :precondition (and
      (eeAt ?r ?from)
    )

    :effect (and
      (eeAt ?r ?to)
      (not (eeAt ?r ?from))
    )
  )

  (:action pick
    :parameters (
      ?r - robot
      ?o - object
      ?l - location
    )

    :precondition (and
      (eeAt ?r ?l)
      (objectAt ?o ?l)
      (gripperEmpty ?r)
    )

    :effect (and
      (gripperHolding ?r ?o)
      (clearLoc ?l)
      (not (objectAt ?o ?l))
      (not (gripperEmpty ?r))
    )
  )

  (:action place
    :parameters (
      ?r - robot
      ?o - object
      ?l - location
    )

    :precondition (and
      (eeAt ?r ?l)
      (gripperHolding ?r ?o)
      (clearLoc ?l)
    )

    :effect (and
      (objectAt ?o ?l)
      (gripperEmpty ?r)
      (not (gripperHolding ?r ?o))
      (not (clearLoc ?l))
    )
  )
)