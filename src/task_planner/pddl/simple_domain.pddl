(define (domain simple)
  (:requirements :strips)
  (:types robot location)
  (:predicates 
    (at ?r - robot ?l - location)
    (adjacent ?l1 ?l2 - location))
  (:action move
    :parameters (?r - robot ?from ?to - location)
    :precondition (and (at ?r ?from) (adjacent ?from ?to))
    :effect (and (not (at ?r ?from)) (at ?r ?to))))
