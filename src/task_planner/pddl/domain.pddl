(define (domain simple)
  (:requirements :strips)
  (:predicates (at ?x) (moveable ?x))
  (:action move
    :parameters (?obj)
    :precondition (and (moveable ?obj) (at ?obj))
    :effect (and (not (at ?obj))))
)