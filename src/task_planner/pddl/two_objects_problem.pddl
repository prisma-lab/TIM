(define (problem red-then-blue)
  (:domain two-object-transfer)
  (:objects
    red_connector blue_peg - item
    home red_pick red_place blue_pick blue_place - location)
  (:init
    (arm-at home)
    (hand-empty)
    (object-at red_connector red_pick)
    (object-at blue_peg blue_pick)
    (destination red_connector red_place)
    (destination blue_peg blue_place)
    ; This relation enforces the requested order, not just the final positions.
    (before red_connector blue_peg))
  (:goal (and
    (object-at red_connector red_place)
    (object-at blue_peg blue_place)
    (arm-at home)
    (hand-empty)))
)
