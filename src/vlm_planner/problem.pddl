(define (problem clip-to-target)
  (:domain clip-assembly)
  (:objects
    clip          - object
    source target - location
    hand          - gripper)
  (:init
    (partially-hidden clip source)
    (gripper-empty hand)
    (gripper-at hand source))
  (:goal (at-target clip target)))