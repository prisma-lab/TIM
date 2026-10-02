(define (problem generated-problem)
  (:domain manipulator-pick-place)
  (:objects
    robot - robot
    blue_object red_object - object
    home blue_initial red_initial right_corner left_corner - location
  )
  (:init
    (eeAt robot home)
    (objectAt blue_object blue_initial)
    (objectAt red_object red_initial)
    (gripperEmpty robot)
    (clearLoc home)
    (clearLoc right_corner)
    (clearLoc left_corner)
  )
  (:goal (and
    (objectAt blue_object right_corner)
  ))
)
