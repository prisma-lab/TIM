(define (problem ur10)

  (:domain manipulator-pick-place)

  (:objects
    robot1 - robot

    red_connector - object
    blue_peg - object

    home - location
    loc1 - location
    loc2 - location
    loc3 - location
    loc4 - location
  )

  (:init
  )

  (:goal
    (and
      (objectAt red_connector loc3)
      (objectAt blue_peg loc4)
    )
  )
)