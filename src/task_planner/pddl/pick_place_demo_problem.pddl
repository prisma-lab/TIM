(define (problem ur10)

  (:domain manipulator-pick-place)

  (:objects
    robot1 - robot

    object1 - object
    object2 - object

    home - location
    loc1 - location
    loc2 - location
    loc3 - location
    loc4 - location
  )

  (:init
    (eeAt robot1 home)

    (objectAt object1 loc1)
    (objectAt object2 loc2)

    (gripperEmpty robot1)

    (clearLoc loc3)
    (clearLoc loc4)
  )

  (:goal
    (and
      (objectAt object1 loc3)
      (eeAt robot1 loc1)
    )
  )
)
