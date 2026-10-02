(define (problem battery-pack-assembly-example)
  (:domain battery-pack-assembly)

  (:objects
    bar1 - conBar
    screw1 screw2 - screw
    hood1 hood2 - conBarProtHood

    barLoc1 - conBarLoc
    screwLoc1 screwLoc2 - conBarScrewsLoc
    hoodLoc1 hoodLoc2 - conBarProtHoodsLoc
  )

  (:init
    (connected screwLoc1 hoodLoc1)
    (connected screwLoc2 hoodLoc2)

  )

  (:goal
    (installedAt bar1 barLoc1)
  )
)