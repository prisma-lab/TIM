(define (problem assemble-battery-pack)
  (:domain battery-pack-assembly)

  (:objects
    bar - conBar
    hoods - conBarProtHoods
    plate - exitConPlate
    connectors - exitCon
    caps - exitConCaps

    screws1 screws2 - screws

    barLoc - conBarLoc
    barScrewsLoc - conBarScrewsLoc
    hoodsLoc - conBarProtHoodsLoc
    plateLoc - exitConPlateLoc
    plateScrewsLoc - exitConPlateScrewsLoc
    connectorsLoc - exitConLoc
    capsLoc - exitConCapsLoc
  )

  (:init

    (available plate)
    (available connectors)
    (available caps)
    (available screws2)

    (free plateLoc)
    (free plateScrewsLoc)
    (free connectorsLoc)
    (free capsLoc)
  )

  (:goal (and
    (installedAt bar barLoc)
    (installedAt connectors connectorsLoc)
  ))
)