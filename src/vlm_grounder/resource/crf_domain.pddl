(define (domain battery-pack-assembly)

  (:requirements :strips :typing :adl)

  (:types
    part location - object

    screws conBar conBarProtHoods
    exitConPlate exitCon exitConCaps - part

    conBarLoc conBarScrewsLoc conBarProtHoodsLoc
    exitConPlateLoc exitConPlateScrewsLoc
    exitConLoc exitConCapsLoc - location
  )

  (:predicates
    (available ?p - part)
    (free ?l - location)
    (at ?p - part ?l - location)
    (tightened ?p - part)
    (installedAt ?p - part ?l - location)
  )

  (:action placeConnectionBar
    :parameters (?bar - conBar ?loc - conBarLoc)
    :precondition (and
      (available ?bar) (free ?loc)
    )
    :effect (and
      (at ?bar ?loc)
      (not (free ?loc))
    )
  )

  (:action tightenConnectionBar
    :parameters (
      ?bar - conBar ?loc - conBarLoc
      ?s - screws ?sLoc - conBarScrewsLoc
    )
    :precondition (and
      (available ?s) (free ?sLoc) (at ?bar ?loc)
    )
    :effect (and
      (tightened ?bar)
      (at ?s ?sLoc)
      (not (available ?bar))
      (not (available ?s))
      (not (free ?sLoc))
    )
  )

  (:action placeBarProtHood
    :parameters (
      ?bar - conBar ?loc - conBarLoc
      ?h - conBarProtHoods ?hLoc - conBarProtHoodsLoc
    )
    :precondition (and
      (available ?h) (free ?hLoc) (tightened ?bar)
    )
    :effect (and
      (installedAt ?bar ?loc) (at ?h ?hLoc) (not (available ?h)) (not (free ?hLoc))
    )
  )

  (:action placeExitConnectorPlate
    :parameters (
      ?plate - exitConPlate ?loc - exitConPlateLoc
    )
    :precondition (and
      (available ?plate) (free ?loc)
    )
    :effect (and
      (at ?plate ?loc)
      (not (free ?loc))
    )
  )

  (:action tightenExitConnectorPlate
    :parameters (
      ?plate - exitConPlate ?loc - exitConPlateLoc
      ?s - screws ?sLoc - exitConPlateScrewsLoc
    )
    :precondition (and
      (available ?s) (free ?sLoc) (at ?plate ?loc)
    )
    :effect (and
      (tightened ?plate)
      (at ?s ?sLoc)
      (not (available ?plate))
      (not (available ?s))
      (not (free ?sLoc))
    )
  )

  (:action insertExitConnector
    :parameters (
      ?plate - exitConPlate
      ?connector - exitCon ?loc - exitConLoc
    )
    :precondition (and
      (tightened ?plate) (available ?connector) (free ?loc)
    )
    :effect (and
      (at ?connector ?loc)
      (not (available ?connector))
      (not (free ?loc))
    )
  )

  (:action placeExitConnectorCap
    :parameters (
      ?connector - exitCon ?connectorLoc - exitConLoc
      ?cap - exitConCaps ?capLoc - exitConCapsLoc
    )
    :precondition (and
      (at ?connector ?connectorLoc) (available ?cap) (free ?capLoc)
    )
    :effect (and
      (installedAt ?connector ?connectorLoc)
      (at ?cap ?capLoc)
      (not (available ?cap))
      (not (free ?capLoc))
    )
  )
)