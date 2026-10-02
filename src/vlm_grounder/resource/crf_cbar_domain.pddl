(define (domain battery-pack-assembly)

  (:requirements :strips :typing :adl :derived-predicates)

  (:types
    part location - object
    screw conBar conBarProtHood - part
    conBarLoc conBarScrewsLoc conBarProtHoodsLoc - location
  )

  (:predicates
    (available ?p - part)
    (free ?l - location)
    (at ?p - part ?l - location)
    (tightened ?p - part)
    (installedAt ?bar - conBar ?loc - conBarLoc)

    (connected
      ?screwLoc - conBarScrewsLoc
      ?hoodLoc - conBarProtHoodsLoc
    )
  )

  (:derived (installedAt ?bar - conBar ?loc - conBarLoc)
    (exists (
      ?s1 ?s2 - screw
      ?h1 ?h2 - conBarProtHood
      ?sl1 ?sl2 - conBarScrewsLoc
      ?hl1 ?hl2 - conBarProtHoodsLoc
    )
      (and
        (at ?bar ?loc)

        (not (= ?s1 ?s2))
        (not (= ?sl1 ?sl2))
        (at ?s1 ?sl1)
        (at ?s2 ?sl2)
        (tightened ?s1)
        (tightened ?s2)

        (not (= ?h1 ?h2))
        (not (= ?hl1 ?hl2))
        (connected ?sl1 ?hl1)
        (connected ?sl2 ?hl2)
        (at ?h1 ?hl1)
        (at ?h2 ?hl2)
      )
    )
  )

  (:action placeBar
    :parameters (
      ?bar - conBar
      ?loc - conBarLoc
    )
    :precondition (and
      (available ?bar)
      (free ?loc)
    )
    :effect (and
      (at ?bar ?loc)
      (not (available ?bar))
      (not (free ?loc))
    )
  )

  (:action placeScrew
    :parameters (
      ?screw - screw
      ?loc - conBarScrewsLoc
      ?bar - conBar
      ?barLoc - conBarLoc
    )
    :precondition (and
      (available ?screw)
      (free ?loc)
      (at ?bar ?barLoc)
    )
    :effect (and
      (at ?screw ?loc)
      (not (available ?screw))
      (not (free ?loc))
    )
  )

  (:action tightenScrew
    :parameters (
      ?screw - screw
      ?loc - conBarScrewsLoc
    )
    :precondition (and
      (at ?screw ?loc)
      (not (tightened ?screw))
    )
    :effect (tightened ?screw)
  )

  (:action placeHood
    :parameters (
      ?hood - conBarProtHood
      ?hoodLoc - conBarProtHoodsLoc
      ?screw - screw
      ?screwLoc - conBarScrewsLoc
    )
    :precondition (and
      (available ?hood)
      (free ?hoodLoc)
      (connected ?screwLoc ?hoodLoc)
      (at ?screw ?screwLoc)
      (tightened ?screw)
    )
    :effect (and
      (at ?hood ?hoodLoc)
      (not (available ?hood))
      (not (free ?hoodLoc))
    )
  )
)