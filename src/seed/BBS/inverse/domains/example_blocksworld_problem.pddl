(define (problem stack-three)
  (:domain blocksworld)

  (:objects
    brick20 brick21 brick22 - block
    gnd1 gnd2 gnd3 - slot
  )

  (:init
    (on-table brick20 gnd1)
    (on-table brick21 gnd2)
    (on-table brick22 gnd3)
    (clear brick20)
    (clear brick21)
    (clear brick22)
    (handempty)
  )

  (:goal
    (and
      (on brick22 brick21)
      (on brick21 brick20)
      (on-table brick20 gnd1)
    )
  )
)