(define (problem stack-three)
  (:domain blocksworld)

  (:objects
    brick10 brick11 brick21 - block
    gnd1 gnd2 gnd3 - slot
  )

  (:init
    (on-table brick10 gnd1)
    (on-table brick11 gnd2)
    (on brick21 brick10)
    (clear brick11)
    (clear brick21)
    (clear gnd3)
    (handempty)
  )

  (:goal
    (and
      (on brick21 brick11)
      (on brick11 brick10)
      (on-table brick10 gnd3)
    )
  )
)