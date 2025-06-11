(define (problem stack-three)
  (:domain blocksworld)

  (:objects
    a b c - block
  )

  (:init
    (block a)
    (block b)
    (block c)
    (on-table a)
    (on-table b)
    (on c a)
    (clear b)
    (clear c)
    (handempty)
  )

  (:goal
    (and
      (on c b)
      (on b a)
      (on-table a)
    )
  )
)
