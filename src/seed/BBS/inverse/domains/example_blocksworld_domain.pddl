(define (domain blocksworld)
  (:requirements :strips :typing)
  
  (:types block)

  (:predicates
    (block ?x - block)
    (clear ?x - block)
    (on ?x - block ?y - block)
    (on-table ?x - block)
    (holding ?x - block)
    (handempty)
  )

  ;; Action: pickup from table
  (:action pickup
    :parameters (?x - block)
    :precondition (and
      (handempty)
      (block ?x)
      (clear ?x)
      (on-table ?x)
    )
    :effect (and
      (not (handempty))
      (not (clear ?x))
      (not (on-table ?x))
      (holding ?x)
    )
  )

  ;; Action: put block on table
  (:action putdown
    :parameters (?x - block)
    :precondition (holding ?x)
    :effect (and
      (handempty)
      (clear ?x)
      (on-table ?x)
      (not (holding ?x))
    )
  )

  ;; Action: unstack from block
  (:action unstack
    :parameters (?x - block ?y - block)
    :precondition (and
      (handempty)
      (block ?x)
      (block ?y)
      (clear ?x)
      (on ?x ?y)
    )
    :effect (and
      (holding ?x)
      (clear ?y)
      (not (handempty))
      (not (clear ?x))
      (not (on ?x ?y))
    )
  )

  ;; Action: stack onto block
  (:action stack
    :parameters (?x - block ?y - block)
    :precondition (and
      (holding ?x)
      (block ?x)
      (block ?y)
      (clear ?y)
    )
    :effect (and
      (not (holding ?x))
      (not (clear ?y))
      (handempty)
      (clear ?x)
      (on ?x ?y)
    )
  )
)
