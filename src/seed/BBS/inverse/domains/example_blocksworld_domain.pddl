(define (domain blocksworld)
  (:requirements :strips :typing)
  
  (:types block slot)

  (:predicates
    (clear ?x - block)
    (on ?x - block ?y - block)
    (on-table ?x - block ?y - slot)
    (holding ?x - block)
    (handempty)
  )

  ;; Action: pickup from table
  (:action iiwaPick
    :parameters (?x - block ?y - slot)
    :precondition (and
      (handempty)
      (clear ?x)
      (on-table ?x ?y)
    )
    :effect (and
      (not (handempty))
      (not (clear ?x))
      (clear ?y)
      (not (on-table ?x ?y))
      (holding ?x)
    )
  )

  ;; Action: put block on table
  (:action iiwaPut
    :parameters (?x - block ?y - slot)
    :precondition (and 
      (holding ?x) 
      (clear ?y)
    )
    :effect (and
      (handempty)
      (clear ?x)
      (on-table ?x ?y)
      (not (holding ?x))
      (not (clear ?y))
    )
  )

  ;; Action: unstack from block
  (:action iiwaUnstack
    :parameters (?x - block ?y - block)
    :precondition (and
      (handempty)
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
  (:action iiwaStack
    :parameters (?x - block ?y - block)
    :precondition (and
      (holding ?x)
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