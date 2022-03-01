(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Conf ?q)
    (Object ?obj)
    (Placeable ?obj)
    (Picked ?obj)
    (Moved ?obj)
    (Poured ?obj)
    (Finished ?obj)
    ; Fluent predicates
    (AtConf ?obj ?q)
  )
  (:action pick
    :parameters (?obj)
    :precondition (not (Picked ?obj))
    :effect (Picked ?obj)
  )
  (:action move
    :parameters (?obj ?goal)
    :precondition (Picked ?obj)
    :effect (and (Moved ?obj))
  )
  (:action place
    :parameters (?obj ?goal)
    :precondition (and (Picked ?obj) (Placeable ?obj))
    :effect (and (not (Picked ?obj)) (AtConf ?obj ?goal))
  )
)
