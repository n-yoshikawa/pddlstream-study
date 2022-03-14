(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Conf ?q)
    (Arm ?arm)
    (Object ?obj)
    (ArmEmpty ?arm)
    (Movable ?obj ?start ?end)
    (MovableEach ?obstacle ?obstacle_pos ?start ?end)
    (ObjectPose ?p)
    ; Fluent predicates
    (AtConf ?obj ?q)
    (ArmPos ?arm ?q)
  )
  (:action pick
    :parameters (?arm ?obj_pos)
    :precondition (and (ArmEmpty ?arm) (ObjectPose ?obj_pos))
    :effect (and (not (ArmEmpty ?arm))
                 (ArmPos ?arm ?obj_pos))
  )
  (:action move_and_place
    :parameters (?arm ?obj ?obj_pos ?goal)
    :precondition (and (AtConf ?obj ?obj_pos)
                       (ArmPos ?arm ?obj_pos)
                       (forall (?obstacle ?obstacle_pos) (imply (AtConf ?obstacle ?obstacle_pos)
                                                                (MovableEach ?obstacle ?obstacle_pos ?obj_pos ?goal)))
                       (ObjectPose ?obj_pos)
                       (ObjectPose ?goal))
    :effect (and (ArmEmpty ?arm)
                 (not (ArmPos ?arm ?obj_pos))
                 (ArmPos ?arm ?goal)
                 (not (AtConf ?obj ?obj_pos))
                 (AtConf ?obj ?goal))
  )
)
