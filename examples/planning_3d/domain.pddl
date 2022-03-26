(define (domain pick-and-place)
    (:requirements :strips :derived-predicates :disjunctive-preconditions :equality)

    (:predicates 
        ; type/static predicates
        (arm ?arm)
        (block ?block)
        (picked ?arm ?block)
        (handpose ?pose)
        (coordinate ?xyz)
        (motion ?arm ?cur_handpose ?new_handpose ?traj)

        ; stream certified
        ; find-ik
        ;(ik ?arm ?block ?X_WB ?X_HB ?pre_q ?q)
        ; find-motion
        ; check-colfree-block
        ; if arm is at q and item at X_WI, are there collisions 
        ;(colfree-block ?arm ?q ?block ?X_WB)
        ; check-colfree-arms
        ; if arm1 is at q1, and arm2 is at q2, are there collisions?
        ;(colfree-arms ?arm1 ?q1 ?arm2 ?q2)
        ; find-table-place
        ;(table-support ?block ?X_WB ?table)
        ; find-block-place
        ;(block-support ?upperblock ?X_WU ?lowerblock ?X_WL)

        ; fluents 
        (empty ?arm)
        (atconf ?block ?xyz)
        (athandpose ?arm ?handpose)
        (grasppose ?block ?handpose)
    )

    (:action pick
        :parameters (?arm ?block ?block_pos ?cur_handpose ?new_handpose ?traj)
        :precondition (and
            (arm ?arm)
            (empty ?arm)
            (block ?block)
            (atconf ?block ?block_pos)
            (handpose ?cur_handpose)
            (athandpose ?arm ?cur_handpose)
            (grasppose ?block ?new_handpose)
            (motion ?arm ?cur_handpose ?new_handpose ?traj)
        ) 
        :effect (and
            (not (empty ?arm))
            (not (athandpose ?arm ?cur_handpose))
            (athandpose ?arm ?new_handpose)
            (picked ?arm ?block)
        )
    )
    (:action place
        :parameters (?arm ?block)
        :precondition (and
            (arm ?arm)
            (block ?block)
            (picked ?arm ?block)
        ) 
        :effect (and
            (empty ?arm)
        )
    )
)
