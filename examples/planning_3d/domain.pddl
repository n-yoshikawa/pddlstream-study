(define (domain pick-and-place)
    (:requirements :strips :derived-predicates :disjunctive-preconditions :equality)

    (:predicates 
        ; type/static predicates
        (arm ?arm)
        (block ?block)
        (picked ?arm ?block)
        (poured ?arm ?block)
        (handpose ?pose)
        (coordinate ?xyz)
        (grasppose ?block ?handpose)
        (pourpose ?block ?handpose)
        (placeposition ?block ?place_pos)
        (placepose ?block ?handpose)
        (motion ?arm ?cur_handpose ?new_handpose ?traj)

        ; fluents 
        (empty ?arm)
        (atconf ?block ?xyz)
        (athandpose ?arm ?handpose)
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
        :parameters (?arm ?block ?place_pos ?cur_handpose ?new_handpose ?traj)
        :precondition (and
            (arm ?arm)
            (block ?block)
            (picked ?arm ?block)
            (placeposition ?block ?place_pos)
            (handpose ?cur_handpose)
            (athandpose ?arm ?cur_handpose)
            (placepose ?block ?new_handpose)
            (motion ?arm ?cur_handpose ?new_handpose ?traj)
        ) 
        :effect (and
            (empty ?arm)
            (not (picked ?arm ?block))
        )
    )
    (:action pour
        :parameters (?arm ?block ?block_pos ?cur_handpose ?new_handpose ?traj)
        :precondition (and
            (arm ?arm)
            (block ?block)
            (atconf ?block ?block_pos)
            (handpose ?cur_handpose)
            (athandpose ?arm ?cur_handpose)
            (pourpose ?block ?new_handpose)
            (motion ?arm ?cur_handpose ?new_handpose ?traj)
        ) 
        :effect (and
            (not (empty ?arm))
            (not (athandpose ?arm ?cur_handpose))
            (athandpose ?arm ?new_handpose)
            (poured ?arm ?block)
        )
    )
)
