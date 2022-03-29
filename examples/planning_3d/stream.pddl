(define (stream pick-and-place)
    (:stream find-traj
        :inputs (?arm ?block ?cur_handpose ?new_handpose) 
        :domain (and
            (arm ?arm)
            (block ?block) 
            (handpose ?cur_handpose)
            (grasppose ?block ?new_handpose)
        )
        :outputs (?traj)
        :certified (and
            (motion ?arm ?cur_handpose ?new_handpose ?traj) 
        )
    )
    (:stream find-traj-pour
        :inputs (?arm ?block ?cur_handpose ?new_handpose) 
        :domain (and
            (arm ?arm)
            (block ?block) 
            (handpose ?cur_handpose)
            (pourpose ?block ?new_handpose)
        )
        :outputs (?traj)
        :certified (and
            (motion ?arm ?cur_handpose ?new_handpose ?traj) 
        )
    )
    (:stream find-traj-place
        :inputs (?arm ?block ?cur_handpose ?new_handpose) 
        :domain (and
            (arm ?arm)
            (block ?block) 
            (handpose ?cur_handpose)
            (placepose ?block ?new_handpose)
        )
        :outputs (?traj)
        :certified (and
            (motion ?arm ?cur_handpose ?new_handpose ?traj) 
        )
    )
    
    (:stream find-grasp
        :inputs (?block ?block_pos)
        :domain (and
            (block ?block) 
            (coordinate ?block_pos) 
            (atconf ?block ?block_pos) 
        ) 
        :outputs (?handpose)
        :certified (and
            (handpose ?handpose)
            (grasppose ?block ?handpose)
        )
    )

    (:stream find-pour
        :inputs (?block ?block_pos)
        :domain (and
            (block ?block) 
            (coordinate ?block_pos) 
            (atconf ?block ?block_pos) 
        ) 
        :outputs (?handpose)
        :certified (and
            (handpose ?handpose)
            (pourpose ?block ?handpose)
        )
    )
    (:stream find-place
        :inputs (?block)
        :domain (and
            (block ?block) 
        ) 
        :outputs (?place_pos)
        :certified (and
            (placeposition ?block ?place_pos)
        )
    )
    (:stream find-place-pose
        :inputs (?block ?place_pos)
        :domain (and
            (block ?block) 
            (placeposition ?block ?place_pos)
        ) 
        :outputs (?handpose)
        :certified (and
            (handpose ?handpose)
            (placepose ?block ?handpose)
        )
    )
)
