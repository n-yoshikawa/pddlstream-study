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
)
