(define (stream pick-and-place)
  (:stream find-motion
    :inputs (?obj ?start ?end)
    :fluents (AtConf)
    :domain (and (Object ?obj) (ObjectPose ?start) (ObjectPose ?end))
    :outputs ()
    :certified (Movable ?obj ?start ?end)
  )
)
