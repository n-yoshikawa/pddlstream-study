(define (stream pick-and-place)
  (:stream find-motion
    :inputs (?obj ?start ?end)
    :fluents (AtConf)
    :domain (and (Object ?obj) (ObjectPose ?start) (ObjectPose ?end))
    :outputs (?path)
    :certified (Path ?path ?start ?end)
  )
)
