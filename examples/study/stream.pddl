(define (stream pick-and-place)
  (:stream motion-planning
    :inputs (?obj)
    :fluents (AtConf)
    :domain (Object ?obj)
    :outputs ()
    :certified (and (Placeable ?obj))
  )
)
