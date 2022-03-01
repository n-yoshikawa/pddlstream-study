(define (stream pick-and-place)
  (:stream find-motion
    :inputs (?obj)
    :fluents (AtConf)
    :domain (Object ?obj)
    :outputs ()
    :certified (Placeable ?obj)
  )
)
