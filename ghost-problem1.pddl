﻿(define (problem ghost_problem1)
    (:domain ghost_domain)
;; make sure these are constants or objects:
(:objects loc1_1 loc1_2 loc2_1 loc2_2 )
(:init
(adjacent loc1_1 loc2_1)
(adjacent loc2_1 loc1_1)
(adjacent loc1_1 loc1_2)
(adjacent loc1_2 loc1_1)
(adjacent loc1_2 loc2_2)
(adjacent loc2_2 loc1_2)
(adjacent loc1_2 loc1_1)
(adjacent loc1_1 loc1_2)
(dotAt loc1_1)
(wallAt loc2_2)
(pacAt loc1_1)
(ghostAt loc2_1)
)
(:goal (not(pacAt loc1_1)))
)