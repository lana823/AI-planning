(define (domain ghost_domain)

    (:requirements
     :strips
    )

    (:predicates
        (pacAt ?pos)
    	(dotAt ?pos)
    	(wallAt ?pos)
    	(ghostAt ?pos)
    	(capsuleAt ?pos)
    	(adjacent ?pos1 ?pos2)
    	(scared)
    )
    (:action move
        :parameters (?pos1 ?pos2)
        :precondition (and (ghostAt ?pos1) (not(wallAt ?pos2))
                    (adjacent ?pos1 ?pos2)(or(not(scared)) (and(scared)(not(pacAt ?pos2)))))
        :effect (and (when (and(not(scared))(pacAt ?pos2)) (not(pacAt ?pos2)))
                    (not(ghostAt ?pos1))
                    (ghostAt ?pos2))
    )
)