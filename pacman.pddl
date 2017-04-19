(define (domain pacman_domain)

    (:requirements
     :strips
    )

    (:types

    )

    (:constants

    )

    (:predicates
        (pacAt ?pos)
    	(dotAt ?pos)
    	(wallAt ?pos)
    	(ghostAt ?pos)
    	(capsuleAt ?pos)
    	(adjacent ?pos1 ?pos2)
    	(powered)

    )

    (:functions

    )

    (:action move
        :parameters (?pos1 ?pos2)
        :precondition (and (pacAt ?pos1) (not(wallAt ?pos2))
                        (or (powered)(not(ghostAt ?pos2)))
                        (adjacent ?pos1 ?pos2))
        :effect (and (when (capsuleAt ?pos2)
                        (and (powered)(not(capsuleAt ?pos2))))
                        (not(pacAt ?pos1))
                        (pacAt ?pos2)
                        (not(dotAt ?pos2))
                        (when (powered)(not(ghostAt ?pos2))))
    )
)