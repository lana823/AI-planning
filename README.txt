15.OCT.2016

Group: WhatEver

Grop Member:
Lan Yang  746569 lany4@student.unimelb.edu.au
Jianan Hu 748138 jiananh@student.unimelb.edu.au
Fan Hong  795265 Hongf@student.unimelb.edu.au

Techniques used in This project:

In the project, we chose heuristic search to implement our plan.
For offensive Agent, we use A* search. It takes the minimum maze distance to the goal point as the heuristic and adds up the weighted value as the costs. For walls and ghost, we set the cost to be relatively high. For enemies out of sight, we multiply some weights by the probability of enemy on that position. We take some threshold for the path depth to control the searching time and return the first action of the current optimal path. Also, we have a start path for the dots initiate from the starting point.
For defending, we take the nearest enemy with higher probability of its position as the goal and the distances to the enemies as the heuristics.
