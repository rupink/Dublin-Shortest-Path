"""
    Problem 3 Template file
"""
import random
import math

import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees (RRT)
for a problem setup given by the "RRT_dubins_problem" class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file "rrt_planning.py". Your implementation
   can be tested by running "RRT_dubins_problem.py" (see the "main()" function).
2. Read all class and function documentation in "RRT_dubins_problem.py" carefully.
   There are plenty of helper functions in the class that you should use.
3. Your solution must meet all the conditions specificed below.
4. Below are some DOs and DONTs for this problem.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random points
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out related issues and will be generously set.
2. The planning function must return a list of nodes that represent a collision free path
   from the start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must be a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation of the node to understand the terminology)
3. The returned path should be a valid list of nodes with a Dubins-style path connecting the nodes. 
   i.e. the list should have the start node at index 0 and goal node at index -1. 
   For all other indices i in the list, the parent node for node i should be at index i-1,  
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   "RRT_dubins_problem.map_area"

DO(s) and DONT(s)
-------------------
1. DO rename the file to rrt_planning.py for submission.
2. Do NOT change change the "planning" function signature.
3. Do NOT import anything other than what is already imported in this file.
4. We encourage you to write helper functions in this file in order to reduce code repetition
   but these functions can only be used inside the "planning" function.
   (since only the planning function will be imported)
"""

def planning(rrt_dubins, display_map=False):

    i=0
    
    random.seed(654)
    
    for i in range(rrt_dubins.max_iter):
        

        # Generate a random vehicle state (x, y, yaw)
        
        if i % 5 == 0:
            x = rrt_dubins.goal.x
            y = rrt_dubins.goal.y
            yaw = rrt_dubins.goal.yaw
        else:
            '''x = random.randrange(rrt_dubins.x_lim[0], rrt_dubins.x_lim[1], 1)
            y = random.randrange(rrt_dubins.y_lim[0], rrt_dubins.y_lim[1], 1)
            yaw = np.deg2rad(random.randrange(0, 360, 1))'''
            x = random.uniform(rrt_dubins.x_lim[0], rrt_dubins.x_lim[1])
            y = random.uniform(rrt_dubins.y_lim[0], rrt_dubins.y_lim[1])
            yaw = np.deg2rad(random.uniform(0, 360))
            
            

        # Find an existing node nearest to the random vehicle state
        
        new_node = rrt_dubins.Node(x,y,yaw)
        
        print("this is new node ", (new_node.x, new_node.y))
        
        best_dist = float('inf')
        c_flag = False
        for i, exist_node in enumerate(rrt_dubins.node_list):
            dist = math.sqrt(math.pow(exist_node.x-new_node.x, 2) + math.pow(exist_node.y-new_node.y, 2))
            
            if exist_node.is_state_identical(new_node):
                c_flag = True
            
            if dist<best_dist:
                best_node = exist_node
                best_dist = dist
        
        if c_flag == True:
            continue
        #delta = np.array([best_node.x, best_node.y]) - np.array([new_node.x, new_node.y])
        print("iter" , (best_node.x, best_node.y))
        
        enter_new_node = rrt_dubins.propogate(best_node, new_node) #example of usage
        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to nodes_list if it is valid
        if rrt_dubins.check_collision(enter_new_node):
            rrt_dubins.node_list.append(enter_new_node) # Storing all valid nodes
        else:
            continue
        
    
                # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()
            
        if int(new_node.x) == int(rrt_dubins.goal.x):
            if int(new_node.y) == int(rrt_dubins.goal.y):
                if int(new_node.yaw) == int(rrt_dubins.goal.yaw):
                    break

        # Check if new_node is close to goal
        # if True:
        #     print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
        #     break

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    node = enter_new_node
    path = []
    while node!=rrt_dubins.start:
        path.append(node)
        node = node.parent
    
    #for node in path:
        #print(node.x, node.y)
        
    path.append(rrt_dubins.start)
    path.reverse()
    # Return path, which is a list of nodes leading to the goal

    return path
