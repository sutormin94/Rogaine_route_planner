# Rogaine_route_planner
Planning routes for rogains and other orienteering sports

##########################

## Rogaine_planner.py

Script takes a TAB file contains a set of control points (CPs) as input:
CP_ID\tCP_Score\tCP_x\tCP_y\n

The script computes distances between CPs and searches for the optimal route: min distance and max score.
You can define start and end CPs, total distance you are planning to cover, number of optimal routes to keep, as well as pathes to logfile (contains information about the number of routes could be continued and the number of routes completed on every iteration of the recursive algorithm). 

Requirements: TAB file with CPs.
