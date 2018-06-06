###############################################
##Dmitry Sutormin, 2018##
##Rogaine route planner##

#Script takes a TAB file contains a set of control points (CPs) as input:
#CP_ID\tCP_Score\tCP_x\tCP_y\n
#The script computes distances between CPs and searches for the optimal route: min distance and max score.
#You can define start and end CPs, total distance you are planning to cover, 
#number of optimal routes to keep, as well as pathes to logfile 
#(contains information about the number of routes could be continued 
#and the number of routes completed on every iteration of the recursive algorithm). 

#Requirements: TAB file with CPs.
###############################################


#######
#Packages to be imported.
#######

import math
import matplotlib.pyplot as plt


#######
#Training set.
#######
'''
k_list={'cp1': 1, 'cp2': 2, 'cp3': 3}
k_taken=[]
distance_matrix={'SEcp1': 2, 'cp1SE': 2, 'SEcp2': 5, 'cp2SE': 5, 'cp3SE': 1.5, 'SEcp3': 1.5, 'cp1cp2': 4, 'cp2cp1': 4, 'cp1cp3': 10, 'cp3cp1': 10, 'cp2cp3': 8, 'cp3cp2': 8}
List_of_the_rotes=[[k_list, k_taken]]
'''

#######
#Variables to be defined.
#######

#Path to the input TAB file with CPs
Input_cp_file="C:/Sutor/Rogaine/Input_output/ten_cp.txt.txt"
#Total distance in the same unites as CPs coordinates in the input TAB file.
Distance_max=20000
#Start CP ID
Start='SE'
#End CP ID
End='SE'
#Path to the output logfile
Path_to_logfile="C:/Sutor/Rogaine/Input_output/ten_cp_log.txt"
#Path to the output map
Path_to_map="C:/Sutor/Rogaine/Input_output/ten_cp.png"
#Number of routes to keep after refinement
Number_of_routes_to_keep=5
#Array to store routes completed 
Output_routes=[]

#######
#Parses input TAB file with CPs.
#######

def read_cp_file(path, start, end):
    filein=open(path, 'r')
    cp_dict={}
    cp_coords={}
    for line in filein:
        line=line.rstrip().split('\t')
        cp_dict[str(line[0])]=int(line[1])
        cp_coords[str(line[0])]=[float(line[2]), float(line[3])]
    del cp_dict[start]
    if start!=end:
        del cp_dict[end]
    filein.close()
    return cp_dict, cp_coords

#######
#Computes euclidian distances between all possible pairs of CPs.
#######

def return_euclidian_distances(cp_coords):
    dist_matrix={}
    for k, v in cp_coords.items():
        for k1, v1 in cp_coords.items():
            if k!=k1:
                dist_matrix[str(k)+str(k1)]=math.sqrt((v[0]-v1[0])*(v[0]-v1[0]) + (v[1]-v1[1])*(v[1]-v1[1]))
    return dist_matrix
 
#######
#Extracts distance between particular pair of CPs from dictionary.
#######        

def Dist(node_1, node_2, dist_matrix):
    dist_matrix[str(node_1)+str(node_2)]
    return dist_matrix[str(node_1)+str(node_2)]

#######
#Recursive algorithm that computes possible routes and returns completed ones.
#######

def take_new_cp_mod(routes_list_cur, Dmax, start, end, dist_matrix, logfile, routes_to_keep, output_routes):
    routes_list_next=[]
    #Initiation.
    check_possible_route=0
    if len(routes_list_cur)==1 and len(routes_list_cur[0][1])==0:
        for cp_first, cp_first_score in routes_list_cur[0][0].items():
            not_taken_cp=dict(routes_list_cur[0][0])
            if Dist(start, cp_first, dist_matrix) + Dist(end, cp_first, dist_matrix) <= Dmax:
                #Remove cp chosen.
                del not_taken_cp[cp_first]
                routes_list_next.append([not_taken_cp, [[start, 0, 0], [cp_first, Dist(start, cp_first, dist_matrix), cp_first_score]]])
                check_possible_route=1
            else:
                print('Do not go to ' + str(cp_first) + ' first! You do not have enough time to return to finish!')  
        if check_possible_route==0:
            print('You do not have enough time to go anywhere... No possible routes detected.')
            return
    
    #Algorith has been already initiated.
    else:
        #Iterate routes already initiated.
        for route in routes_list_cur:
            #Iterate cps that are not taken.
            check_possible_route=0
            for cp_next, cp_next_score in route[0].items():
                not_taken_cp=dict(route[0])
                taken_cp=list(route[1])
                dist_cur=route[1][-1][1]
                cp_cur=route[1][-1][0]
                score_cur=route[1][-1][2]
                if dist_cur + Dist(cp_cur, cp_next, dist_matrix) + Dist(cp_next, end, dist_matrix) <= Dmax:
                    check_possible_route=1
                    del not_taken_cp[cp_next]
                    dist_new=dist_cur + Dist(cp_cur, cp_next, dist_matrix)
                    score_new=score_cur + cp_next_score                    
                    if len(not_taken_cp)!=0:
                        #print(not_taken_cp_next, taken_cp_next_copy, [k, New_dist, New_score])
                        #print(taken_cp_next_copy + [[k, New_dist, New_score]])
                        routes_list_next.append([not_taken_cp, taken_cp + [[cp_next, dist_new, score_new]]])
                    else:
                        output_routes.append(taken_cp + [[cp_next, dist_new, score_new]] + [[end, dist_new + Dist(cp_next, end, dist_matrix), score_new]])
            #No possible routes found.
            if check_possible_route==0:
                route_without_elongation=list(route[1])
                output_routes.append(route_without_elongation + [[end, route_without_elongation[-1][1] + Dist(route_without_elongation[-1][0], end, dist_matrix), route_without_elongation[-1][2]]])

    #Go to the next step of recursion if there are possible routes to prolong.
    print('Number of new completed routes found: ' + str(len(output_routes)))
    logfile.write('Number of new completed routes found: ' + str(len(output_routes)) + '\n')
    print('Number of routes could be prolonged: ' + str(len(routes_list_next)))
    logfile.write('Number of routes could be prolonged: ' + str(len(routes_list_next)) + '\n')
    output_routes=refine_output_routes(output_routes, routes_to_keep)
    if len(routes_list_next)>0:
        output_routes=take_new_cp_mod(routes_list_next, Dmax, start, end, dist_matrix, logfile, routes_to_keep, output_routes)
    return output_routes

#######
#Routes refinement: sorting, keeping only perspective.
#######

def refine_output_routes(routes_list, routes_to_keep):
    list_for_sorting=[]
    list_of_routes_refined=[]
    for i in range(len(routes_list)):
        list_for_sorting.append([i, routes_list[i][-1][2], routes_list[i][-1][1]])
    list_for_sorting.sort(key=lambda tup: tup[1], reverse=True)
    if len(list_for_sorting)!=0:
        the_highest_score=list_for_sorting[0][1]
        top_scores_list=[]
        for i in list_for_sorting:
            if i[1]==the_highest_score:
                top_scores_list.append(i)
            else:
                break
        top_scores_list.sort(key=lambda tup: tup[2], reverse=False)
        for i in top_scores_list[:routes_to_keep]:
            list_of_routes_refined.append(routes_list[i[0]])
    return list_of_routes_refined

#######
#Funsctions wrapper.
#######     

def wrapper(path, max_dist, start, end, path_to_log, routes_to_keep, output_routes, output_map):
    cp_dict_and_coords=read_cp_file(path, start, end)
    for k, v in cp_dict_and_coords[1].items():
        plt.plot(v[0], v[1])
        plt.annotate(k, xy=(v[0], v[1]), xytext=(-20, 20), textcoords='offset points')
    plt.savefig(output_map)
    dist_matrix=return_euclidian_distances(cp_dict_and_coords[1])
    logfile=open(path_to_log, 'w+')
    output_routes=take_new_cp_mod([[cp_dict_and_coords[0], []]], max_dist, start, end, dist_matrix, logfile, routes_to_keep, output_routes)
    logfile.write(str(output_routes))
    print(output_routes) 
    logfile.close()
    return

wrapper(Input_cp_file, Distance_max, Start, End, Path_to_logfile, Number_of_routes_to_keep, Output_routes, Path_to_map)