from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from analyse_results import *


directory = 'logs/geometric/'

onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]
onlyfiles.sort()
print(onlyfiles)

for exp_n in range(len(onlyfiles)):
    print("Exp {}".format(exp_n))
    exps.append(Exp())
    exp = exps[exp_n]
    with open(directory+onlyfiles[exp_n]) as f:
        while(True):
            line = f.readline()
            if line[0:5] == "Flags":
                break
        f.readline()
        f.readline()
        f.readline()
        f.readline()
        line = f.readline().split()
        runs_per_planner = int(line[0])
        print("Runs per planner:")
        print(runs_per_planner)
        f.readline()
        f.readline()
        f.readline()
        line = f.readline().split()
        planners = int(line[0])
        for planner_number in range(planners):
            exp.planners_name.append(f.readline()[:-1])
            exp.planners_time[exp.planners_name[planner_number]] = list()
            print(exp.planners_name[planner_number])
            line = f.readline().split()
            common_prop = int(line[0])
            
            for _ in range(common_prop):
                f.readline()
            line = f.readline().split()
            prop_per_run = int(line[0])
            
            for _ in range(prop_per_run):
                f.readline()
            f.readline()
            
            for run in range(runs_per_planner):
                line = f.readline().split()
                # print(line[-2])
                exp.planners_time[exp.planners_name[planner_number]].append(float(line[-2][:-1]))
            line = f.readline()
            
            if line[0] != ".":
                line = line.split()
                prog_prop = int(line[0])
                for _ in range(prog_prop):
                    f.readline()
                f.readline()
                for run in range(runs_per_planner):
                    f.readline()
                f.readline() #"."
            print(exp.planners_time[exp.planners_name[planner_number]])
    print()
                

print("==========================================================")
print(len(exps))
print(get_avg_time_of_planner_per_exp("geometric_RRTstar new_state_rejection=1.00 range=1320.04 rewire_factor=1.50 delay_collision_checking=0.00"))

print("get_avg_time_per_planner_per_exp:")
avg_time_per_planner_per_exp = get_avg_time_per_planner_per_exp()
for key in avg_time_per_planner_per_exp.keys():
    print(key)
    print(avg_time_per_planner_per_exp[key])
    
print("get_avg_time_per_planner:")
avg_time_per_planner = get_avg_time_per_planner()
for key in avg_time_per_planner.keys():
    print(key)
    print(avg_time_per_planner[key])
    
geometric_RRTstar_names = get_planner_names("geometric_RRTg")
print(geometric_RRTstar_names)

print("oi")
print(get_avg_time_per_param("geometric_RRTg", 'range'))

print(get_avg_time_per_param("geometric_RRTstar", 'rewire_factor'))

print("\n\n")
print("===================================")
print("RRTstar::::")
print("===================================")
print('new_state_rejection')
print(get_avg_time_per_param("geometric_RRTstar", 'new_state_rejection'))
print(get_best_avg_time_per_param_per_params("geometric_RRTstar", 'new_state_rejection'))
print(get_number_best("geometric_RRTstar", 'new_state_rejection'))

print('range')
print(get_avg_time_per_param("geometric_RRTstar", 'range'))
print(get_best_avg_time_per_param_per_params("geometric_RRTstar", 'range'))
print(get_number_best("geometric_RRTstar", 'range'))

print('rewire_factor')
print(get_avg_time_per_param("geometric_RRTstar", 'rewire_factor'))
print(get_best_avg_time_per_param_per_params("geometric_RRTstar", 'rewire_factor'))
print(get_number_best("geometric_RRTstar", 'rewire_factor'))

print(get_min_time_and_name_in_avg_time_per_planner("geometric_RRTstar"))
print(get_time_and_name_in_avg_time_per_planner_based_on_number_best("geometric_RRTstar"))

print(get_rrtstar_table())

create_image_rrtstar_compare_params()


print("\n\n")
print("===================================")
print("RRTg::::")
print("===================================")
print('range')
print(get_avg_time_per_param("geometric_RRTg", 'range'))
print(get_best_avg_time_per_param_per_params("geometric_RRTg", 'range'))
print(get_number_best("geometric_RRTg", 'range'))

print('intermediate_states')
print(get_avg_time_per_param("geometric_RRTg", 'intermediate_states'))
print(get_best_avg_time_per_param_per_params("geometric_RRTg", 'intermediate_states'))
print(get_number_best("geometric_RRTg", 'intermediate_states'))

print(get_min_time_and_name_in_avg_time_per_planner("geometric_RRTg"))
print(get_time_and_name_in_avg_time_per_planner_based_on_number_best("geometric_RRTg"))

# print(get_avg_time_per_planner())
print(get_rrtg_table())

create_image_rrtg_compare_params()