from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from analyse_results import *


directory = 'logs/control/'

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
print("\n\n")
print("===================================")
print("SST::::")
print("===================================")

print('selection_radius')
print(get_avg_time_per_param("control_SST", 'selection_radius'))
print(get_best_avg_time_per_param_per_params("control_SST", 'selection_radius'))
print(get_number_best("control_SST", 'selection_radius'))

print('pruning_radius')
print(get_avg_time_per_param("control_SST", 'pruning_radius'))
print(get_best_avg_time_per_param_per_params("control_SST", 'pruning_radius'))
print(get_number_best("control_SST", 'pruning_radius'))

print(get_min_time_and_name_in_avg_time_per_planner("control_SST"))
print(get_time_and_name_in_avg_time_per_planner_based_on_number_best("control_SST"))

print(get_every_time_per_planner_conf("control_SST", "control_SST selection_radius=25.00 pruning_radius=5.00"))
print(get_time_greater_every_time_per_planner_conf("control_SST", \
    "control_SST selection_radius=25.00 pruning_radius=5.00", 0.8))
print(get_counter_time_greater_every_time_per_planner_conf("control_SST", \
    "control_SST selection_radius=25.00 pruning_radius=5.00", 0.8))

print(get_sst_table())

create_image_sst_compare_params()


print("===================================")
print("::::")
print("===================================")

print(get_avg_time_of_planner_per_exp("control_RRTc intermediate_states=1.00"))

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
    
geometric_RRTstar_names = get_planner_names("control_RRTc")
print(geometric_RRTstar_names)

print("\n\n")
print("===================================")
print("RRTc::::")
print("===================================")

print('intermediate_states')
print(get_avg_time_per_param("control_RRTc", 'intermediate_states'))
print(get_best_avg_time_per_param_per_params("control_RRTc", 'intermediate_states'))
print(get_number_best("control_RRTc", 'intermediate_states'))

print(get_min_time_and_name_in_avg_time_per_planner("control_RRTc"))
print(get_time_and_name_in_avg_time_per_planner_based_on_number_best("control_RRTc"))

print(get_rrtc_table())

create_image_rrtc_compare_params()