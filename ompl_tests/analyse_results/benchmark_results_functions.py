from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
import matplotlib.image as mpimg



class Exp:
  def __init__(self):
    self.planners_name = list()
    self.planners_time = dict()

exps = list()
planners = 0
runs_per_planner = 0


def get_avg_time_of_planner_per_exp(name):
    avg_times = list()
    for exp in exps:
        times = exp.planners_time[name]
        avg = 0
        for t in times:
            avg+=t
        avg/=len(times)
        avg_times.append(avg)
    return avg_times

def get_avg_time_per_planner_per_exp():
    avg_time_per_planner_per_exp = dict()
    for name in exps[0].planners_name:
        avg_time_per_planner_per_exp[name] = get_avg_time_of_planner_per_exp(name)
    return avg_time_per_planner_per_exp

def avg(list):
    total = 0
    for i in range(len(list)):
        total += list[i]
    total/=len(list)
    return total

def get_avg_time_per_planner():
    avg_time_per_planner = dict()
    avg_time_per_planner_per_exp = get_avg_time_per_planner_per_exp()
    for name in avg_time_per_planner_per_exp.keys():
        avg_time_per_planner[name] = avg(avg_time_per_planner_per_exp[name])
    return avg_time_per_planner

def get_planner_names(planner):
    names = exps[0].planners_name
    planner_names = list()
    for name in names:
        if planner in name:
            planner_names.append(name)
    return planner_names

def get_param_value_in_name(name, param):
    splitted_name = name.split()
    value = ""
    for param_w_value in splitted_name:
        if param in param_w_value:
            param_w_value_splitted = param_w_value.split("=")
            value = param_w_value_splitted[1]
    return value
        
def get_param_values(planner_names, param):
    param_values = list()
    for name in planner_names:
        value = get_param_value_in_name(name, param)
        if value not in param_values:
            param_values.append(value)
    return param_values

            

def get_avg_time_per_param(planner, param):
    planner_names = get_planner_names(planner)
    param_values = get_param_values(planner_names, param)
    avg_time_per_param = dict()
    for param_value in param_values:
        avg_time_per_param[param_value] = list()
    
    avg_time_per_planner = get_avg_time_per_planner()
    for name in avg_time_per_planner.keys():
        if planner in name:
            avg_time_per_param[get_param_value_in_name(name, param)].append(avg_time_per_planner[name])
    
    return avg_time_per_param

def get_best_avg_time_per_param_per_params(planner, param):
    best_avg_time_per_param_per_params = list()
    avg_time_per_param = get_avg_time_per_param(planner, param)
    length = len(list(avg_time_per_param.values())[0])
    for i in range(length):
        min = 999999
        best = ""
        for param_value in avg_time_per_param.keys():
            if avg_time_per_param[param_value][i] < min:
                min = avg_time_per_param[param_value][i]
                best = param_value
        best_avg_time_per_param_per_params.append(best)
    return best_avg_time_per_param_per_params

def get_param_names(planner):
    param_names = list()
    planner_names = get_planner_names(planner)
    name = planner_names[0]
    splitted_name = name.split()
    for i in range(1, len(splitted_name)):
        param_names.append(splitted_name[i].split("=")[0])
    return param_names

def get_number_best(planner, param):
    planner_names = get_planner_names(planner)
    param_values = get_param_values(planner_names, param)
    best_avg_time_per_param_per_params = get_best_avg_time_per_param_per_params(planner, param)
    number_best = dict()
    for param_value in param_values:
        number_best[param_value] = 0
    for best in best_avg_time_per_param_per_params:
        number_best[best]+=1
    return number_best

def get_min_time_and_name_in_avg_time_per_planner(planner):
    avg_time_per_planner = get_avg_time_per_planner()
    min = 999999
    best = ""
    planner_names = get_planner_names(planner)
    for name in planner_names:
        if avg_time_per_planner[name] < min:
            min = avg_time_per_planner[name]
            best = name
    return best, min

def get_time_and_name_in_avg_time_per_planner_based_on_number_best(planner):
    best_param_value = dict()
    avg_time_per_planner = get_avg_time_per_planner()
    param_names = get_param_names(planner)
    for param_name in param_names:
        number_best = get_number_best(planner, param_name)
        max = -1
        for param_value in number_best.keys():
            if number_best[param_value] > max:
                max = number_best[param_value]
                best_param_value[param_name] = param_value
    # print(best_param_value)
    options = get_planner_names(planner)
    for param_name in param_names:
        options_kept = list()
        for option in options:
            value = get_param_value_in_name(option, param_name)
            if best_param_value[param_name] == value:
                options_kept.append(option)
        options = options_kept.copy()
    # print(options)
    return options[0], avg_time_per_planner[options[0]]

def get_every_time_per_planner_conf(planner, planner_conf):
    planner_names = get_planner_names(planner)
    every_time_per_planner = list()
    for exp in exps:
        for name in planner_names:
            if name ==planner_conf:
                every_time_per_planner.append(exp.planners_time[name])
        # print(exp.planners_time)
    return every_time_per_planner

def get_time_greater_every_time_per_planner_conf(planner, planner_conf, limit):
    every_time_per_planner = get_every_time_per_planner_conf(planner, planner_conf)
    time_greater = list()
    for i in range(len(every_time_per_planner)):
        time_greater.append(list())
    for i in range(len(every_time_per_planner)):
        for j in range(len(every_time_per_planner[i])):
            if every_time_per_planner[i][j] > limit:
                time_greater[i].append(every_time_per_planner[i][j])
    return time_greater

def get_counter_time_greater_every_time_per_planner_conf(planner, planner_conf, limit):
    time_greater = get_time_greater_every_time_per_planner_conf(planner, planner_conf, limit)
    counter_time = list()
    for i in range(len(time_greater)):
        counter_time.append(len(time_greater[i]))
    return counter_time

def get_rrtg_table():
    table=""
    avg_time_per_planner = get_avg_time_per_planner()
    # avg_time_per_planner_rrtg = dict()
    planner_names = get_planner_names("geometric_RRTg")
    # for name in planner_names:
    #     avg_time_per_planner_rrtg[name] = avg_time_per_planner[name]
    addIntermediateStatesValues = get_param_values(planner_names, "intermediate_states")
    # addIntermediateStatesValues.sort()
    rangeValues = get_param_values(planner_names, "range")
    # rangeValues.sort()
    # print(addIntermediateStatesValues)
    # print(rangeValues)
    times = list()
    # print(planner_names)
    for rangeValue in rangeValues:
        for addIntermediateStatesValue in addIntermediateStatesValues:
            # print(rangeValue, addIntermediateStatesValue)
            for name in planner_names:
                name_rangeValue = get_param_value_in_name(name, "range")
                name_addIntermediateStatesValue = get_param_value_in_name(name, "intermediate_states")
                if addIntermediateStatesValue == name_addIntermediateStatesValue and rangeValue == name_rangeValue:
                    # print(name)
                    times.append(avg_time_per_planner[name])
    # print(times)
    
    with open("tables/rrtg.txt", 'r') as file:
        lines = file.readlines()
        for line in lines:
            if "putValue" in line:
                while("putValue" in line):
                    line = line.replace("putValue", "{:.6f}".format(times[0]), 1)
                    times.pop(0)
                table+=line
            else:
                table+=line
    return table

def get_rrtstar_table():
    table=""
    avg_time_per_planner = get_avg_time_per_planner()
    planner_names = get_planner_names("geometric_RRTstar")
    rewireFactorValues = get_param_values(planner_names, "rewire_factor")
    rangeValues = get_param_values(planner_names, "range")
    newStateRejectionValues = get_param_values(planner_names, "new_state_rejection")
    times = list()
    # print(planner_names)
    for newStateRejectionValue  in newStateRejectionValues:
        for rangeValue in rangeValues:
            for rewireFactorValue in rewireFactorValues:
                # print(rangeValue, addIntermediateStatesValue)
                for name in planner_names:
                    name_rangeValue = get_param_value_in_name(name, "range")
                    name_rewireFactorValue = get_param_value_in_name(name, "rewire_factor")
                    name_newStateRejectionValue = get_param_value_in_name(name, "new_state_rejection")
                    if rewireFactorValue == name_rewireFactorValue and rangeValue == name_rangeValue \
                    and newStateRejectionValue == name_newStateRejectionValue:
                        # print(name)
                        times.append(avg_time_per_planner[name])
    # print(times)
    
    with open("tables/rrtstar.txt", 'r') as file:
        lines = file.readlines()
        for line in lines:
            if "putValue" in line:
                while("putValue" in line):
                    line = line.replace("putValue", "{:.6f}".format(times[0]), 1)
                    times.pop(0)
                table+=line
            else:
                table+=line
    return table
    
def get_sst_table():
    table=""
    avg_time_per_planner = get_avg_time_per_planner()
    # avg_time_per_planner_rrtg = dict()
    planner_names = get_planner_names("control_SST")
    # for name in planner_names:
    #     avg_time_per_planner_rrtg[name] = avg_time_per_planner[name]
    srValues = get_param_values(planner_names, "selection_radius")
    # addIntermediateStatesValues.sort()
    prValues = get_param_values(planner_names, "pruning_radius")
    # rangeValues.sort()
    print(srValues)
    print(prValues)
    times = list()
    # print(planner_names)
    for prValue in prValues:
        for srValue in srValues:
            # print(rangeValue, addIntermediateStatesValue)
            for name in planner_names:
                name_srValue = get_param_value_in_name(name, "selection_radius")
                name_prValue = get_param_value_in_name(name, "pruning_radius")
                if srValue == name_srValue and prValue == name_prValue:
                    # print(name)
                    times.append(avg_time_per_planner[name])
    # print(times)
    
    with open("tables/sst.txt", 'r') as file:
        lines = file.readlines()
        for line in lines:
            if "putValue" in line:
                while("putValue" in line):
                    line = line.replace("putValue", "{:.6f}".format(times[0]), 1)
                    times.pop(0)
                table+=line
            else:
                table+=line
    return table
    
def get_rrtc_table():
    table=""
    avg_time_per_planner = get_avg_time_per_planner()
    # avg_time_per_planner_rrtg = dict()
    planner_names = get_planner_names("control_RRT")
    # for name in planner_names:
    #     avg_time_per_planner_rrtg[name] = avg_time_per_planner[name]
    addIValues = get_param_values(planner_names, "intermediate_states")
    # rangeValues.sort()
    print(addIValues)
    times = list()
    # print(planner_names)
    for addIValue in addIValues:
        # print(rangeValue, addIntermediateStatesValue)
        for name in planner_names:
            name_addIValue = get_param_value_in_name(name, "intermediate_states")
            if addIValue == name_addIValue:
                times.append(avg_time_per_planner[name])
    # print(times)
    
    with open("tables/rrtc.txt", 'r') as file:
        lines = file.readlines()
        for line in lines:
            if "putValue" in line:
                while("putValue" in line):
                    line = line.replace("putValue", "{:.6f}".format(times[0]), 1)
                    times.pop(0)
                table+=line
            else:
                table+=line
    return table

    
def create_image_rrtg_compare_params():
    dir = "selected_images/rrtg_compare_params/"
    onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
    onlyfiles.sort()
    # print(onlyfiles)
    fig, axes = plt.subplots(2, 5, figsize=(15, 10), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0
    planner_names = get_planner_names("geometric_RRTg")
    # print(planner_names)
    # print(onlyfiles)
    plt.rcParams.update({'font.size': 15})
    # for name in planner_names:
    #     red_name = name[len("geometric_"):]
    #     print(red_name)
    #     for file in onlyfiles:
    #         if red_name in file:
    #             img = mpimg.imread(dir+file)
    for file in onlyfiles:
        img = mpimg.imread(dir+file)
        for name in planner_names:
            red_name = name[len("geometric_"):]
            if red_name in file:
                ax[img_position].imshow(img, cmap='magma')
                addI = ""
                if get_param_value_in_name(name, "intermediate_states") == "0.00":
                    addI = "False"
                else:
                    addI = "True"
                ax[img_position].set_title("range="+get_param_value_in_name(name, "range")+"\naddI="+addI)
                ax[img_position].axis('off')
                img_position+=1 
    fig.tight_layout()
    plt.savefig(dir+'rrtg_compare_params.png')

def create_image_rrtstar_compare_params():
    dir = "selected_images/rrtstar_compare_params/"
    onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
    onlyfiles.sort()
    # print(onlyfiles)
    fig, axes = plt.subplots(2, 5, figsize=(15, 10), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0
    planner_names = get_planner_names("geometric_RRTstar")
    # print(planner_names)
    # print(onlyfiles)
    plt.rcParams.update({'font.size': 15})
    # for name in planner_names:
    #     red_name = name[len("geometric_"):]
    #     print(red_name)
    #     for file in onlyfiles:
    #         if red_name in file:
    #             img = mpimg.imread(dir+file)
    for file in onlyfiles:
        img = mpimg.imread(dir+file)
        for name in planner_names:
            red_name = name[len("geometric_"):]
            if red_name in file:
                ax[img_position].imshow(img, cmap='magma')
                nsr = ""
                if get_param_value_in_name(name, "new_state_rejection") == "0.00":
                    nsr = "False"
                else:
                    nsr = "True"
                ax[img_position].set_title("NSR="+nsr+"\nrange="+get_param_value_in_name(name, "range")\
                    +"\nRF="+get_param_value_in_name(name, "rewire_factor"))
                ax[img_position].axis('off')
                img_position+=1 
    fig.tight_layout()
    plt.savefig(dir+'rrtstar_compare_params.png')
    
def create_image_rrtc_compare_params():
    dir = "selected_images/rrtc_compare_params/"
    onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
    onlyfiles.sort()
    # print(onlyfiles)
    fig, axes = plt.subplots(2, 5, figsize=(15, 10), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0
    planner_names = get_planner_names("control_RRTc")
    # print(planner_names)
    # print(onlyfiles)
    plt.rcParams.update({'font.size': 15})
    # for name in planner_names:
    #     red_name = name[len("control_"):]
    #     print(red_name)
    #     for file in onlyfiles:
    #         if red_name in file:
                # img = mpimg.imread(dir+file)
    for file in onlyfiles:
        img = mpimg.imread(dir+file)
        for name in planner_names:
            red_name = name[len("control_"):]
            if red_name in file:
                ax[img_position].imshow(img, cmap='magma')
                addI = ""
                if get_param_value_in_name(name, "intermediate_states") == "0.00":
                    addI = "False"
                else:
                    addI = "True"
                ax[img_position].set_title("addI={}".format(addI))
                ax[img_position].axis('off')
                img_position+=1 
    fig.tight_layout()
    plt.savefig(dir+'rrtc_compare_params.png')
        
def create_image_sst_compare_params():
    dir = "selected_images/sst_compare_params/"
    onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
    onlyfiles.sort()
    # print(onlyfiles)
    fig, axes = plt.subplots(2, 5, figsize=(15, 10), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0
    planner_names = get_planner_names("control_SST")
    # print(planner_names)
    # print(onlyfiles)
    plt.rcParams.update({'font.size': 15})
    # for name in planner_names:
    #     red_name = name[len("control_"):]
    #     print(red_name)
    #     for file in onlyfiles:
    #         if red_name in file:
    for file in onlyfiles:
        img = mpimg.imread(dir+file)
        for name in planner_names:
            red_name = name[len("control_"):]
            if red_name in file:
                img = mpimg.imread(dir+file)
                ax[img_position].imshow(img, cmap='magma')
                ax[img_position].set_title("SR="+get_param_value_in_name(name, "selection_radius")\
                    +"\nPR="+get_param_value_in_name(name, "pruning_radius"))
                ax[img_position].axis('off')
                img_position+=1 
    fig.tight_layout()
    plt.savefig(dir+'sst_compare_params.png')
        