 
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/util/String.h>

#include <string>
#include <map>

#include <math.h>

#include "read_og.h"

int run_counter = 0;
int exp_counter = 0;
int files_count;
double manhattan_dist;
// medir
double crawler_L = 40;

void setParam(std::string param, double value, const ompl::base::PlannerPtr& planner){
    ompl::base::ParamSet& params = planner->params();
    char params_buffer[100];
    if (params.hasParam(param)){
        params.setParam(param, ompl::toString(value));
        sprintf(params_buffer, "=%.2lf", value);
        planner->setName(planner->getName()+ std::string(" ") + param + std::string(params_buffer));
    }
}

void addRRTPlanner(ompl::tools::Benchmark& benchmark, const ompl::base::PlannerPtr& planner, double intermediate_states){
    planner->setName(planner->getName()+ std::string("c"));
    setParam(std::string("intermediate_states"), intermediate_states, planner);
    std::cout << "name:: " << planner->getName() << std::endl; 
    benchmark.addPlanner(planner);
}

void addSSTPlanner(ompl::tools::Benchmark& benchmark, const ompl::base::PlannerPtr& planner, double selection_radius, double pruning_radius){
    // planner->setName(planner->getName()+ std::string("c"));
    setParam(std::string("selection_radius"), selection_radius, planner);
    setParam(std::string("pruning_radius"), pruning_radius, planner);
    std::cout << "name:: " << planner->getName() << std::endl; 
    benchmark.addPlanner(planner);
}


void optionalPreRunEvent(const ob::PlannerPtr &planner)
{
    // if(planner->getName().compare("RRT-range=3.000000") == 0){
        // std::cout << "change si\n";
        // planner->getSpaceInformation()->setStateValidityCheckingResolution(0.1);
    // }else{
        // std::cout << "unchange si\n";
    // }
    // std::cout << "planner name: "<< planner->getName() << std::endl;
    // std::cout << "Resolution planner: "<< si_resolution_for_planner[planner->getName()] << std::endl;
    // planner->getSpaceInformation()->setStateValidityCheckingResolution(si_resolution_for_planner[planner->getName()]);
    // std::cout << "Resolution: "<< planner->getSpaceInformation()->getStateValidityCheckingResolution() << std::endl;
    
}

 

void optionalPostRunEvent(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run)
{
    auto pdef = planner->getProblemDefinition();
    ob::PathPtr path = pdef->getSolutionPath();
    
    char string_buffer [200];
    
    sprintf(string_buffer, "../solution_paths/exp_%03d_run_%03d_%s.txt", exp_counter, run_counter++, planner->getName().c_str());
    
    std::ofstream myfile;
    myfile.open(string_buffer);
    myfile << OCC_HEIGHT_CELL << " " << OCC_WIDTH_CELL << std::endl;
    for(int i=0;i<OCC_HEIGHT_CELL;i++){
        for(int j=0;j<OCC_WIDTH_CELL;j++){
            myfile << occ_grid[i][j] << " ";
        }
        myfile << std::endl;
    }
    path->print(myfile);
    myfile.close();
}
 
// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    manhattan_dist = abs(goal_state_x)+CAM_DIST+goal_state_z;
    std::cout << "Threshold: "<< manhattan_dist <<std::endl;
    obj->setCostThreshold(ob::Cost(manhattan_dist));
    return obj;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double yaw = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
    // std::cout << "duration: "<< duration << std::endl ;
 
    // result->as<ob::SE2StateSpace::StateType>()->setXY(
    //     pos[0] + ctrl[0] * duration * cos(rot),
    //     pos[1] + ctrl[0] * duration * sin(rot));
    // // result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);
    // double value = (rot + ctrl[1] * duration);
    // if (value >= 2*M_PI) {
    //     value -= 4*M_PI;
    // } else if(value < -2*M_PI){
    //     value += 4*M_PI;
    // }
    
    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * cos(yaw),
        pos[1] + ctrl[0] * sin(yaw));
    // result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);
    double new_yaw = yaw;
    // std::cout << "yaw: " << yaw <<std::endl; 
    new_yaw += (ctrl[0]/crawler_L)*tan(ctrl[1]);
    // std::cout << "new_yaw: " << new_yaw <<std::endl;
    // precisa? 
    // if (new_yaw >= 2*M_PI) {
    //     new_yaw -= 2*M_PI;
    // } else if(new_yaw < -2*M_PI){
    //     new_yaw += 2*M_PI;
    // }
    // intervalo eh de [0, pi][-pi, 0]
    if (new_yaw > M_PI && new_yaw < 2*M_PI) {
        new_yaw -= 2*M_PI;
    } else if(new_yaw < -M_PI && new_yaw > -2*M_PI) {
        new_yaw += 2*M_PI;
    }  else if(new_yaw >= 2*M_PI){
        new_yaw -= 2*M_PI;
    } else if(new_yaw < -2*M_PI){
        new_yaw += 2*M_PI;
    }
    // std::cout << "final_new_yaw: " << new_yaw <<std::endl;
    result->as<ob::SE2StateSpace::StateType>()->setYaw(new_yaw);
}

int main(int argc, char **argv)
{
    get_files();
    files_count = files.size();
    for(int i=0; i<files_count; i++){
        run_counter = 0;
        // read info
        read_info((std::string("../../skeletonize_and_goal_state_tests/to_mp/") + files[i]).c_str());

        
        // construct the state space we are planning in
        auto space(std::make_shared<ob::SE2StateSpace>());
        
        // set the bounds for the R^2 part of SE(2)
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, -OCC_WIDTH/2);
        bounds.setHigh(0, OCC_WIDTH/2 - 0.00001); //make sure that 0CC_WIDTH/2 is out of the boundary
        bounds.setLow(1, -CAM_DIST);
        bounds.setHigh(1, OCC_HEIGHT - 0.00001);
        space->setBounds(bounds);
        // enforce yaw even with a allowed position displacement
        std::cout << "Weights:" << std::endl;
        std::cout << space->getSubspaceWeight(0) << std::endl;
        std::cout << space->getSubspaceWeight(1) << std::endl;
        // w0*5cm = w1*degree2rad(10 degrees)
        // w1 = (w0*5cm)/degree2rad(10 degrees)
        double new_yaw_weight = 5.0/degree2rad(5);
        std::cout << "New yaw weight:" << std::endl;
        std::cout << new_yaw_weight << std::endl;
        space->setSubspaceWeight(1, new_yaw_weight);
        
        // space->setSubspaceWeight(1, 0);
    
        // create a control space
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
        // set the bounds for the control space
        ob::RealVectorBounds cbounds(2);
        cbounds.setLow(0, 0);
        //v = 100cm/s, mas vamos considerar que cada step de motion tem 0.05s ent??o vamos passar o deslocamento max na bound superior
        double vel_max = 50;
        // double step_time = 0.05;
        double step_time = 0.2;
        // double step_time = 0.5;
        // 0.5s - 90
        // step_time - x
        // 0.5*x = 90*step_time
        // x = (90*step_time)/0.5
        double rot_bound = (60.0*step_time)/1;
        cbounds.setHigh(0, vel_max*step_time);
        // apesar de o alcance de rotacao das rodas ser de [-45, 45] vamos considerar menos, pois imaginamos que a roda n??o consegue
        // rotacionar tudo isso em um intervalo de tempo t??o pequeno - vai virar no add no yaw no m??ximo 0.6265913968685753 graus por delta_t
        cbounds.setLow(1, degree2rad(-rot_bound));
        cbounds.setHigh(1, degree2rad(rot_bound));
        // cbounds.setLow(1, degree2rad(-45));
        // cbounds.setHigh(1, degree2rad(45));
        cspace->setBounds(cbounds);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // construct an instance of  space information from this state space
        oc::SimpleSetup ss(si);

        // set state validity checking for this space
        ss.getSpaceInformation()->setStateValidityChecker(isStateValid);
        
        // set si/space parameters
        double longestSegmentLength = 5.0;
        // 5 = maxExtent_ * fraction
        ss.getSpaceInformation()->setStateValidityCheckingResolution(longestSegmentLength/ss.getStateSpace()->getMaximumExtent());
        // deve ser sqrt(800*800+1050*1050) -> 1320.03787824
        std::cout << "getMaximumExtent: " << ss.getStateSpace()->getMaximumExtent() << std::endl;
        std::cout << "longestSegmentFraction: " << ss.getStateSpace()->getLongestValidSegmentFraction() << std::endl;
        
        ss.getSpaceInformation()->setStatePropagator(propagate);
        // cuidar se add_intermediate eh true ou false
        ss.getSpaceInformation()->setMinMaxControlDuration(5, 15);
        ss.getSpaceInformation()->setPropagationStepSize(15); //deixar sempre igual ao maxControlDuration para n??o influenciar em nada
        
        // ss.getSpaceInformation()->setup();

        // create a random start state
        ob::ScopedState<ob::SE2StateSpace> start(space);
        start->setXY(0, -CAM_DIST);
        start->setYaw(degree2rad(90));

        // create a random goal state
        ob::ScopedState<ompl::base::SE2StateSpace> goal(space);
        goal->setXY(goal_state_x, goal_state_z);
        goal->setYaw(goal_state_orientation);

        // set the start and goal states and threashold
        ss.setStartAndGoalStates(start, goal, 20);
        ss.setOptimizationObjective(getThresholdPathLengthObj(ss.getSpaceInformation()));
        // std::cout << "Threshold: "<<ss.getOptimizationObjective()->getCostThreshold() <<std::endl;
        
        ss.setup();
        std::cout << "getMaximumExtent: " << ss.getStateSpace()->getMaximumExtent() << std::endl;
        std::cout << "longestSegmentFraction: " << ss.getStateSpace()->getLongestValidSegmentFraction() << std::endl;
        // this should be available just now -> deve ser 5
        std::cout << "longestSegmentLength: " << ss.getStateSpace()->getLongestValidSegmentLength() << std::endl;

        // rrtc params
        std::vector<bool>   rrtc_intermediate_states {false, true};
        // std::vector<bool>   rrtc_intermediate_states {true};
        
        // sst params
        std::vector<double>   sst_selection_radius {0.2, 1, 5, 25, 50};
        std::vector<double>   sst_pruning_radius {1, 2.5, 5, 10, 30};
        // std::vector<double>   sst_selection_radius {25};
        // std::vector<double>   sst_pruning_radius {5};
        

        // by default, use the Benchmark class
        // nao ta respeitando o runtime_limit se runtime_limit < 0.1 
        // acho que para o rrtc o goal bias nao tem efeito, por isso ele demora bem mais
        double runtime_limit = 0.1, memory_limit = 4096, time_between_update = runtime_limit/5;
        int run_count = 5;
        ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, time_between_update, true, true, false);
        ompl::tools::Benchmark b(ss, "RRT_TEST");
        // b.addExperimentParameter("num_dims", "INTEGER", std::to_string(ndim));
        
        for(int it1=0; it1<rrtc_intermediate_states.size(); it1++){
            // for(int it2=0; it2<rrtg_ranges.size(); it2++){
                // addRRTPlanner(b, std::make_shared<oc::RRT>(ss.getSpaceInformation()), rrtg_ranges[it2], rrtc_intermediate_states[it1]);
                addRRTPlanner(b, std::make_shared<oc::RRT>(ss.getSpaceInformation()), rrtc_intermediate_states[it1]);
            // }
        }
        
        for(int it1=0; it1<sst_selection_radius.size(); it1++){
            for(int it2=0; it2<sst_pruning_radius.size(); it2++){
                addSSTPlanner(b, std::make_shared<oc::SST>(ss.getSpaceInformation()), sst_selection_radius[it1], sst_pruning_radius[it2]);
            }
        }
        
        
        b.setPreRunEvent(std::bind(&optionalPreRunEvent, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&optionalPostRunEvent, std::placeholders::_1, std::placeholders::_2));
        // b.setPostRunEvent(&optionalPostRunEvent);
        
        // setup dos planner eh feito aqui
        b.benchmark(request);
        char log_file[100];
        sprintf(log_file, "../logs/control/benchmark%03d.log", i);
        b.saveResultsToFile(log_file);
        exp_counter++;
        // break;
    }
    
    return 0;
}