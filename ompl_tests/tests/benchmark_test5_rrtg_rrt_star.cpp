 
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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

void setParam(std::string param, double value, const ompl::base::PlannerPtr& planner){
    ompl::base::ParamSet& params = planner->params();
    char params_buffer[100];
    if (params.hasParam(param)){
        params.setParam(param, ompl::toString(value));
        sprintf(params_buffer, "=%.2lf", value);
        planner->setName(planner->getName()+ std::string(" ") + param + std::string(params_buffer));
    }
}

void addRRTPlanner(ompl::tools::Benchmark& benchmark, const ompl::base::PlannerPtr& planner, double range, double intermediate_states){
    planner->setName(planner->getName()+ std::string("g"));
    setParam(std::string("range"), range, planner);
    setParam(std::string("intermediate_states"), intermediate_states, planner);
    std::cout << "name:: " << planner->getName() << std::endl; 
    benchmark.addPlanner(planner);
}

void addRRTStarPlanner(ompl::tools::Benchmark& benchmark, const ompl::base::PlannerPtr& planner, double new_state_rejection, \
    double range, double rewire_factor, double delayCC){
    setParam(std::string("new_state_rejection"), new_state_rejection, planner);
    setParam(std::string("range"), range, planner);
    setParam(std::string("rewire_factor"), rewire_factor, planner);
    setParam(std::string("delay_collision_checking"), delayCC, planner);
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
    
    sprintf(string_buffer, "../solution_paths/exp_%03d_run_%05d_%s.txt", exp_counter, run_counter++, planner->getName().c_str());
    
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
        // it does not make a big difference
        space->setSubspaceWeight(1, 0);

        // construct an instance of  space information from this state space
        og::SimpleSetup ss(space);
        
        // set state validity checking for this space
        ss.getSpaceInformation()->setStateValidityChecker(isStateValid);
        
        // set si/space parameters
        double longestSegmentLength = 5.0;
        // 5 = maxExtent_ * fraction
        ss.getSpaceInformation()->setStateValidityCheckingResolution(longestSegmentLength/ss.getStateSpace()->getMaximumExtent());
        // deve ser sqrt(800*800+1050*1050) -> 1320.03787824
        std::cout << "getMaximumExtent: " << ss.getStateSpace()->getMaximumExtent() << std::endl;
        std::cout << "longestSegmentFraction: " << ss.getStateSpace()->getLongestValidSegmentFraction() << std::endl;
        
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
        ss.setStartAndGoalStates(start, goal, 5.0);
        ss.setOptimizationObjective(getThresholdPathLengthObj(ss.getSpaceInformation()));
        // std::cout << "Threshold: "<<ss.getOptimizationObjective()->getCostThreshold() <<std::endl;
        
        ss.setup();
        std::cout << "getMaximumExtent: " << ss.getStateSpace()->getMaximumExtent() << std::endl;
        std::cout << "longestSegmentFraction: " << ss.getStateSpace()->getLongestValidSegmentFraction() << std::endl;
        // this should be available just now -> deve ser 5
        std::cout << "longestSegmentLength: " << ss.getStateSpace()->getLongestValidSegmentLength() << std::endl;

        // rrtg params
        std::vector<double> rrtg_ranges {50, 100, 250, 500, ss.getStateSpace()->getMaximumExtent()};
        std::vector<bool>   rrtg_intermediate_states {false, true};
        
        // rrt* params
        bool delayCC = false;
        std::vector<bool>   rrt_star_new_state_rejection {false, true};
        std::vector<double> rrt_star_ranges {50, 100, 250, 500, ss.getStateSpace()->getMaximumExtent()};
        std::vector<double> rrt_star_rewire_factor {1.05, 1.1, 1.5, 2.0, 5.0}; //esses valores realmente mudam k
        

        // by default, use the Benchmark class
        // nao ta respeitando o runtime_limit se runtime_limit < 0.1 
        double runtime_limit = 1, memory_limit = 4096, time_between_update = runtime_limit/5;
        int run_count = 5;
        ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, time_between_update, true, true, false);
        ompl::tools::Benchmark b(ss, "RRT_TEST");
        // b.addExperimentParameter("num_dims", "INTEGER", std::to_string(ndim));
        
        for(int it1=0; it1<rrtg_intermediate_states.size(); it1++){
            for(int it2=0; it2<rrtg_ranges.size(); it2++){
                addRRTPlanner(b, std::make_shared<og::RRT>(ss.getSpaceInformation()), rrtg_ranges[it2], rrtg_intermediate_states[it1]);
            }
        }
        
        for(int it1=0; it1<rrt_star_new_state_rejection.size(); it1++){
            for(int it2=0; it2<rrt_star_ranges.size(); it2++){
                for(int it3=0; it3<rrt_star_rewire_factor.size(); it3++){
                    addRRTStarPlanner(b, std::make_shared<og::RRTstar>(ss.getSpaceInformation()), \
                    rrt_star_new_state_rejection[it1], rrt_star_ranges[it2], rrt_star_rewire_factor[it3], delayCC);
                }
            }
        }
        
        b.setPreRunEvent(std::bind(&optionalPreRunEvent, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&optionalPostRunEvent, std::placeholders::_1, std::placeholders::_2));
        // b.setPostRunEvent(&optionalPostRunEvent);
        
        // setup dos planner eh feito aqui
        b.benchmark(request);
        char log_file[100];
        sprintf(log_file, "../logs/geometric/benchmark%03d.log", i);
        b.saveResultsToFile(log_file);
        exp_counter++;
    }
    
    return 0;
}