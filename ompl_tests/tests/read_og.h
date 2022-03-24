#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <stdio.h>

#include <iostream>
#include <fstream>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

#define OCC_CELL_SIZE          5   //cm
#define OCC_WIDTH              800
#define OCC_WIDTH_CELL         OCC_WIDTH/OCC_CELL_SIZE
#define OCC_HEIGHT             1000
#define OCC_HEIGHT_CELL        OCC_HEIGHT/OCC_CELL_SIZE
#define CAM_DIST               50 //eh 45 até o fim da chapa mas estamos considerando um pouco das rodas da frente

// std::ofstream write_file, read_file;
double goal_state_x, goal_state_z, goal_state_orientation;
std::vector<int> path_x;
std::vector<int> path_z;
std::vector<std::string> files;

// space vai de -400 a 400 e -50 a 1000

int occ_grid[OCC_HEIGHT_CELL][OCC_WIDTH_CELL];

inline double degree2rad(double value){
    return value*(M_PI/180.0);
}

inline uint get_grid_pos_x(double x){
    return (uint) ((x + (OCC_WIDTH/2))/OCC_CELL_SIZE);
}

inline uint get_grid_pos_z(double z){
    return (uint) (OCC_HEIGHT_CELL - 1 - (z/OCC_CELL_SIZE));
}

inline double get_pos_from_grid_x(int x) {
    // return x*OCC_CELL_SIZE - OCC_WIDTH/2 + OCC_CELL_SIZE*0.5;
    return x*OCC_CELL_SIZE - OCC_WIDTH/2;
}

inline double get_pos_from_grid_z(int z) {
    // return z*OCC_CELL_SIZE + OCC_CELL_SIZE*0.5;
    return OCC_HEIGHT - z*OCC_CELL_SIZE;
}

bool isStateValid(const ob::State *state)
{
    const auto *SE2state = state->as<ob::SE2StateSpace::StateType>();
    int x;
    int z;
    
    x = get_grid_pos_x(SE2state->getX());
    z = get_grid_pos_z(SE2state->getY());
    // std::cout << "rx = " << SE2state->getX() << "; ";
    // std::cout << "rz = " << SE2state->getY() << "\n";
    // std::cout << "x = " << x << "; ";
    // std::cout << "z = " << z << "\n";
    
    if(SE2state->getY() < -CAM_DIST || SE2state->getY() >= 1000){
        // std::cout << "**not valid - z out of bounds**\n";
        return false;
    }
    
    if(SE2state->getY() <= 0 && SE2state->getY() >= -CAM_DIST){
        // std::cout << "**valid**\n";
        return true;
    }
    
    if(x >= OCC_WIDTH_CELL || z >= OCC_HEIGHT_CELL || x < 0){
        // std::cout << "**not valid**\n";
        return false;
    }
    
    if(occ_grid[z][x]){
        // std::cout << "**valid**\n";
        return true;
    } else{
        // std::cout << "**not valid**\n";
        return false;
    }
}

void get_files(){
    FILE *fp;
    char file_name[1000];
    fp = fopen("../files.txt", "r");
    fscanf(fp, "%s\n", file_name);
    files.push_back(std::string(file_name));
    // std::cout << files[files.size()-1] << std::endl;
    while(file_name[0] != '$'){
        fscanf(fp, "%s\n", file_name);
        files.push_back(std::string(file_name));
        // std::cout << files[files.size()-1] << std::endl;
    }
    files.pop_back();
}


void read_info(const char * file_name){
    FILE *fp;
    int og_width_file, og_depth_file, path_len, goal_state_x_file, goal_state_z_file, path_x_file, path_z_file;

    fp = fopen(file_name, "r");
    // std::cout << std::string(file_name) << std::endl; 
    fscanf(fp, "width: %d depth: %d\n", &og_width_file, &og_depth_file);
    printf("%d %d\n",og_width_file, og_depth_file);
    
    for(int i=0; i< og_depth_file; i++){
        for(int j=0;j <og_width_file; j++){
            fscanf(fp, "%d", &occ_grid[i][j]);
            // printf("%d ", occ_grid[i][j]);
        }
        // printf("\n");
    }
    
    fscanf(fp, "\npath: %d\n", &path_len);
    printf("path_len: %d\n",path_len);
    for(int i=0; i<path_len; i++){
        fscanf(fp, "(%d, %d)\n", &path_z_file, &path_x_file);
        path_x.push_back(path_x_file);
        path_z.push_back(path_z_file);
    }
    
    fscanf(fp, "goal state: (%d, %d) %lf\n", &goal_state_z_file, &goal_state_x_file, &goal_state_orientation);
    goal_state_z = get_pos_from_grid_z(goal_state_z_file);
    goal_state_x = get_pos_from_grid_x(goal_state_x_file);
    printf("goal state: (%d, %d) %lf\n", goal_state_z_file, goal_state_x_file, goal_state_orientation);
    printf("goal state: (%lf, %lf) %lf\n", goal_state_z, goal_state_x, goal_state_orientation);
    
    fclose(fp);
}
