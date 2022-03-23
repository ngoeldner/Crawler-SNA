#include <opencv2/opencv.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>
// #include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/String.h>

#include <stdio.h>
#include <math.h>

#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>

#include <vector>
#include <set>
#include <map>
#include <queue>
#include <list>

#define og_pixel_width 160
#define og_pixel_depth 200

#define OCC_CELL_SIZE          5   //cm
#define OCC_WIDTH              800
#define OCC_WIDTH_CELL         OCC_WIDTH/OCC_CELL_SIZE
#define OCC_HEIGHT             1000
#define OCC_HEIGHT_CELL        OCC_HEIGHT/OCC_CELL_SIZE
#define CAM_DIST               50 //eh 45 até o fim da chapa mas estamos considerando um pouco das rodas da frente

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

std::vector<std::string> files_names;

// skeletonize
char occ_grid[og_pixel_depth][og_pixel_width];
char skeleton_og[og_pixel_depth][og_pixel_width];
char skeleton_og_bigger[og_pixel_depth+2][og_pixel_width+2];
int skeleton_to_clean[og_pixel_depth*og_pixel_width][2];
int skeleton_to_clean_counter;
int lut[256] =
      {0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0, 1, 3, 0, 0, 0, 0, 0, 0,
       0, 0, 2, 0, 2, 0, 3, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 2, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0,
       0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 2, 0, 0, 0, 3, 1,
       0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 1, 3, 0, 0,
       1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3,
       0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0};

char og_without_morph[og_pixel_depth][og_pixel_width];

char og_color[og_pixel_depth][og_pixel_width][3];
float distance[og_pixel_depth][og_pixel_width];
std::set<std::pair<int, int>> skeleton_all;
std::set<std::pair<int, int>> skeleton_rem;
int skeleton_counter = 0;

// read
cv::Mat og_color_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_8UC3, og_color);
cv::Mat og_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_8U, occ_grid);
cv::Mat og_without_morph_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_8U, og_without_morph);
cv::Mat distance_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_32F, distance);

int has_checkpoint = 0;
int checkpoint_i, checkpoint_j = 0;
std::vector<std::pair<int,int>> checkpoints;
std::vector<bool> has_checkpoints;
int iteration_number;


double dist_close_min = 10.0;

// start point
double min_dist_crawler = 999999;
std::pair<int,int> min_dist_crawler_pixel(0, 0);

// bfs
std::map<std::pair<int,int>, std::vector<std::pair<int,int>>> connections;
std::map<std::pair<int,int>, std::pair<int,int>> parent;
std::queue<std::pair<int,int>> queue;
std::set<std::pair<int,int>> skeleton_visited;

// min distance checkpoint pixel
double min_dist_checkpoint = 999999;
std::pair<int,int> min_dist_checkpoint_pixel(0, 0);

// max distance crawler pixel
double max_dist_crawler = -1;
std::pair<int,int> max_dist_crawler_pixel(0, 0);

// path
std::vector<std::pair<int,int>> path;

// extend
std::vector<std::pair<int,int>> extended_path;

// orientation
double orientaion_angle_rad;

// time
std::vector<std::chrono::microseconds> reset_time;
std::vector<std::chrono::microseconds> read_info_time;
std::vector<std::chrono::microseconds> get_og_time;
std::vector<std::chrono::microseconds> skeletonize_zs_time;
std::vector<std::chrono::microseconds> get_start_point_and_connections_time;
std::vector<std::chrono::microseconds> bfs_time;
std::vector<std::chrono::microseconds> get_min_or_max_pixel_time;
std::vector<std::chrono::microseconds> get_path_time;
std::vector<std::chrono::microseconds> get_extended_path_time;
std::vector<std::chrono::microseconds> total_time;

// planning
// construct the state space we are planning in
auto space(std::make_shared<ob::SE2StateSpace>());
// set the bounds for the R^2 part of SE(2)
ob::RealVectorBounds bounds(2);
// create a control space
auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
// set the bounds for the control space
ob::RealVectorBounds cbounds(2);
// construct an instance of  space information from this control space
auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
// construct an instance of  space information from this state space
oc::SimpleSetup ss(si);
// create a random start state
ob::ScopedState<ob::SE2StateSpace> start(space);
// create planner
auto planner(std::make_shared<oc::SST>(si));

double manhattan_dist;
double crawler_L = 40;
double goal_state_x, goal_state_z, goal_state_orientation;


void read_files_names_and_checkpoints(){
    const char* file_name = "checkpoint_file_list/list.txt"; 
    FILE *fp;
    fp = fopen(file_name, "r");
    char name [1000];
    int og_number, exp_number, cur_i, cur_j, id;
    for(;;){
        fscanf(fp, "%s", name);
        if(name[0] =='e'){
            id = 0;
            fscanf(fp, "%d\n", &exp_number);
            printf("exp_number=%d\n", exp_number);
        }else if(name[0] == 'o'){
            id = 0;
            fscanf(fp, "%d\n", &og_number);
            printf("og_number=%d\n", og_number);
        }else if(name[0] == 'h'){
            has_checkpoints.push_back(true);
            fscanf(fp, "\n");
            fscanf(fp, "%d %d\n",  &cur_i, &cur_j);
            checkpoints.push_back(std::make_pair(cur_i, cur_j));
            sprintf(name, "og_output_%02d_%03d_%02d", exp_number, og_number, id++);
            files_names.push_back(std::string(name));
        }else if(name[0] == 'd'){
            has_checkpoints.push_back(false);
            fscanf(fp, "\n");
            checkpoints.push_back(std::make_pair(0, 0));
            sprintf(name, "og_output_%02d_%03d_%02d", exp_number, og_number, id++);
            files_names.push_back(std::string(name));
        }else if(name[0] == '$'){
            break;
        }
    }
    fclose(fp);
}

void reset(){
    memset(distance, 0, sizeof(float)*og_pixel_depth*og_pixel_width);
    memset(occ_grid, 0, sizeof(char)*og_pixel_depth*og_pixel_width);
    memset(skeleton_og, 0, sizeof(char)*og_pixel_depth*og_pixel_width);
    skeleton_all.clear();
    skeleton_rem.clear();
    
    skeleton_counter = 0;
    has_checkpoint, checkpoint_i, checkpoint_j = 0;
    
    // start point
    min_dist_crawler = 999999;
    min_dist_crawler_pixel = std::pair<int,int>(0,0);
    
    // bfs
    connections.clear();
    parent.clear();
    if(!queue.empty()){
        printf("Error!!!\n");
    }
    skeleton_visited.clear();
    
    // min distance checkpoint pixel
    min_dist_checkpoint = 999999;
    min_dist_checkpoint_pixel = std::pair<int,int>(0, 0);

    // max distance crawler pixel
    max_dist_crawler = -1;
    max_dist_crawler_pixel = std::pair<int,int>(0, 0);
    
    // path
    path.clear();
    
    // extended_path
    extended_path.clear();
    
    // orientation
    orientaion_angle_rad = 0;
}

void read_og_from_file_and_set_checkpoint(std::string file_name){
    char buffer[1000];
    std::strcpy (buffer, (std::string("../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/") 
        + files_names[iteration_number].substr(0, files_names[iteration_number].size()-3) + std::string(".png")).c_str());
    printf("%s\n", buffer);
    // sprintf(buffer, "../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/og_output_20_135.png");
    og_color_mat = cv::imread(buffer, cv::IMREAD_COLOR);
    // get checkpoint
    has_checkpoint = has_checkpoints[iteration_number];
    checkpoint_i = checkpoints[iteration_number].first;
    checkpoint_j = checkpoints[iteration_number].second;
    
    cv::cvtColor(og_color_mat, og_without_morph_mat, cv::COLOR_RGB2GRAY);
    cv::threshold(og_without_morph_mat, og_without_morph_mat, 0, 1, cv::THRESH_BINARY);
}

void get_og(){
    cv::morphologyEx(og_without_morph_mat, og_mat, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
    cv::morphologyEx(og_mat, og_mat, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30,30)));
    cv::distanceTransform(og_mat, distance_mat, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

void skeletonize_zs(){
    int neighbors_case = 0;
    bool firts_pass;
    bool pixel_removed = true;
    int zs_it_counter = 0;
    for(int i=0;i<og_pixel_depth; i++){
        for(int j=0;j<og_pixel_width;j++){
            skeleton_og_bigger[i+1][j+1] = occ_grid[i][j];
        }
    }
    for(int i=0;i<og_pixel_depth+2; i++){
        skeleton_og_bigger[i][0] = 0;
        skeleton_og_bigger[i][og_pixel_width+1] = 0;
    }
    for(int j=0;j<og_pixel_width+2; j++){
        skeleton_og_bigger[0][j] = 0;
        skeleton_og_bigger[og_pixel_depth+1][j] = 0;
    }
    
    while(pixel_removed){
        pixel_removed = false;
        for(int sub_it = 0; sub_it < 2; sub_it++){
            skeleton_counter = 0;
            firts_pass = (sub_it == 0);
            for(int i=0;i<og_pixel_depth+2; i++){
                for(int j=0;j<og_pixel_width+2;j++){
                    if(skeleton_og_bigger[i][j]){
                        neighbors_case = lut[ 1*skeleton_og_bigger[i - 1][j - 1] +   2*skeleton_og_bigger[i - 1][ j] +
                                              4*skeleton_og_bigger[i - 1][j + 1] +   8*skeleton_og_bigger[i] [j + 1] +
                                             16*skeleton_og_bigger[i + 1][j + 1] +  32*skeleton_og_bigger[i + 1][ j] +
                                             64*skeleton_og_bigger[i + 1][j - 1] + 128*skeleton_og_bigger[i] [j - 1]];
                        if ((neighbors_case == 1 &&  firts_pass) || 
                            (neighbors_case == 2 && !firts_pass) || 
                            (neighbors_case == 3)){
                            skeleton_to_clean[skeleton_counter][0] = i;
                            skeleton_to_clean[skeleton_counter][1] = j;
                            skeleton_counter++; 
                            pixel_removed = true;
                        }
                    }
                }
            }
            for(int i=0;i<skeleton_counter;i++){
                skeleton_og_bigger[skeleton_to_clean[i][0]][skeleton_to_clean[i][1]] = 0;
            }
        }
        // printf("skeleton_couter: %d\n", skeleton_counter);
        zs_it_counter++;
    }
    // printf("zs_it_counter: %d\n", zs_it_counter);
    for(int i=0;i<og_pixel_depth; i++){
        for(int j=0;j<og_pixel_width;j++){
            skeleton_og[i][j] = skeleton_og_bigger[i+1][j+1];
            if(skeleton_og[i][j]){
                skeleton_all.insert(std::pair<int, int>(i, j));
                if(distance[i][j] >= dist_close_min){
                    skeleton_rem.insert(std::pair<int, int>(i, j));
                    // connections[std::pair<int, int>(i, j)] = std::vector<std::pair<int,int>> {};
                }
            }
        }
    }
}

void skeletonize_zs_test(){
    int neighbors_case = 0;
    bool firts_pass;
    bool pixel_removed = true;
    int zs_it_counter = 0;
    std::list<std::pair<int, int>> skeleton_active;
    for(int i=0;i<og_pixel_depth; i++){
        for(int j=0;j<og_pixel_width;j++){
            skeleton_og_bigger[i+1][j+1] = occ_grid[i][j];
            if(occ_grid[i][j]){
                skeleton_active.push_back(std::pair<int,int>(i,j));
            }
        }
    }
    for(int i=0;i<og_pixel_depth+2; i++){
        skeleton_og_bigger[i][0] = 0;
        skeleton_og_bigger[i][og_pixel_width+1] = 0;
    }
    for(int j=0;j<og_pixel_width+2; j++){
        skeleton_og_bigger[0][j] = 0;
        skeleton_og_bigger[og_pixel_depth+1][j] = 0;
    }
    
    while(pixel_removed){
        pixel_removed = false;
        for(int sub_it = 0; sub_it < 2; sub_it++){
            skeleton_counter = 0;
            firts_pass = (sub_it == 0);
            int i,j;
            for(auto it = skeleton_active.begin(); it!=skeleton_active.end();){
                i = it->first + 1;
                j = it->second + 1;
                if(skeleton_og_bigger[i][j]){
                    neighbors_case = lut[ 1*skeleton_og_bigger[i - 1][j - 1] +   2*skeleton_og_bigger[i - 1][ j] +
                                          4*skeleton_og_bigger[i - 1][j + 1] +   8*skeleton_og_bigger[i] [j + 1] +
                                         16*skeleton_og_bigger[i + 1][j + 1] +  32*skeleton_og_bigger[i + 1][ j] +
                                         64*skeleton_og_bigger[i + 1][j - 1] + 128*skeleton_og_bigger[i] [j - 1]];
                    if ((neighbors_case == 1 &&  firts_pass) || 
                        (neighbors_case == 2 && !firts_pass) || 
                        (neighbors_case == 3)){
                        skeleton_to_clean[skeleton_counter][0] = i;
                        skeleton_to_clean[skeleton_counter][1] = j;
                        skeleton_counter++; 
                        pixel_removed = true;
                        it = skeleton_active.erase(it);
                        continue;
                    }
                }
                ++it;
            }
            for(int i=0;i<skeleton_counter;i++){
                skeleton_og_bigger[skeleton_to_clean[i][0]][skeleton_to_clean[i][1]] = 0;
            }
        }
        // printf("skeleton_couter: %d\n", skeleton_counter);
        zs_it_counter++;
    }
    // printf("zs_it_counter: %d\n", zs_it_counter);
    memset(skeleton_og, 0, sizeof(char)*og_pixel_depth*og_pixel_width); 
    int i,j;
    for(auto it = skeleton_active.begin(); it!=skeleton_active.end();++it){
        i = it->first;
        j = it->second;
        skeleton_og[i][j] = 1;
        skeleton_all.insert(std::pair<int, int>(i, j));
        if(distance[i][j] >= dist_close_min){
            skeleton_rem.insert(std::pair<int, int>(i, j));
            // connections[std::pair<int, int>(i, j)] = std::vector<std::pair<int,int>> {};
        }
    }
    // for(int i=0;i<og_pixel_depth; i++){
    //     for(int j=0;j<og_pixel_width;j++){
    //         skeleton_og[i][j] = skeleton_og_bigger[i+1][j+1];
    //         if(skeleton_og[i][j]){
    //             skeleton_all.insert(std::pair<int, int>(i, j));
    //             if(distance[i][j] >= dist_close_min){
    //                 skeleton_rem.insert(std::pair<int, int>(i, j));
    //                 // connections[std::pair<int, int>(i, j)] = std::vector<std::pair<int,int>> {};
    //             }
    //         }
    //     }
    // }
}


inline double calculate_dist2crawler(std::pair<int,int> pixel){
    return ((og_pixel_width/2.0) - pixel.second)*((og_pixel_width/2.0) - pixel.second) + 
            (og_pixel_depth - pixel.first)*(og_pixel_depth - pixel.first);
}

inline double calculate_dist2checkpoint(std::pair<int,int> pixel){
    return (checkpoint_j - pixel.second)*(checkpoint_j - pixel.second)  +  (checkpoint_i - pixel.first)*(checkpoint_i - pixel.first);
}

void get_start_point_and_connections(){
    double cur_dist = 0;
    std::pair<int,int> cur_pixel(0, 0);
    std::pair<int,int> neighbor(0,0);
    for(auto it = skeleton_rem.begin(); it!=skeleton_rem.end(); ++it){
        // get start point
        cur_pixel = *it;
        cur_dist = calculate_dist2crawler(cur_pixel);
        if(cur_dist < min_dist_crawler){
            min_dist_crawler = cur_dist;
            min_dist_crawler_pixel = cur_pixel;
        }
        
        // get connections
        if (cur_pixel.first != 199){
            neighbor = std::make_pair(cur_pixel.first+1, cur_pixel.second);
            if (skeleton_rem.find(neighbor)!=skeleton_rem.end()){
                connections[cur_pixel].push_back(neighbor);
                connections[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.second != 199){
            neighbor = std::make_pair(cur_pixel.first, cur_pixel.second+1);
            if (skeleton_rem.find(neighbor)!=skeleton_rem.end()){
                connections[cur_pixel].push_back(neighbor);
                connections[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.first!=199 and cur_pixel.second!=199){
            neighbor = std::make_pair(cur_pixel.first+1, cur_pixel.second+1);
            if (skeleton_rem.find(neighbor)!=skeleton_rem.end()){
                connections[cur_pixel].push_back(neighbor);
                connections[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.first!=199 and cur_pixel.second!=0){
            neighbor = std::make_pair(cur_pixel.first+1, cur_pixel.second-1);
            if (skeleton_rem.find(neighbor)!=skeleton_rem.end()){
                connections[cur_pixel].push_back(neighbor);
                connections[neighbor].push_back(cur_pixel);
            }
        }
    }
}

void bfs(){
    std::pair<int,int> u, v;
    std::vector<std::pair<int,int>> cur_connections;
    skeleton_visited.insert(min_dist_crawler_pixel);
    parent[min_dist_crawler_pixel] = std::pair<int,int>(-1,-1);
    queue.push(min_dist_crawler_pixel);
    while(!queue.empty()){
        u = queue.front();
        queue.pop();
        cur_connections = connections[u];
        for(auto it=cur_connections.begin(); it!=cur_connections.end(); ++it){
            v = *it;
            if(skeleton_visited.find(v)==skeleton_visited.end()){
                skeleton_visited.insert(v);
                queue.push(v);
                parent[v] = u;
            }
        }
    }
}

void get_min_dist_checkpoint_pixel(){
    double cur_dist = 0;
    std::pair<int,int> cur_pixel(0, 0);
    for(auto it = skeleton_visited.begin(); it!=skeleton_visited.end(); ++it){
        cur_pixel = *it;
        cur_dist = calculate_dist2checkpoint(cur_pixel);
        if(cur_dist < min_dist_checkpoint){
            min_dist_checkpoint = cur_dist;
            min_dist_checkpoint_pixel = cur_pixel;
        }
    }
}

double get_median_contour_distance(){
    std::vector<int> contour_distances;
    std::pair<int,int> cur_pixel;
    for(auto it=skeleton_visited.begin(); it!=skeleton_visited.end(); ++it){
        cur_pixel = *it;
        contour_distances.push_back(distance[cur_pixel.first][cur_pixel.second]);
    }
    std::sort(contour_distances.begin(), contour_distances.end());
    return contour_distances[(int)(contour_distances.size()/2.0)];
}


void get_max_dist_crawler_pixel(){
    double cur_dist = 0;
    std::pair<int,int> cur_pixel(0, 0);
    double median = get_median_contour_distance();
    for(auto it=skeleton_visited.begin(); it!=skeleton_visited.end(); ++it){
        cur_pixel = *it;
        if(distance[cur_pixel.first][cur_pixel.second] > median){
            cur_dist = calculate_dist2crawler(cur_pixel);
            if(cur_dist > max_dist_crawler){
                max_dist_crawler = cur_dist;
                max_dist_crawler_pixel = cur_pixel;
            }
        }
    }
}

void get_min_or_max_pixel(){
    if(has_checkpoint){
        get_min_dist_checkpoint_pixel();
    }else{
        get_max_dist_crawler_pixel();
    }
}

void get_path(){
    std::pair<int,int> cur_parent;
    if(has_checkpoint){
        path.push_back(min_dist_checkpoint_pixel);
    }else{
        path.push_back(max_dist_crawler_pixel);
    }
    cur_parent = parent[path[0]];
    while(cur_parent.first != -1 && cur_parent.second != -1){
        path.push_back(cur_parent);
        cur_parent = parent[cur_parent];
    }
    
    // put reverse path in extended path
    int len_path = path.size();
    extended_path.reserve(path.size());
    for(int i=0;i<len_path;i++){
        extended_path.push_back(path[0]); //push anything to increase size
    }
    for(int i = 0; i<len_path/2; i++){
        extended_path[i] = path[len_path-1-i];
        extended_path[len_path-1-i] = path[i];
    }
}

void get_extended_path(){
    if(!has_checkpoint){
        return ;
    }
    std::pair<int,int> it_min_dist_checkpoint_pixel = min_dist_checkpoint_pixel;
    int neighbors [8][2] = { {-1,-1}, {-1,0}, {-1,+1}, {0,+1}, {+1,+1}, {+1,0}, {+1,-1}, {0,-1}};
    double cur_min_distance;
    double cur_distance;
    std::pair<int,int> sub_it_min_dist_checkpoint_pixel, cur_neighbor_pixel;
    for(;;){
        cur_min_distance = calculate_dist2checkpoint(it_min_dist_checkpoint_pixel);
        sub_it_min_dist_checkpoint_pixel = it_min_dist_checkpoint_pixel;
        for(int i=0;i<8;i++){
            cur_neighbor_pixel = std::make_pair(it_min_dist_checkpoint_pixel.first + neighbors[i][0], 
                it_min_dist_checkpoint_pixel.second + neighbors[i][1]);
            if(cur_neighbor_pixel.first < 0 || cur_neighbor_pixel.first >= og_pixel_depth || 
                cur_neighbor_pixel.second < 0 || cur_neighbor_pixel.second >= og_pixel_width || 
                distance[cur_neighbor_pixel.first][cur_neighbor_pixel.second] < dist_close_min){
                    continue;
            } else {
                cur_distance = calculate_dist2checkpoint(cur_neighbor_pixel);
                if(cur_distance < cur_min_distance) {
                    cur_min_distance = cur_distance;
                    sub_it_min_dist_checkpoint_pixel = cur_neighbor_pixel;
                }
            }
        }
        if(sub_it_min_dist_checkpoint_pixel == it_min_dist_checkpoint_pixel){
            break;
        } else{
            it_min_dist_checkpoint_pixel = sub_it_min_dist_checkpoint_pixel;
            extended_path.push_back(sub_it_min_dist_checkpoint_pixel);
        }
    }
}

void get_orientation(){
    double orientation_z = extended_path[(int) extended_path.size()*0.75].first - extended_path[extended_path.size()-1].first;
    double orientation_x = extended_path[extended_path.size()-1].second - extended_path[(int) extended_path.size()*0.75].second;
    orientaion_angle_rad = atan2(orientation_z, orientation_x);
}

void print_time_it(double div){
    std::cout << "\nTimes:::::::" << std::endl;
    std::cout << "Time taken by reset: " << reset_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by read_info: " << read_info_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by get_og: " << get_og_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by skeletonize_zs: " << skeletonize_zs_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by get_start_point_and_connections: " << get_start_point_and_connections_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by bfs: " << bfs_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by get_min_or_max_pixel: " << get_min_or_max_pixel_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by get_path: " << get_path_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    std::cout << "Time taken by get_extended_path: " << get_extended_path_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
    if (div < 1.1){
        total_time.push_back(reset_time[reset_time.size()-1] + read_info_time[reset_time.size()-1] +
            get_og_time[reset_time.size()-1] + skeletonize_zs_time[reset_time.size()-1] + 
            get_start_point_and_connections_time[reset_time.size()-1] + bfs_time[reset_time.size()-1] + 
            get_min_or_max_pixel_time[reset_time.size()-1] + get_path_time[reset_time.size()-1] +
            get_extended_path_time[reset_time.size()-1]);
    }
    std::cout << "Total time: " << total_time[reset_time.size()-1].count()/div << " microseconds" << std::endl;
}

void print_time_all(){
    reset_time.push_back(reset_time[0]);
    read_info_time.push_back(read_info_time[0]);
    get_og_time.push_back(get_og_time[0]);
    skeletonize_zs_time.push_back(skeletonize_zs_time[0]);
    get_start_point_and_connections_time.push_back(get_start_point_and_connections_time[0]);
    bfs_time.push_back(bfs_time[0]);
    get_min_or_max_pixel_time.push_back(get_min_or_max_pixel_time[0]);
    get_path_time.push_back(get_path_time[0]);
    get_extended_path_time.push_back(get_extended_path_time[0]);
    
    total_time.push_back(total_time[0]);
    
    for(int i=1;i<iteration_number+1;i++){
        reset_time[iteration_number+1] += reset_time[i];
        read_info_time[iteration_number+1] += read_info_time[i];
        get_og_time[iteration_number+1] += get_og_time[i];
        skeletonize_zs_time[iteration_number+1] += skeletonize_zs_time[i];
        get_start_point_and_connections_time[iteration_number+1] += get_start_point_and_connections_time[i];
        bfs_time[iteration_number+1] += bfs_time[i];
        get_min_or_max_pixel_time[iteration_number+1] += get_min_or_max_pixel_time[i];
        get_path_time[iteration_number+1] += get_path_time[i];
        get_extended_path_time[iteration_number+1] += get_extended_path_time[i];
        total_time[iteration_number+1] += total_time[i];
    }
   print_time_it(iteration_number+1);
}

void save_results_skel_gs(){
    char file_name_c [1000];
    if(files_names.size()){
        std::strcpy (file_name_c, (std::string("skel_gs_results/") + files_names[iteration_number] 
        + std::string(".txt")).c_str());
    }else{
        sprintf(file_name_c, "%05d", iteration_number);
         std::strcpy (file_name_c, (std::string("skel_gs_results/") + std::string(file_name_c) 
        + std::string(".txt")).c_str());
    }
    FILE *fp;
    fp = fopen(file_name_c, "w");
    char name [1000];
    
    fprintf(fp,"Og without morph:\n");
    for(int i=0;i<og_pixel_depth;i++){
        for(int j=0;j<og_pixel_width;j++){
            fprintf(fp,"%d ", og_without_morph[i][j]);
        }
        fprintf(fp,"\n");
    }
    
    fprintf(fp,"Distance:\n");
    for(int i=0;i<og_pixel_depth;i++){
        for(int j=0;j<og_pixel_width;j++){
            fprintf(fp,"%lf ", distance[i][j]);
        }
        fprintf(fp,"\n");
    }
    fprintf(fp,"Skeleton:\n");
    for(int i=0;i<og_pixel_depth;i++){
        for(int j=0;j<og_pixel_width;j++){
            fprintf(fp,"%d ", skeleton_og[i][j]);
        }
        fprintf(fp,"\n");
    }
    fprintf(fp,"Skeleton_rem:\n");
    fprintf(fp, "%lu\n", skeleton_rem.size());
    for(auto it=skeleton_rem.begin(); it!=skeleton_rem.end(); ++it){
        fprintf(fp, "%d %d\n", it->first, it->second);
    }
    
    fprintf(fp,"Start point:\n");
    fprintf(fp,"%d %d\n", min_dist_crawler_pixel.first, min_dist_crawler_pixel.second);
    
    fprintf(fp,"Has checkpoint:\n");
    fprintf(fp,"%d\n", has_checkpoint);
    
    fprintf(fp,"Checkpoint:\n");
    fprintf(fp,"%d %d\n", checkpoint_i, checkpoint_j);
    
    fprintf(fp,"Final point path:\n");
    int path_len = path.size();
    fprintf(fp, "%d %d\n", path[0].first, path[0].second);
    
    fprintf(fp,"Path:\n");
    fprintf(fp, "%lu\n", path.size());
    for(int i=path_len-1;i>=0;i--){
        fprintf(fp, "%d %d\n", path[i].first, path[i].second);
    }
    
    fprintf(fp,"Extended path:\n");
    int extended_path_len = extended_path.size();
    fprintf(fp, "%lu\n", extended_path.size());
    for(int i=0;i<extended_path_len;i++){
        fprintf(fp, "%d %d\n", extended_path[i].first, extended_path[i].second);
    }
    
    fprintf(fp,"Goal state:\n");
    fprintf(fp, "%d %d %lf\n", extended_path[extended_path_len-1].first, extended_path[extended_path_len-1].second, orientaion_angle_rad);
    
    fclose(fp);
}

// ===========================Planning=============================
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
    
    if(occ_grid[z][x] && distance[z][x] >= 10.0){
        // std::cout << "**valid**\n";
        return true;
    } else{
        // std::cout << "**not valid**\n";
        return false;
    }
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
    // obj->setCostThreshold(ob::Cost(manhattan_dist));
    return obj;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double yaw = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * cos(yaw),
        pos[1] + ctrl[0] * sin(yaw));
        
    // std::cout << "old: " << pos[0] << "; new: " << pos[0] + ctrl[0] * cos(yaw) << std::endl;
    // std::cout << "old: " << pos[1] << "; new: " << pos[1] + ctrl[0] * sin(yaw) << std::endl;
    
    double new_yaw = yaw;
    // std::cout << "yaw: " << yaw <<std::endl; 
    new_yaw += (ctrl[0]/crawler_L)*tan(ctrl[1]);
    // std::cout << "new_yaw: " << new_yaw <<std::endl;
    
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


void setup_planning(){
    
    
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
    double new_yaw_weight = 5.0/degree2rad(2.5);
    std::cout << "New yaw weight:" << std::endl;
    std::cout << new_yaw_weight << std::endl;
    space->setSubspaceWeight(1, new_yaw_weight);
    
    //v = 100cm/s, mas vamos considerar que cada step de motion tem 0.05s então vamos passar o deslocamento max na bound superior
    // double vel_max = 50;
    double vel_max = 50;
    // double step_time = 0.05;
    // double step_time = 0.2;
    double step_time = 0.3;
    // cbounds.setLow(0, 0);
    cbounds.setLow(0, vel_max*step_time);
    // 0.5s - 90
    // step_time - x
    // 0.5*x = 90*step_time
    // x = (90*step_time)/0.5
    double rot_bound = (60.0*step_time)/1;
    cbounds.setHigh(0, vel_max*step_time);
    // apesar de o alcance de rotacao das rodas ser de [-45, 45] vamos considerar menos, pois imaginamos que a roda não consegue
    // rotacionar tudo isso em um intervalo de tempo tão pequeno - vai virar no add no yaw no máximo 0.6265913968685753 graus por delta_t
    // cbounds.setLow(1, degree2rad(-rot_bound));
    // cbounds.setHigh(1, degree2rad(rot_bound));
    cbounds.setLow(1, degree2rad(-12));
    cbounds.setHigh(1, degree2rad(12));
    cspace->setBounds(cbounds);

    // set state validity checking for this space
    ss.getSpaceInformation()->setStateValidityChecker(isStateValid);
    
    // set si/space parameters
    ss.getSpaceInformation()->setStatePropagator(propagate);
    ss.getSpaceInformation()->setMinMaxControlDuration(2, 5);
    ss.getSpaceInformation()->setPropagationStepSize(5); //deixar sempre igual ao maxControlDuration para não influenciar em nada
    
    // ss.getSpaceInformation()->setup();

    start->setXY(0, -CAM_DIST);
    start->setYaw(degree2rad(90));

    ss.setOptimizationObjective(getThresholdPathLengthObj(ss.getSpaceInformation()));
    // std::cout << "Threshold: "<<ss.getOptimizationObjective()->getCostThreshold() <<std::endl;
    
    // sst params
    ompl::base::ParamSet& params = planner->params();
    double   sst_selection_radius  = 25;
    double   sst_pruning_radius = 5;
    params.setParam("selection_radius", ompl::toString(sst_selection_radius));
    params.setParam("pruning_radius", ompl::toString(sst_pruning_radius));

    ss.setPlanner(planner);
}

void save_results_planning(const ob::PlannerPtr &planner)
{
    auto pdef = planner->getProblemDefinition();
    ob::PathPtr path = pdef->getSolutionPath();
    
    char string_buffer [200];
    
    sprintf(string_buffer, "solution_paths/run_%03d_%s.txt", iteration_number, planner->getName().c_str());
    
    std::ofstream myfile;
    myfile.open(string_buffer);
    myfile << OCC_HEIGHT_CELL << " " << OCC_WIDTH_CELL << std::endl;
    myfile << goal_state_x << " " << goal_state_z << " " << goal_state_orientation << std::endl;
    for(int i=0;i<OCC_HEIGHT_CELL;i++){
        for(int j=0;j<OCC_WIDTH_CELL;j++){
            myfile << (int) occ_grid[i][j] << " ";
        }
        myfile << std::endl;
    }
    path->print(myfile);
    myfile.close();
}

void plan(){
    // read info
    goal_state_z = get_pos_from_grid_z(extended_path[extended_path.size()-1].first);
    goal_state_x = get_pos_from_grid_x(extended_path[extended_path.size()-1].second);
    goal_state_orientation = orientaion_angle_rad;
    
    // reset
    planner->clear();
    ss.getProblemDefinition()->clearSolutionPaths();
    ss.getProblemDefinition()->clearSolutionPaths();
    ss.getProblemDefinition()->clearGoal();
    
    ob::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setXY(goal_state_x, goal_state_z);
    goal->setYaw(goal_state_orientation);
    
    // set the start and goal states and threashold
    ss.setStartAndGoalStates(start, goal, 60);
    ss.setup();
    // print the planner settings
    std::cout << "========START PLANNER SETTINGS========" << std::endl;
    planner->printSettings(std::cout);
    std::cout << "========END PLANNER SETTINGS========\n" << std::endl;
    
    
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);
    printf("oi3\n");fflush(stdout);
    
    if (solved){
        save_results_planning(ss.getPlanner());
    } else{
        std::cout << "No solution found" << std::endl;
    }
}
