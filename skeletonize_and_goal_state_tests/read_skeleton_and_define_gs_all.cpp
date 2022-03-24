#include <stdio.h>
#include <math.h>

#include <cstring>
#include <string>
#include <iostream>
#include <algorithm>

#include <chrono>

#include <vector>
#include <set>
#include <map>
#include <queue>

#define og_pixel_width 160
#define og_pixel_depth 200

typename std::chrono::microseconds time_unit;

std::vector<std::string> files_names;


double distance[og_pixel_depth][og_pixel_width];
int skeleton[og_pixel_depth][og_pixel_width];
std::set<std::pair<int, int>> skeleton_all;
std::set<std::pair<int, int>> skeleton_rem;
int skeleton_counter = 0;

int has_checkpoint = 0;
int checkpoint_i, checkpoint_j = 0;

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
std::vector<std::chrono::microseconds> get_start_point_and_connections_time;
std::vector<std::chrono::microseconds> bfs_time;
std::vector<std::chrono::microseconds> get_min_or_max_pixel_time;
std::vector<std::chrono::microseconds> get_path_time;
std::vector<std::chrono::microseconds> get_extended_path_time;
std::vector<std::chrono::microseconds> total_time;

void read_files_names(){
    const char* file_name = "to_c/names.txt"; 
    FILE *fp;
    fp = fopen(file_name, "r");
    char name [1000];
    for(;;){
        fscanf(fp, "%s\n", name);
        if(name[0] == '$'){
            break;
        }
        files_names.push_back(std::string(name));
        std::cout << files_names[files_names.size()-1] << "\n"; 
    }
    fclose(fp);
}

void reset(){
    memset(distance, 0, sizeof(double)*og_pixel_depth*og_pixel_width);
    memset(skeleton, 0, sizeof(int)*og_pixel_depth*og_pixel_width); 
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

void read_info(std::string file_name){
    char file_name_c [1000];
    std::strcpy (file_name_c, (std::string("to_c/") + file_name).c_str());
    FILE *fp;
    fp = fopen(file_name_c, "r");
    char buffer [10000];
    fgets(buffer, 10000, fp); // width and depth
    fgets(buffer, 10000, fp); // distance:
    for(int i=0; i<og_pixel_depth; i++){
        for(int j=0;j<og_pixel_width; j++){
            fscanf(fp, "%lf", &distance[i][j]);
        }
    }
    fgets(buffer, 10000, fp); //  \n
    fgets(buffer, 10000, fp); // skeleton:
    fscanf(fp, "%d\n", &skeleton_counter);
    int pos_i, pos_j;
    for(int i=0; i<skeleton_counter; i++){
        fscanf(fp, "%d %d\n", &pos_i, &pos_j);
        skeleton[pos_i][pos_j] = 1;
        skeleton_all.insert(std::pair<int, int>(pos_i, pos_j));
        if(distance[pos_i][pos_j] >= dist_close_min){
            skeleton_rem.insert(std::pair<int, int>(pos_i, pos_j));
            // connections[std::pair<int, int>(pos_i, pos_j)] = std::vector<std::pair<int,int>> {};
        }
    }
    fscanf(fp, "has_checkpoint = %d\n", &has_checkpoint);
    if(has_checkpoint){
        fscanf(fp, "checkpoint_i = %d\n", &checkpoint_i);
        fscanf(fp, "checkpoint_j = %d\n", &checkpoint_j);
    }
    fclose(fp);
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
    
    // reverse path and put in extended path
    int len_path = path.size();
    extended_path.reserve(path.size());
    for(int i = 0; i<len_path/2; i++){
        extended_path[i] = path[len_path-1-i];
        extended_path[len_path-1-i] = path[i];;
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

void print_time_it(){
    std::cout << "\nTimes:::::::" << std::endl;
    std::cout << "Time taken by reset: " << reset_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by read_info: " << read_info_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by get_start_point_and_connections: " << get_start_point_and_connections_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by bfs: " << bfs_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by get_min_or_max_pixel: " << get_min_or_max_pixel_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by get_path: " << get_path_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    std::cout << "Time taken by get_extended_path: " << get_extended_path_time[reset_time.size()-1].count() << " microseconds" << std::endl;
    total_time.push_back(reset_time[reset_time.size()-1] + read_info_time[reset_time.size()-1] + 
        get_start_point_and_connections_time[reset_time.size()-1] + bfs_time[reset_time.size()-1] + 
        get_min_or_max_pixel_time[reset_time.size()-1] + get_path_time[reset_time.size()-1] +
        get_extended_path_time[reset_time.size()-1]);
    std::cout << "Total time: " << total_time[reset_time.size()-1].count() << " microseconds" << std::endl;
}

void print_time_all(){
    reset_time.push_back(reset_time[0]);
    read_info_time.push_back(read_info_time[0]);
    get_start_point_and_connections_time.push_back(get_start_point_and_connections_time[0]);
    bfs_time.push_back(bfs_time[0]);
    get_min_or_max_pixel_time.push_back(get_min_or_max_pixel_time[0]);
    get_path_time.push_back(get_path_time[0]);
    get_extended_path_time.push_back(get_extended_path_time[0]);
    
    total_time.push_back(total_time[0]);
    
    for(int i=1;i<files_names.size();i++){
        reset_time[files_names.size()] += reset_time[i];
        read_info_time[files_names.size()] += read_info_time[i];
        get_start_point_and_connections_time[files_names.size()] += get_start_point_and_connections_time[i];
        bfs_time[files_names.size()] += bfs_time[i];
        get_min_or_max_pixel_time[files_names.size()] += get_min_or_max_pixel_time[i];
        get_path_time[files_names.size()] += get_path_time[i];
        get_extended_path_time[files_names.size()] += get_extended_path_time[i];
        total_time[files_names.size()] += total_time[i];
    }
    std::cout << "\nTimes:::::::" << std::endl;
    std::cout << "Time taken by reset: " << reset_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by read_info: " << read_info_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by get_start_point_and_connections: " << 
    get_start_point_and_connections_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by bfs: " << bfs_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by get_min_or_max_pixel: " << 
    get_min_or_max_pixel_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by get_path: " << get_path_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Time taken by get_extended_path_time: " << 
    get_extended_path_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
    std::cout << "Total time: " << total_time[files_names.size()].count()/(double)files_names.size() << " microseconds" << std::endl;
}

int main(){
    read_files_names();
    int len_files_names = files_names.size();
    std::chrono::high_resolution_clock::time_point start, stop;
    for(int it1 = 0; it1  < len_files_names; it1++){
            start = std::chrono::high_resolution_clock::now();
        // printf("%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);fflush(stdout);
        reset();
            stop = std::chrono::high_resolution_clock::now();
            reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        read_info(files_names[it1]);
            stop = std::chrono::high_resolution_clock::now();
            read_info_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        get_start_point_and_connections();
            stop = std::chrono::high_resolution_clock::now();
            get_start_point_and_connections_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        bfs();
            stop = std::chrono::high_resolution_clock::now();
            bfs_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            
            start = std::chrono::high_resolution_clock::now();
        get_min_or_max_pixel();
            stop = std::chrono::high_resolution_clock::now();
            get_min_or_max_pixel_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        get_path();
            stop = std::chrono::high_resolution_clock::now();
            get_path_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        
            start = std::chrono::high_resolution_clock::now();
        get_extended_path();
            stop = std::chrono::high_resolution_clock::now();
            get_extended_path_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            print_time_it();
            
            start = std::chrono::high_resolution_clock::now();
        get_orientation();
            stop = std::chrono::high_resolution_clock::now();
            // get_orientation_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
    }
    print_time_all();
    
    return 0;
}        

