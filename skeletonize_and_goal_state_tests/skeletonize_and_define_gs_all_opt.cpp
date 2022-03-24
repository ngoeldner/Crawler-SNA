// g++ skeletonize_and_define_gs_all_opt.cpp -o skeletonize_and_define_gs_all_opt `pkg-config --cflags --libs opencv4`
#include "skeletonize_and_define_gs.h"

int main(){
    read_files_names_and_checkpoints();
    int len_files_names = files_names.size();
    std::chrono::high_resolution_clock::time_point start, stop;
    for(int it1 = 0; it1  < len_files_names; it1++){
        iteration_number = it1;
            start = std::chrono::high_resolution_clock::now();
        // printf("%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);fflush(stdout);
        reset();
            stop = std::chrono::high_resolution_clock::now();
            reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        read_og_from_file_and_set_checkpoint(files_names[it1]);
            stop = std::chrono::high_resolution_clock::now();
            read_info_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        get_og();
            stop = std::chrono::high_resolution_clock::now();
            get_og_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        skeletonize_zs_test();
            stop = std::chrono::high_resolution_clock::now();
            skeletonize_zs_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
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
            
            start = std::chrono::high_resolution_clock::now();
        get_orientation();
            stop = std::chrono::high_resolution_clock::now();
            // get_orientation_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
        save_results();
            print_time_it(1);
    }
    print_time_all();
    
    return 0;
}        

