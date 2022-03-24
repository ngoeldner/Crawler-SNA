// g++ skeletonize_and_define_gs_all_opt.cpp -o skeletonize_and_define_gs_all_opt `pkg-config --cflags --libs opencv4`
#include "skeletonize_and_define_gs.h"
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080

int server_fd, new_socket;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);
const char *hello = "Hello from server";
int checkpoint_info[3];
    
void setup_socket(){
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
       
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, 
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

void receive_og_from_socket(){
    send(new_socket , hello , strlen(hello) , 0 );
    int valread;
    valread = read( new_socket , og_without_morph, og_pixel_depth*og_pixel_width);
    
    // for(int i=0;i<og_pixel_depth;i++){
    //     for(int j=0;j<og_pixel_width;j++){
    //         if(skeleton_og[i][j] != 0){
    //             printf("ok");
    //         }
    //     }
    // }
    // printf("valread: %d\n", valread);
    if(valread!=og_pixel_depth*og_pixel_width){
        printf("ERRO!!!!!!!!\n");
    }
    valread = read( new_socket , checkpoint_info, sizeof(int)*3);
    has_checkpoint = checkpoint_info[0];
    checkpoint_i = checkpoint_info[1];
    checkpoint_j = checkpoint_info[2];
    // printf("Checkpoint:%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);
    // printf("valread: %d\n", valread);
    if(valread!=sizeof(int)*3){
        printf("ERRO!!!!!!!!\n");
    }
}

int main(){
    setup_socket();
    int iterations_limit [0];
    read(new_socket, iterations_limit, sizeof(int));
    printf("iterations_limit: %d\n", iterations_limit[0]);
    std::chrono::high_resolution_clock::time_point start, stop;
    // send(new_socket , hello , strlen(hello) , 0 );
    for(int it1 = 0; it1  < iterations_limit[0]; it1++){
        iteration_number = it1;
            start = std::chrono::high_resolution_clock::now();
        // printf("%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);fflush(stdout);
        reset();
            stop = std::chrono::high_resolution_clock::now();
            reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            start = std::chrono::high_resolution_clock::now();
        receive_og_from_socket();
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
        // printf("Path::%d %d\n", path[0].first, path[0].second);
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

