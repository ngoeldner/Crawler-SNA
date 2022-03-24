// g++ client_send_ogs_checkpoints.cpp -o client_send_ogs_checkpoints `pkg-config --cflags --libs opencv4`

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 8080

#include "skeletonize_and_define_gs.h"

int checkpoint_info[3];


   
int main(int argc, char const *argv[])
{
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
       
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    
    read_files_names_and_checkpoints();
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    
    
    
    int len_files_names = files_names.size();
    send(sock , &len_files_names , sizeof(int) , 0 );
    // std::chrono::high_resolution_clock::time_point start, stop;
    for(int it1 = 0; it1  < len_files_names; it1++){
        iteration_number = it1;
            // start = std::chrono::high_resolution_clock::now();
        // printf("%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);fflush(stdout);
        reset();
            // stop = std::chrono::high_resolution_clock::now();
            // reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
            
            // start = std::chrono::high_resolution_clock::now();
        read_og_from_file_and_set_checkpoint(files_names[it1]);
        
        cv::cvtColor(og_color_mat, og_mat, cv::COLOR_RGB2GRAY);
        cv::threshold(og_mat, og_mat, 0, 1, cv::THRESH_BINARY);
            // stop = std::chrono::high_resolution_clock::now();
            // read_info_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        valread = read( sock , buffer, 1024);
        printf("%s\n",buffer );
    
        send(sock , skeleton_og , 200*160 , 0 );
        checkpoint_info[0] = has_checkpoint;
        checkpoint_info[1] = checkpoint_i;
        checkpoint_info[2] = checkpoint_j;
        send(sock, checkpoint_info, 3*sizeof(int), 0);
        usleep(400000);
        
    }
    return 0;
}