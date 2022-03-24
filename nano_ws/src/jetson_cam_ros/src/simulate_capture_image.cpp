// g++ client_send_ogs_checkpoints.cpp -o client_send_ogs_checkpoints `pkg-config --cflags --libs opencv4`

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include <string>
#include <vector>

#define PORT 8080
#define og_pixel_depth 200
#define og_pixel_width 160

int checkpoint_info[3];
int iteration_number;
std::vector<std::string> files_names;

char og_without_morph[og_pixel_depth][og_pixel_width];
char og_color[og_pixel_depth][og_pixel_width][3];


cv::Mat og_color_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_8UC3, og_color);
cv::Mat og_without_morph_mat = cv::Mat(og_pixel_depth, og_pixel_width, CV_8U, og_without_morph);


std::string dir = "../../../og_images/";
void read_files_names(){
    const char* file_name = "list.txt"; 
    FILE *fp;
    fp = fopen(file_name, "r");
    char name [1000];
    int og_number, exp_number, cur_i, cur_j, id;
    for(;;){
        fscanf(fp, "%s\n", name);
        if(name[0] == '$'){
            break;
        }
        files_names.push_back(std::string(name));
    }
    fclose(fp);
}

void read_og_from_file(std::string file_name){
    char buffer[1000];
    std::strcpy (buffer, (dir + files_names[iteration_number].substr(0, files_names[iteration_number].size()-3) + std::string("png")).c_str());
    printf("%s\n", buffer);
    // sprintf(buffer, "../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/og_output_20_135.png");
    og_color_mat = cv::imread(buffer, cv::IMREAD_COLOR);
}
   
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
    if(inet_pton(AF_INET, "192.168.10.1", &serv_addr.sin_addr)<=0) 
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    
    read_files_names();
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    
    
    
    int len_files_names = files_names.size();
    send(sock , &len_files_names , sizeof(int) , 0 );
    for(int it1 = 0; it1  < len_files_names; it1++){
        iteration_number = it1;
        read_og_from_file(files_names[it1]);
        
        cv::cvtColor(og_color_mat, og_without_morph_mat, cv::COLOR_RGB2GRAY);
        cv::threshold(og_without_morph_mat, og_without_morph_mat, 0, 1, cv::THRESH_BINARY);
        valread = read( sock , buffer, 1024);
        printf("%s\n",buffer );
    
        send(sock , og_without_morph , 200*160 , 0 );
        printf("sent1\n");
        checkpoint_info[0] = 0;
        checkpoint_info[1] = 0;
        checkpoint_info[2] = 0;
        send(sock, checkpoint_info, 3*sizeof(int), 0);
        printf("sent2\n");
        usleep(400000);
        
    }
    usleep(10000000);
    return 0;
}