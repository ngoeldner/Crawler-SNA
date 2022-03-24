// g++ skeletonize_and_define_gs_socket_threads.cpp -o skeletonize_and_define_gs_socket_threads `pkg-config --cflags --libs opencv4` -pthread
#include "skeletonize_and_define_gs.h"
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>

#define PORT 8080

int server_fd, new_socket;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);
const char *hello = "Hello from server";
int checkpoint_info[3];
char skeleton_og_read[og_pixel_depth][og_pixel_width];
int has_checkpoint_read, checkpoint_i_read, checkpoint_j_read;
int iterations_limit [0];
bool server_executou = false;
bool copier_executou = false;
bool copier_tried = false;
bool terminou = false;
    
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
    int valread_total = 0;
    // printf("receiving og\n");
    while(valread_total!=og_pixel_depth*og_pixel_width){
        // printf("&skeleton_og_read[0][0]:%p\n", &skeleton_og_read[0][0]);
        // printf("&skeleton_og_read[0][0]+valread_total:%p\n", &skeleton_og_read[0][0]+valread_total);
        valread = read( new_socket , &skeleton_og_read[0][0] + valread_total, og_pixel_depth*og_pixel_width - valread_total);
        if(valread==-1){
            int errsv = errno;
            printf("read failed: %d\n", errsv);
        }
        valread_total += valread;
        // printf("valread:%d\n", valread);
        // printf("valread_total:%d\n", valread_total);
    }
    // printf("receiving og2\n");
    
    // for(int i=0;i<og_pixel_depth;i++){
    //     for(int j=0;j<og_pixel_width;j++){
    //         if(skeleton_og[i][j] != 0){
    //             printf("ok");
    //         }
    //     }
    // }
    // printf("valread: %d\n", valread);
    if(valread_total!=og_pixel_depth*og_pixel_width){
        printf("ERRO!!!!!!!!\n");
        printf("valread_total:%d\n", valread_total);
    }
    // printf("receiving og3\n");
    valread_total = 0;
    while(valread_total!=sizeof(int)*3){
        valread = read( new_socket , &checkpoint_info[0] + valread_total, sizeof(int)*3 - valread_total);
        valread_total += valread;
    }
    // printf("receiving og4\n");
    has_checkpoint_read = checkpoint_info[0];
    checkpoint_i_read = checkpoint_info[1];
    checkpoint_j_read = checkpoint_info[2];
    // printf("Checkpoint:%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);
    // printf("valread: %d\n", valread);
    if(valread_total!=sizeof(int)*3){
        printf("ERRO!!!!!!!!\n");
        printf("valread_total:%d\n", valread_total);
    }
    // printf("receiving og5\n");
}

pthread_t server_thread, processor_thread, copier_thread;
pthread_mutex_t server_mutex, processor_mutex;
sem_t sem_server, sem_copier, sem_processor;


// executar se copier nao estiver executando
void *server_function(void * arg){
    std::chrono::high_resolution_clock::time_point start, stop;
    for(int it1 = 0; it1  < iterations_limit[0]; it1++){
        sem_wait(&sem_server);
        pthread_mutex_lock(&server_mutex);
        printf("server_function started %d\n", it1);fflush(stdout);
            
            // start = std::chrono::high_resolution_clock::now();
        receive_og_from_socket();
            // stop = std::chrono::high_resolution_clock::now();
            // read_info_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        
        if(it1==iterations_limit[0]-1){
                terminou = true;
        }
        printf("server_function finished\n");fflush(stdout);
        pthread_mutex_unlock(&server_mutex);
        printf("server_function finished1\n");fflush(stdout);
        sem_post(&sem_copier);        
        
        
    }
    return 0;
}



// executar se server nao estiver executando e se processor nao estiver executando
void *copier_function(void * arg){
    std::chrono::high_resolution_clock::time_point start, stop;
    bool server_lock_acq, processor_lock_acq;
    while(true){
        while(true){
            // copier_tried = true;
            sem_wait(&sem_copier);
            server_lock_acq = false;
            processor_lock_acq = false;
            if(pthread_mutex_trylock(&server_mutex) ==0 ){
                printf("server_lock_acq\n");fflush(stdout);
                server_lock_acq = true;
            }
            if(pthread_mutex_trylock(&processor_mutex) ==0 ){
                printf("processor_lock_acq\n");fflush(stdout);
                processor_lock_acq = true;
            }
            
            if(server_lock_acq && processor_lock_acq){
                break;
            }
            if(server_lock_acq){
                pthread_mutex_unlock(&server_mutex);
                sem_post(&sem_server);
            }
            if(processor_lock_acq){
                pthread_mutex_unlock(&processor_mutex);
            }
        }
        printf("copier_function started\n");fflush(stdout);
            // start = std::chrono::high_resolution_clock::now();
            // printf("%d %d %d\n", has_checkpoint, checkpoint_i, checkpoint_j);fflush(stdout);
        reset();
            // stop = std::chrono::high_resolution_clock::now();
            // reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        memcpy(&og_without_morph[0][0], &skeleton_og_read[0][0], sizeof(char)*og_pixel_depth*og_pixel_width);
        has_checkpoint = has_checkpoint_read;
        checkpoint_j = checkpoint_j_read;
        checkpoint_i = checkpoint_i_read;
        
        printf("copier_function finished1\n");fflush(stdout);
        pthread_mutex_unlock(&server_mutex);
        printf("copier_function finished2\n");fflush(stdout);
        pthread_mutex_unlock(&processor_mutex);
        
        sem_post(&sem_server);
        sem_post(&sem_processor);
        if(terminou){
            break;
        }
    }
    printf("copier_function finished all\n");fflush(stdout);
    return 0;
}

void processor_job(){
    std::chrono::high_resolution_clock::time_point start, stop;

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
        
        // push anything
        reset_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        read_info_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
        print_time_it(1);
}

// executar se copier nao estiver executando e se copier executou
void *processor_function(void * arg){
    while(true){
        sem_wait(&sem_processor);
        pthread_mutex_lock(&processor_mutex);
        
        printf("processor_function started\n");fflush(stdout);
        processor_job();
        iteration_number++;
        
        
        printf("processor_function finished\n");fflush(stdout);
        pthread_mutex_unlock(&processor_mutex);
        printf("processor_function finished1\n");fflush(stdout);
        sem_post(&sem_copier);
        printf("processor_function finished2\n");fflush(stdout);
        if(terminou){
            break;
        }
    }
    printf("processor_function finished all\n");fflush(stdout);
    printf("iteration_number=%d\n", iteration_number);fflush(stdout);
    return 0;
}

    
int main(){
    setup_socket();
    read(new_socket, iterations_limit, sizeof(int));
    printf("iterations_limit: %d\n", iterations_limit[0]);
    
    reset();
    
    pthread_mutex_init(&server_mutex, NULL);
    pthread_mutex_init(&processor_mutex, NULL);
    sem_init(&sem_server, 0, 1);
    sem_init(&sem_copier, 0, 0);
    sem_init(&sem_processor, 0, 0);
    // pthread_mutex_init(&server_processor_executed_mutex, NULL);
    // pthread_cond_init(&server_processor_executed_cond, NULL);
    // pthread_mutex_init(&copier_executed_mutex, NULL);
    // pthread_cond_init(&copier_executed_cond, NULL);
    // pthread_mutex_init(&copier_tried_mutex, NULL);
    // pthread_cond_init(&copier_tried_cond, NULL);
    
    pthread_create(&server_thread, NULL, server_function, NULL);
    pthread_create(&copier_thread, NULL, copier_function, NULL);
    pthread_create(&processor_thread, NULL, processor_function, NULL);
    
    
    pthread_join(server_thread, NULL);
    pthread_join(processor_thread, NULL);
    pthread_join(copier_thread, NULL);
    
    pthread_mutex_destroy(&server_mutex);
    pthread_mutex_destroy(&processor_mutex);
    sem_destroy(&sem_server);
    sem_destroy(&sem_copier);
    sem_destroy(&sem_processor);
    // pthread_mutex_destroy(&server_processor_executed_mutex);
    // pthread_cond_destroy(&server_processor_executed_cond);
    // pthread_mutex_destroy(&copier_executed_mutex);
    // pthread_cond_destroy(&copier_executed_cond);
    // pthread_mutex_destroy(&copier_tried_mutex);
    // pthread_cond_destroy(&copier_tried_cond);
    
    iteration_number--;
    print_time_all();
    
    return 0;
}        

