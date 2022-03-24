#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <sys/socket.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>

#include <opencv2/opencv.hpp>

#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>

#include <jetson-utils/cudaOverlay.h>
#include <jetson-utils/cudaMappedMemory.h>

#include <jetson-inference/segNet.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#ifdef HEADLESS
	#define IS_HEADLESS() "headless"             // run without display
	#define DEFAULT_VISUALIZATION "overlay"      // output overlay only
#else
	#define IS_HEADLESS() (const char*)NULL      // use display (if attached)
    // #define DEFAULT_VISUALIZATION "mask"      // output overlay only
	#define DEFAULT_VISUALIZATION "overlay|mask" // output overlay + mask
#endif

#define ROS_INFO(...)	RCUTILS_LOG_INFO_NAMED(__node_name_.c_str(), __VA_ARGS__)
#define ROS_TIME_NOW()										__global_clock_->now()

namespace sensor_msgs
{
	typedef msg::Image Image;
	typedef msg::Image::SharedPtr ImagePtr;
	typedef msg::Image::ConstSharedPtr ImageConstPtr;
}

std::string __node_name_;
rclcpp::Clock::SharedPtr __global_clock_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub;
rclcpp::Publisher<sensor_msgs::Image>::SharedPtr image_pub;
size_t count_ = 0;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


bool signal_recieved = false;

// occupancy grid
// double cam_height  = 45.5;
double cam_height  = -(104.5 + 1.5 + 24); //why -?
// double cam_angle = 17.5;
double cam_angle = 14.0;
double x_grid_size = 800;
double z_grid_size = 1000; 
double grid_side   = 5;
double img_width   = 800;
double img_height  = 600;
double scale_factor = 2.5;

#define og_width 160
#define og_depth 200


cv::Mat rvec, tvec, mtx, dist;
cv::Mat grid_image, grid_points;
char grid[og_depth][og_width];
cv::Mat mask_image_bgr, mask_image_rgb, original_image_bgr, original_image_rgb;

// double zero_zero_grid_z =   (z_grid_size/4.0);
// double zero_zero_grid_z =   (z_grid_size/5.0) + 45;
// double zero_zero_grid_x = - (x_grid_size/2.0);
double divide_z_grid_size = (z_grid_size/100.0);
double zero_zero_grid_z =   (z_grid_size/divide_z_grid_size) + 45;
double zero_zero_grid_x = - (x_grid_size/2.0);
    

cv::Point3d get_pos_grid(int i, int j){
    cv::Point3d p;
    p.x = zero_zero_grid_x + j*grid_side + grid_side/2.0;
    p.z = zero_zero_grid_z + i*grid_side + grid_side/2.0;
    // p.y = 0;
	p.y = -cam_height;
    return p;
}

cv::Point3d get_pos_grid_scale(int i, int j){
    cv::Point3d p;
    p.x = (zero_zero_grid_x + j*grid_side + grid_side/2.0)/scale_factor;
    p.z = (zero_zero_grid_z + i*grid_side + grid_side/2.0)/scale_factor;
    // p.y = 0;
    p.y = (-cam_height)/scale_factor;
    return p;
}


inline long long int double2int(double value){
    long long int cast = (long long int) value;
    if (value - cast > 0.5)
        cast += 1;
    return cast;
}

inline double degree2rad(double value){
    return value*(M_PI/180.0);
}

inline cv::Point3d rotate_around_x(double angle, cv::Point3d old_point){
    cv::Point3d new_point;
    new_point.x = old_point.x;
    new_point.y = old_point.y*cos(angle) - old_point.z*sin(angle);
    new_point.z = old_point.y*sin(angle) + old_point.z*cos(angle);
    return new_point;
}

void rs2_project_point_to_pixel(double pixel[2], cv::Mat dist, cv::Mat mat, const double point[3])
{
    double x = point[0] / point[2], y = point[1] / point[2];

    double r2 = x * x + y * y;
    double f = 1 + dist.at<double>(0,0) * r2 + dist.at<double>(0,1) * r2 * r2 + dist.at<double>(0,4) * r2 * r2 * r2;
    x *= f;
    y *= f;
    double dx = x + 2 * dist.at<double>(0,2) * x * y + dist.at<double>(0,3) * (r2 + 2 * x * x);
    double dy = y + 2 * dist.at<double>(0,3) * x * y + dist.at<double>(0,2) * (r2 + 2 * y * y);
    x = dx;
    y = dy;
    
    pixel[0] = x * mat.at<double>(0,0) + mat.at<double>(0,2);
    pixel[1] = y * mat.at<double>(1,1) + mat.at<double>(1,2);
}

void rs2_project_point_to_pixel_2(double pixel[2], cv::Mat dist, cv::Mat mat, const double point[3])
{
    double x = point[0] / point[2], y = point[1] / point[2];

    double r2 = x * x + y * y;
    double f = 1 + dist.at<double>(0,0) * r2 + dist.at<double>(0,1) * r2 * r2 + dist.at<double>(0,4) * r2 * r2 * r2;
    double xf = x * f;
    double yf = y * f;
    double dx = xf + 2 * dist.at<double>(0,2) * x * y + dist.at<double>(0,3) * (r2 + 2 * x * x);
    double dy = yf + 2 * dist.at<double>(0,3) * x * y + dist.at<double>(0,2) * (r2 + 2 * y * y);

    x = dx;
    y = dy;
    
    pixel[0] = x * mat.at<double>(0,0) + mat.at<double>(0,2);
    pixel[1] = y * mat.at<double>(1,1) + mat.at<double>(1,2);
}

// grid_img
// 00 01
// 10 11

// grid
// -11    11
// -1-1  -11

// occupancy grid


void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
	}
}

int usage()
{
	printf("usage: segnet [--help] [--network NETWORK] ...\n");
	printf("              input_URI [output_URI]\n\n");
	printf("Segment and classify a video/image stream using a semantic segmentation DNN.\n");
	printf("See below for additional arguments that may not be shown above.\n\n");
	printf("positional arguments:\n");
	printf("    input_URI       resource URI of input stream  (see videoSource below)\n");
	printf("    output_URI      resource URI of output stream (see videoOutput below)\n\n");

	printf("%s\n", segNet::Usage());
	printf("%s\n", videoSource::Usage());
	printf("%s\n", videoOutput::Usage());
	printf("%s\n", Log::Usage());

	return 0;
}


//
// segmentation buffers
//
typedef uchar3 pixelType;		// this can be uchar3, uchar4, float3, float4

pixelType* imgMask      = NULL;	// color of each segmentation class
pixelType* imgOverlay   = NULL;	// input + alpha-blended mask
pixelType* imgComposite = NULL;	// overlay with mask next to it
pixelType* imgOutput    = NULL;	// reference to one of the above three

int2 maskSize;
int2 overlaySize;
int2 compositeSize;
int2 outputSize;

// allocate mask/overlay output buffers
bool allocBuffers( int width, int height, uint32_t flags )
{
	// check if the buffers were already allocated for this size
	if( imgOverlay != NULL && width == overlaySize.x && height == overlaySize.y )
		return true;

	// free previous buffers if they exit
	CUDA_FREE_HOST(imgMask);
	CUDA_FREE_HOST(imgOverlay);
	CUDA_FREE_HOST(imgComposite);

	// allocate overlay image
	overlaySize = make_int2(width, height);
	
	if( flags & segNet::VISUALIZE_OVERLAY )
	{
		if( !cudaAllocMapped(&imgOverlay, overlaySize) )
		{
			LogError("segnet:  failed to allocate CUDA memory for overlay image (%ux%u)\n", width, height);
			return false;
		}

		imgOutput = imgOverlay;
		outputSize = overlaySize;
	}

	// allocate mask image (half the size, unless it's the only output)
	if( flags & segNet::VISUALIZE_MASK )
	{
		maskSize = (flags & segNet::VISUALIZE_OVERLAY) ? make_int2(width/2, height/2) : overlaySize;

		if( !cudaAllocMapped(&imgMask, maskSize) )
		{
			LogError("segnet:  failed to allocate CUDA memory for mask image\n");
			return false;
		}

		imgOutput = imgMask;
		outputSize = maskSize;
	}

	// allocate composite image if both overlay and mask are used
	if( (flags & segNet::VISUALIZE_OVERLAY) && (flags & segNet::VISUALIZE_MASK) )
	{
		compositeSize = make_int2(overlaySize.x + maskSize.x, overlaySize.y);

		if( !cudaAllocMapped(&imgComposite, compositeSize) )
		{
			LogError("segnet:  failed to allocate CUDA memory for composite image\n");
			return false;
		}

		imgOutput = imgComposite;
		outputSize = compositeSize;
	}

	return true;
}

int checkpoint_info[3];
char hello_buffer[1000];
int sock = 0;
struct sockaddr_in serv_addr;
#define PORT 8080

int setup_socket(){
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
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    // iteration limit
    checkpoint_info[0] = 999999;
    send(sock, checkpoint_info, 1*sizeof(int), 0);
    return 0;
}

void send_og(){
    int valread, valsent;
    valread = read(sock , hello_buffer, 1000);
    ROS_INFO("Read: %s\n", hello_buffer);
    valsent = send(sock, grid, og_depth*og_width, 0);
    ROS_INFO("Sent: %d\n", valsent);
    checkpoint_info[0] = 0;
    checkpoint_info[1] = 0;
    checkpoint_info[2] = 0;
    valsent = send(sock, checkpoint_info, 3*sizeof(int), 0);
    ROS_INFO("Sent: %d\n", valsent);
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("talker", "/" "talker");
    __node_name_ = "talker";
    __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    string_pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
    image_pub = node->create_publisher<sensor_msgs::Image>("image_topic", 3);
        
    
    ROS_INFO("video source3");
    
    const int argc2 = 11;
    int argc2c = 0;
    char* argv2[argc2];
    char inp_exec[100] = "oi";
    char inp_model[100] = "--model=/home/nvidia/ros2_new_ws/hrnet_w18.onnx";
    char inp_labels[100] = "--labels=/home/nvidia/ros2_new_ws/classes.txt";
    char inp_colors[100] = "--colors=/home/nvidia/ros2_new_ws/colors.txt";
    char inp_input_blob[100] = "--input_blob=input.1";
    char inp_output_blob[100] = "--output_blob=3545";
    // char inp_input[100] = "/home/nvidia/ros2_new_ws/ufsc.mp4";
    // char inp_input[100] = "/home/nvidia/ros2_new_ws/planetarioParte1.mp4";
	char inp_input[100] = "/dev/video0";
    char inp_input_width[100] = "--input_width=800";
    char inp_input_height[100] = "--input_height=600";
    // char inp_input_framerate[100] = "--nput_framerate=15";
    char inp_output[100] = "/home/nvidia/ros2_new_ws/output.mp4";
    char inp_headless[100] = "--headless";
    
    argv2[argc2c++] = inp_exec;
    argv2[argc2c++] = inp_model;
    argv2[argc2c++] = inp_labels;
    argv2[argc2c++] = inp_colors;
    argv2[argc2c++] = inp_input_blob;
    argv2[argc2c++] = inp_output_blob;
    argv2[argc2c++] = inp_input;
    argv2[argc2c++] = inp_input_width;
    argv2[argc2c++] = inp_input_height;
    // argv2[argc2c++] = inp_input_framerate;
    argv2[argc2c++] = inp_output;
    argv2[argc2c++] = inp_headless;
    for(int i=0;i<argc2;i++){
        ROS_INFO("+%s+\n",argv2[i]);
    }
    
    /*
    * parse command line
    */
    commandLine cmdLine(argc2, argv2, IS_HEADLESS());

    if( cmdLine.GetFlag("help") ){
        return usage();
	}
    
	// socket
    if(setup_socket() == -1){
        return -1;
    }
    // socket
    
	// OG
	rvec = cv::Mat::zeros(1,3,CV_64F);
    tvec = cv::Mat::zeros(1,3,CV_64F);
    mtx  = cv::Mat::zeros(3,3,CV_64F);
    dist = cv::Mat::zeros(1,5,CV_64F);
    
    int grid_dims[] = {(int)(z_grid_size/grid_side), (int)(x_grid_size/grid_side)}; //HxW
    // grid = cv::Mat::zeros(grid_dims[0],grid_dims[1], CV_32S);
    
    int grid_image_dims[] = {(int)(z_grid_size/grid_side), (int)(x_grid_size/grid_side)}; //HxW
    grid_image = cv::Mat::zeros(2, grid_image_dims, CV_8UC3);
    
    std::vector<cv::Point3d> grid_points;

    // tvec.at<double>(0,1) = cam_height;
    mtx.at<double>(0,0)  = 669.56360147;
    mtx.at<double>(0,2)  = 417.80137854;
    mtx.at<double>(1,1)  = 678.49127745;
    mtx.at<double>(1,2)  = 269.62459559;
    mtx.at<double>(2,2)  = 1.0;
    dist.at<double>(0,0)  = 0.10876619;
    dist.at<double>(0,1)  = -0.50646342;
    dist.at<double>(0,2)  = -0.01617755;
    dist.at<double>(0,3)  = 0.00968778;
    dist.at<double>(0,4)  = 0.77699546;
	
	for(int i=0; i < grid_dims[0]; i++){
        for(int j=0; j < grid_dims[1]; j++){
            // cv::Point3d p = get_pos_grid(i,j);
            cv::Point3d p = get_pos_grid_scale(i,j);
            grid_points.push_back(p);
        }
    }
	
	std::vector<cv::Point2d> projectedPoints;
	
	cv::Point3d point3d;
    double point3d_array[3];
    double point2d_array[2];
    double rotate_x_rad = degree2rad(cam_angle);
    for(auto it=grid_points.begin(); it!=grid_points.end(); ++it){
        point3d = *it;
        // std::cout << "point3d" <<std::endl;
        // std::cout << point3d <<std::endl;
        // rotate
        point3d = rotate_around_x(rotate_x_rad, point3d);
        // std::cout << point3d <<std::endl;
        point3d_array[0] = point3d.x;
        point3d_array[1] = point3d.y;
        point3d_array[2] = point3d.z;
        
        // project - which?
        rs2_project_point_to_pixel(point2d_array, dist, mtx, point3d_array);
        projectedPoints.push_back(cv::Point2d(point2d_array[0], point2d_array[1]));
    }
    // cv::projectPoints(grid_points, rvec, tvec, mtx, dist, projectedPoints);
	// OG


    /*
    * attach signal handler
    */
    if( signal(SIGINT, sig_handler) == SIG_ERR )
        LogError("can't catch SIGINT\n");


    /*
    * create input stream
    */
    videoSource* input = videoSource::Create(cmdLine, ARG_POSITION(0));

    if( !input )
    {
        LogError("segnet:  failed to create input stream\n");
        return 0;
    }


    /*
    * create output stream
    */
    videoOutput* output = videoOutput::Create(cmdLine, ARG_POSITION(1));
    
    if( !output )
        LogError("segnet:  failed to create output stream\n");	
    

    /*
    * create segmentation network
    */
    segNet* net = segNet::Create(cmdLine);
    
    if( !net )
    {
        LogError("segnet:  failed to initialize segNet\n");
        return 0;
    }

    // set alpha blending value for classes that don't explicitly already have an alpha	
    net->SetOverlayAlpha(cmdLine.GetFloat("alpha", 150.0f));

    // get the desired overlay/mask filtering mode
    const segNet::FilterMode filterMode = segNet::FilterModeFromStr(cmdLine.GetString("filter-mode", "linear"));

    // get the visualization flags
    const uint32_t visualizationFlags = segNet::VisualizationFlagsFromStr(cmdLine.GetString("visualize", DEFAULT_VISUALIZATION));

    // get the object class to ignore (if any)
    const char* ignoreClass = cmdLine.GetString("ignore-class", "void");
    
    
    while( !signal_recieved )
	{
        auto start = std::chrono::high_resolution_clock::now();
		// capture next image image
		pixelType* imgInput = NULL;

		if( !input->Capture(&imgInput, 1000) )
		{
			// check for EOS
			if( !input->IsStreaming() )
				break; 

			LogError("segnet:  failed to capture video frame\n");
			continue;
		}

		// allocate buffers for this size frame
		if( !allocBuffers(input->GetWidth(), input->GetHeight(), visualizationFlags) )
		{
			LogError("segnet:  failed to allocate buffers\n");
			continue;
		}

		// process the segmentation network
        auto start_process = std::chrono::high_resolution_clock::now();
		if( !net->Process(imgInput, input->GetWidth(), input->GetHeight(), ignoreClass) )
		{
			LogError("segnet:  failed to process segmentation\n");
			continue;
		}
        auto stop_process = std::chrono::high_resolution_clock::now();
        auto duration_process = std::chrono::duration_cast<std::chrono::microseconds>(stop_process - start_process);
        std::cout << "DURATION_PROCESS::" << duration_process.count() << std::endl;
		
		// generate overlay
		if( visualizationFlags & segNet::VISUALIZE_OVERLAY )
		{
			if( !net->Overlay(imgOverlay, overlaySize.x, overlaySize.y, filterMode) )
			{
				LogError("segnet:  failed to process segmentation overlay.\n");
				continue;
			}
		}

		// generate mask
		if( visualizationFlags & segNet::VISUALIZE_MASK )
		{
			if( !net->Mask(imgMask, maskSize.x, maskSize.y, filterMode) )
			{
				LogError("segnet:-console:  failed to process segmentation mask.\n");
				continue;
			}
		}
		
		// wait for the GPU to finish		
		CUDA(cudaDeviceSynchronize()); 
        memset(grid, 0, og_depth*og_width);
		
		ROS_INFO("masksize x e y: %u - %u\n", maskSize.x, maskSize.y);
		ROS_INFO("grid_dims[0] e [1]: %d - %d\n", grid_dims[0], grid_dims[1]);
		for(int i = 0; i < grid_dims[0]; i++){
			for(int j = 0; j < grid_dims[1]; j++){
				long long int pixel_w = double2int((projectedPoints[i*grid_dims[1] + j].x)/2.0);
				long long int pixel_h = double2int((projectedPoints[i*grid_dims[1] + j].y)/2.0);
				
				if (pixel_w >= maskSize.x || pixel_w < 0 || pixel_h >= maskSize.y || pixel_h < 0)
					continue;
				else{
					// grid.at<int>((grid_dims[0]-1) - i, j) = 1;
					char * uc = reinterpret_cast<char *>(&imgMask[maskSize.x*pixel_h + pixel_w]);
					uint r,g,b;
					r = *uc;
					g = *(++uc);
					b = *(++uc);
                    if (r==0 && g==0 && b==0){
                        grid[grid_dims[0]-1 - i][j] = 0;
                    } else{
                        grid[grid_dims[0]-1 - i][j] = 1;
                    }
					// std::cout << "i:" << i << " j:" << j << "--- rgb: " << r << " " << g << " " << b << std::endl;
					// std::cout << "pixel_w:" << pixel_w << " pixel_h:" << pixel_h << std::endl;  
					grid_image.at<cv::Vec3b>((grid_dims[0]-1) - i, j) = cv::Vec3b(b, g, r);
				}
			}
		}
		
        send_og();
        
		// std::cout << grid << std::endl;
		
		char buffer_img_name [1000];
		sprintf(buffer_img_name, "/home/nvidia/ros2_new_ws/og_images/occupancy_grid_output%03lu.png", count_);
		std::string img_name =  std::string(buffer_img_name);
		std::cout << img_name << std::endl;
		cv::imwrite(img_name, grid_image);
		
		// // populate the message
        // sensor_msgs::Image msg;

        // // populate timestamp in header field
        // msg.header.stamp = ROS_TIME_NOW();
        
        // // calculate size of the msg
        // const size_t msg_size = imageFormatSize(IMAGE_RGB8, maskSize.x, maskSize.y);
        // ROS_INFO("6MSG_SIZE: %lu\n", msg_size);

        // // allocate msg storage
        // msg.data.resize(msg_size);
		
		// // copy the converted image into the msg
        // memcpy(msg.data.data(), imgMask, msg_size);
		
        // int counter = 0;
        // int counter2 = 0;
        // for(auto it=msg.data.begin();it!=msg.data.end();++it){
        //     counter++;
        //     if(*it!=0){
        //         counter2++;
        //     }
        // }
        // std::cout<< "COUNTERS:" << std::endl;
        // std::cout<< counter << std::endl;
        // std::cout<< counter2 << std::endl;

        // // publish the message
        // image_pub->publish(msg);
        // ROS_INFO("published %ux%u video frame", maskSize.x, maskSize.y);
		
		// save mask and image
		mask_image_rgb = cv::Mat(maskSize.y, maskSize.x, CV_8UC3, imgMask);
		mask_image_bgr = cv::Mat(maskSize.y, maskSize.x, CV_8UC3);
		cv::cvtColor(mask_image_rgb, mask_image_bgr, cv::COLOR_RGB2BGR);
		sprintf(buffer_img_name, "/home/nvidia/ros2_new_ws/masks/masks_output%03lu.png", count_);
		img_name =  std::string(buffer_img_name);
		std::cout << img_name << std::endl;
		cv::imwrite(img_name, mask_image_bgr);
		
		original_image_rgb = cv::Mat(img_height, img_width, CV_8UC3, imgInput);
		original_image_bgr = cv::Mat(img_height, img_width, CV_8UC3);
		cv::cvtColor(original_image_rgb, original_image_bgr, cv::COLOR_RGB2BGR);
		sprintf(buffer_img_name, "/home/nvidia/ros2_new_ws/originals/originals_output%03lu.png", count_);
		img_name =  std::string(buffer_img_name);
		std::cout << img_name << std::endl;
		cv::imwrite(img_name, original_image_bgr);
		
		
		

		// generate composite
		if( (visualizationFlags & segNet::VISUALIZE_OVERLAY) && (visualizationFlags & segNet::VISUALIZE_MASK) )
		{
			CUDA(cudaOverlay(imgOverlay, overlaySize, imgComposite, compositeSize, 0, 0));
			CUDA(cudaOverlay(imgMask, maskSize, imgComposite, compositeSize, overlaySize.x, 0));
		}

		// render outputs
		if( output != NULL )
		{
			output->Render(imgOutput, outputSize.x, outputSize.y);

			// update the status bar
			char str[256];
			sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, net->GetNetworkName(), net->GetNetworkFPS());
			output->SetStatus(str);

			// check if the user quit
			if( !output->IsStreaming() )
				signal_recieved = true;
		}

		// wait for the GPU to finish		
		CUDA(cudaDeviceSynchronize());        
        
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        ROS_INFO("Publishing: '%s'", message.data.c_str());
        string_pub->publish(message);
        
        

        // print out timing info
        net->PrintProfilerTimes();
        
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  
        std::cout << "DURATION::" << duration.count() << std::endl;
        std::cout << "OUTPUTSIZE::" << net->GetOutputSize() << std::endl;
        std::cout << "OUTPUTW::" << net->GetOutputWidth() << std::endl;
        std::cout << "OUTPUTH::" << net->GetOutputHeight() << std::endl;
	}
	

	/*
	 * destroy resources
	 */
	LogVerbose("segnet:  shutting down...\n");
	
	SAFE_DELETE(input);
	SAFE_DELETE(output);
	SAFE_DELETE(net);

	CUDA_FREE_HOST(imgMask);
	CUDA_FREE_HOST(imgOverlay);
	CUDA_FREE_HOST(imgComposite);

	LogVerbose("segnet:  shutdown complete.\n");
    
    rclcpp::shutdown();
    
    return 0;
}
