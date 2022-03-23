//  g++ occupancy_grid_correct_nano_ogs_test.cpp -o occupancy_grid `pkg-config --cflags --libs opencv4`
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <set>
#include <utility>

// file
int exp_number = 18;
int img_number = 83;

// occupancy grid
// double cam_height  = -45.5;
double cam_height  = -(104.5 + 1.5 + 24); //why -?
double cam_angle = 5;
double x_grid_size = 800;
// double x_grid_size = 200;
double z_grid_size = 1000;
// double z_grid_size = 400; 
double grid_side   = 5;
double img_width   = 800;
double img_height  = 600;
double scale_factor = 2.5;
// double img_width   = 400;
// double img_height  = 300;

cv::Mat rvec, tvec, mtx, dist;
cv::Mat grid, grid_image;


double divide_z_grid_size = (z_grid_size/100.0);
double zero_zero_grid_z =   (z_grid_size/divide_z_grid_size) + 45;
// double zero_zero_grid_z =   (z_grid_size/5.0);
// double zero_zero_grid_z =   (200);
double zero_zero_grid_x = - (x_grid_size/2.0);
// double zero_zero_grid_z =   (-z_grid_size/2.0);
// double zero_zero_grid_x = - (x_grid_size/2.0);

// visualize grid ni img
cv::Mat img_with_grid;
std::set<std::pair<long long int, long long int>> og_pixels; //visualize pixels in img

    

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

// librealsense
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

// librealsense
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


int main(){
    
    rvec = cv::Mat::zeros(1,3,CV_64F);
    tvec = cv::Mat::zeros(1,3,CV_64F);
    mtx  = cv::Mat::zeros(3,3,CV_64F);
    dist = cv::Mat::zeros(1,5,CV_64F);
    
    int grid_dims[] = {(int)(z_grid_size/grid_side), (int)(x_grid_size/grid_side)}; //HxW
    grid = cv::Mat::zeros(grid_dims[0],grid_dims[1], CV_32S);
    
    int grid_image_dims[] = {(int)(z_grid_size/grid_side), (int)(x_grid_size/grid_side)}; //HxW
    grid_image = cv::Mat::zeros(2, grid_image_dims, CV_8UC3);
    
    std::vector<cv::Point3d> grid_points;

    // tvec.at<double>(0,1) = cam_height;
    // tvec.at<double>(0,2) = cam_height;
    // rvec.at<double>(0,0) = -degree2rad(5.5);
    // rvec.at<double>(0,0) = +degree2rad(45.0);
    // rvec.at<double>(0,1) = -degree2rad(45.0);
    // rvec.at<double>(0,2) = -M_PI/11.0;
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
    char imread_buffer[1000];
    sprintf(imread_buffer, "../correct_nano_ogs/imgs/masks_output%03d.png", img_number);
    cv::Mat image = cv::imread(imread_buffer);
    // cv::Mat image = cv::imread("../correct_nano_ogs/imgs/originals_output010.png");

    cv::imshow("image", image);
  
    // Wait for any keystroke
    // cv::waitKey(0);
    
    // std::cout << image;
    cv::Size s = image.size();
    int rows = s.height;
    int cols = s.width;
    std::cout << s << "\n";
    std::cout << rows << "\n";
    std::cout << cols << "\n";
    
    int out_first = 0;
    int out_last_i = -1;
    bool out_can_be_last_i = true;
    int out_total = 0;
    
    for(int i = 0; i < grid_dims[0]; i++){
        out_can_be_last_i = true;
        for(int j = 0; j < grid_dims[1]; j++){
            long long int pixel_w = double2int((projectedPoints[i*grid_dims[1] + j].x)/2.0);
            long long int pixel_h = double2int((projectedPoints[i*grid_dims[1] + j].y)/2.0);
            // long long int pixel_w = double2int((projectedPoints[i*grid_dims[1] + j].x));
            // long long int pixel_h = double2int((projectedPoints[i*grid_dims[1] + j].y));
            std::cout << "pixel_w: " << pixel_w << "; pixel_h: " << pixel_h << "\n";
            
            if (pixel_w >= cols || pixel_w < 0 || pixel_h >= rows || pixel_h < 0){
                if(i==0){
                    out_first++;
                }
                out_can_be_last_i = false;
                out_total++;
                continue;
            }
            else{
                grid.at<int>((grid_dims[0]-1) - i, j) = 1;
                grid_image.at<cv::Vec3b>((grid_dims[0]-1) - i, j) = image.at<cv::Vec3b>(pixel_h, pixel_w);
                og_pixels.insert(std::pair<long long int, long long int>(pixel_h, pixel_w)); //visualize pixels in img
            }
        }
        if(out_can_be_last_i && out_last_i==-1){
            out_last_i = i;
        }
    }
    std::cout << "out_total:" << out_total << "\n";
    std::cout << "out_last_i:" << out_last_i << "\n";
    std::cout << "out_first:" << out_first << "\n";
    
    // show distance - put lines in grid_image
    // for(int i=0; i < z_grid_size; i++){
    //     if(i%100 >=0 && i%100 <=2){
    //         for(int j=0;j <grid_dims[1]; j++){
    //             grid_image.at<cv::Vec3b>((grid_dims[0]-1) - (i/grid_side), j) = cv::Vec3b(0, 0, 255);
    //         }
    //     }
    // }
    
    
    
    char imwrite_buffer [1000];
    sprintf(imwrite_buffer, "../correct_nano_ogs/new_ogs/og_output_%02d_%03d.png", \
        exp_number, img_number);
    cv::imshow("grid", grid_image);
    cv::imwrite(imwrite_buffer, grid_image);
    
    long long int max_height = 0;
    for(auto it = og_pixels.begin(); it!=og_pixels.end(); ++it){
        std::pair<long long int, long long int> p = *it;
        if(max_height < p.first)
            max_height = p.first;
        image.at<cv::Vec3b>(p.first, p.second) = cv::Vec3b(0, 0, 255);
    }
    std::cout << "max_height: " << max_height << std::endl;
    
    cv::imshow("grid in image", image);
  
    // Wait for any keystroke
    cv::waitKey(0);
    return 0;
}

