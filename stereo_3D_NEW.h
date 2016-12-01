//AUTHOR HAOBO (DAVID) YU
//THIS CLASS HAS FUNCTIONS TO FIND DEPTH OF DIFFERENT OBJECTS
//SOLVES THE POSE PROBLEM GIVEN A SET OF WORLD POINTS AND A TEMPLATE LENGTH VECTOR


#ifndef STEREO_3D_H
#define STEREO_3D_H
#include <opencv2\core.hpp>
#include <opencv2\calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <map>
#include "ovrvision_setting.h"
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>

////cuda
//#include <opencv2/cudastereo.hpp>
//#include <opencv2/cudafeatures2d.hpp>
//#include "opencv2/xfeatures2d/cuda.hpp"
//#include <opencv2/cudaimgproc.hpp>
using namespace cv::cuda;

class get_stereo_depth
{
public:
	get_stereo_depth();
	~get_stereo_depth();

	//Left or right
	enum direction { LEFT, RIGHT };

	//This structure is to store information about the branch connecting different nodes
	struct path_3D
	{
		//int d_i;//id of the distance;
		int id_begin;//id of the point that begins the loops
		int id_end; //id of the point that ends the loops
		double distance;
		//double error;
		std::vector<double> errors;
	};

	//gets the camera dimensions
	void get_cam_dim(int h, int w){ cam_H = h, cam_W = w; }
	//gets focal length, but it is not used yet
	void get_focal_length(double f){ focal_length = f; }
	void get_norm_gap(double g){ norm_gap = g; }
	void get_focal_point_scale(double s){ focal_point_scale = s; }

	//Gets the rotation, and translation matrix/vector for use in the triangulation of points.
	//And at the same time the intrinsic properties of the two cameras will used as the inputs.
	void get_projection_parameters(cv::Mat F_in,cv::Mat R_in, cv::Mat t_in, cv::Mat A_L_in, cv::Mat A_R_in, cv::Mat P1_in, cv::Mat P2_in, cv::Mat R1_in, cv::Mat R2_in, cv::Mat D1_in, cv::Mat D2_in){
		R = R_in, t = t_in, A_L = A_L_in, A_R = A_R_in, P1 = P1_in, P2 = P2_in;
		R1 = R1_in, R2 = R2_in, D1 = D1_in, D2 = D2_in; F = F_in;
	}
	//get points for right and left the inputs needs to be of the type vector<cv::keypoints>
	void get_points_keypoints(std::vector<cv::KeyPoint> left, std::vector<cv::KeyPoint> right);

	//get points for right and left the inputs needs to be of the type vector<cv::point2f>
	void get_points_point2f(std::vector<cv::Point2f> left, std::vector<cv::Point2f> right){
		left_points_vector = left;
		right_points_vector = right;
	}


	//This function will return true if we have the same number of detected points in the left images as the right.
	//and it must be the case that the size if greater than 0;
	bool same_points();

	
	//This function will find the correspondance between the left and right. It will compare difference between each of the points and finds the closest.
	//This function is only called if same_points is true;
	int find_correspondance(cv::Mat F);

	//Responsible for finding the 3D position given that we already have correspondance
	int find_world_points_triangulation(void);

	//After find_world_points_triangulation, the 3D points can then be outputed as a point3f vector
	std::vector<cv::Point3f> print_world_point(void);

	

	//if the distance_template is different from the class initialization
	void get_distance_template(std::vector<double> distance_template_in){distance_template = distance_template_in;}

	//This function will find the correspondance between a rigid virtual object with n points.
	//If there are n points it will then the first input vector will have all of the distances
	//between points, i.e. d1=abs(p1-p2), ..., dn = abs(p(n-1)-pn). 
	//input: real_3d_points: distance_3D (stereo or kinectv2)
	//		 distance_template: The distances between the real probe, the measurements will be d_{0-1},d_{1-2},...,d_{n_1}. 
	//Output: will be a std::vector<cv::Point3f> with the point ids being inorder.
	std::vector<cv::Point3f> rigid_pose_estimation(int num_pose_points, std::vector<cv::Point3f> real_3d_points);

	//This function will return the distance between two points and the distance in the zaxis between the points and the camera
#ifdef 0
	void return_points(double *d_two, double *x_d, double *y_d, double *z_d){
		*d_two = distance_two_points;
		*x_d = x_distance;
		*y_d = y_distance;
		*z_d = z_distance;
	}
#endif
	//This function will check if the path described by the vector of path_3D structures form a path
	bool is_path_closed(std::vector<path_3D> check_vector_input);
	

	//this function will return the ordered images points, for left and right
	void return_ordered_imagepoints(std::vector<cv::Point2f> *left_p, std::vector<cv::Point2f>  *right_p);


	//This function will clear some of the variabes;

	void clear_points();

	//return ordered world point \in 3R
	std::vector<cv::Point3f>  return_ordered_world_point(void){		return ordered_world_points;	}
	std::vector<cv::Point3f> return_probe_vector(void){		return probe_position;	}
	pcl::PointCloud<pcl::PointXYZ> return_green_points(void){		return *temp_xyz;	}
	
	//return distorted ordered points
	std::vector<cv::Point2f>  return_distorted_ordered_points(direction d_in);


	void get_image(cv::Mat *im_left_in, cv::Mat *im_right_in);

	//Find the green points in both RGBA images using HSV conversion
	void detect_key_points();

	void stereo_SVD(void);

	//Returns the transformation matrix between the markers and the camera
	Eigen::Matrix4f get_transformation_matrix(void){return transformation_mat;}

	//Remaping the images
	void remap_image(OVR::OvrvisionSetting *ovr_setting_ptr,cv::Mat *im_left_output,cv::Mat *im_right_output);
	
	//Finds the image mapper for the left and right images, then the function remap_image(..) can be called
	void get_image_mappers(OVR::OvrvisionSetting *ovr_setting_ptr, double x_offset, double y_offset){
		ovr_setting_ptr->GetUndistortionMatrix(OVR::Cameye::OV_CAMEYE_LEFT, map1x[0], map1y[0], cam_W, cam_H, x_offset, y_offset);
		ovr_setting_ptr->GetUndistortionMatrix(OVR::Cameye::OV_CAMEYE_RIGHT, map1x[1], map1y[1], cam_W, cam_H, x_offset, y_offset);
	}

	std::vector<cv::Point3f> ordered_world_points;
	std::vector<cv::Point3f> probe_position;
	
private:
	//These variabes are used for 3D stereo mapping
	int cam_W, cam_H;
	double focal_length;
	double norm_gap;
	double focal_point_scale;
	std::vector<cv::KeyPoint> left_points;
	std::vector<cv::KeyPoint> right_points;
	std::vector<cv::Point2f> left_points_vector;
	std::vector<cv::Point2f> right_points_vector;
	std::vector<cv::Point2f> left_points_ordered, right_points_ordered;
	std::vector<cv::Point2f> left_points_ordered_distorted, right_points_ordered_distorted;
	std::vector<cv::Point3f> world_points;
	
	cv::Mat R, t;
	cv::Mat A_L, A_R;
	cv::Mat P1, P2;
	//Rotation for cam1 and 2, and distortion vectors for cam 1 and 2
	cv::Mat R1, R2, D1, D2;
	cv::Mat F;

	//This for testing the accuracy of thee stereocamera, 
	double distance_two_points; //Distance between two points 
	//average distances for the two green balls.
	double y_distance;
	double x_distance;
	double z_distance;

	//Num of points on tracker
	int num_tracker_points;
	//The tempalte geometry
	pcl::PointCloud<pcl::PointXYZ>* green_points;//(new pcl::PointCloud<pcl::PointXYZ>);
	//The template distance 

	std::vector<double> distance_template;



	// This vector will have the correct orientation with respect to our template geometry
	std::vector<path_3D> final_answer;


	//image pointers and matrices
	unsigned char* p;
	unsigned char* p2;
	cv::Mat im_left = cv::Mat(cam_H, cam_W, CV_8UC4);
	cv::Mat im_right = cv::Mat(cam_H, cam_W, CV_8UC4);
	cv::Mat color_HSV = cv::Mat(cam_H, cam_W, CV_8UC3);
	cv::Mat gray_image = cv::Mat(cam_H, cam_W, CV_8UC1);
	/*cv::Mat im_left ;
	cv::Mat im_right ;
	cv::Mat color_HSV ;
	cv::Mat gray_image;*/
	//Blob detector params
	cv::SimpleBlobDetector::Params params;

	

	cv::Ptr<cv::SimpleBlobDetector> detector_blob;

	//ICP POINTER
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD_estimate;

	//SVD parameters
	Eigen::Matrix4f transformation_mat;

	//image mappers
	cv::Mat map1x[2];
	cv::Mat map1y[2];
	std::vector<cv::Point2f> project_2D_left, project_2D_right;
	pcl::PointCloud<pcl::PointXYZ> * temp_xyz ;

	//old image pointers for when the probe cannot be tracked
	std::vector<cv::Point2f> temp_left_old, temp_right_old;

	//For the chessboard found positions
	std::vector<cv::Point2f> current_corners_left;
	std::vector<cv::Point2f> current_corners_right;

	bool print_chess_board = false; //print or not print chessabord

//	GpuMat img1;
	//GpuMat img2;

	
	//Filters
	//cv::Ptr<cv::cuda::HoughCirclesDetector>  hough_circle; 
	//gpu vector for hough circle
	//GpuMat d_circle_left,d_circle_right;
	

	//cv::Ptr<cv::cuda::Filter> cuda_gauss_filter;// 
};



#endif // !STEREO_3D_H
