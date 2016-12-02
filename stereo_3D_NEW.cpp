
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



get_stereo_depth::get_stereo_depth()
{
	//default num tracker is 4, this can be more, but will require more computation
	//and also the map needs to be changed in rigid_pose_estimation
	num_tracker_points = 4;
	
	//Distance template 
	distance_template.push_back(59.67 / 1000.0);
	distance_template.push_back(60.45 / 1000.0);
	distance_template.push_back(39.93 / 1000.0);
	distance_template.push_back(134.66 / 1000.0);
	//default geometry
	green_points=(new pcl::PointCloud<pcl::PointXYZ>);
	temp_xyz = (new pcl::PointCloud<pcl::PointXYZ>);
	
	green_points->push_back(pcl::PointXYZ(128.78/1000.0, 40.16/1000.0, 0.0));
	green_points->push_back(pcl::PointXYZ(99.43/1000.0, -11.87/1000.0, -0.0));
	green_points->push_back(pcl::PointXYZ(39.77/1000.0, 0.0, -0.0));
	green_points->push_back(pcl::PointXYZ(0.0, 0.0, -0.0));
	
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 45;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.6;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.6;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.4;

	detector_blob  = cv::SimpleBlobDetector::create(params);

}

get_stereo_depth::~get_stereo_depth()
{
}


void get_stereo_depth::get_points_keypoints(std::vector<cv::KeyPoint> left, std::vector<cv::KeyPoint> right){
		left_points = left; right_points = right;
		
		cv::KeyPoint::convert(left_points, left_points_vector);
		cv::KeyPoint::convert(right_points, right_points_vector);
		for (auto cc = left_points_vector.begin(); cc != left_points_vector.end(); ++cc){
			cc->x = cc->x * 2.0;
			cc->y = cc->y*2.0;
		}
		for (auto cc = right_points_vector.begin(); cc != right_points_vector.end(); ++cc){
			cc->x = cc->x *2.0;
			cc->y = cc->y*2.0;
		}
}


bool get_stereo_depth::same_points(){
	if (!(left_points.size() == right_points.size())) return false;
	if (!(left_points.size() > 0)) return false;
	return true;
}

int get_stereo_depth::find_correspondance(cv::Mat F){
		if (!same_points())
			return 0;
	
		left_points_ordered_distorted = left_points_vector;
		right_points_ordered_distorted = right_points_vector;
		cv::correctMatches(F, left_points_vector, right_points_vector, left_points_ordered_distorted, right_points_ordered_distorted);

		//CORRECTMATCHeS AND UNDISTORT WAS SWITCHED BEFORE

		if (!left_points_vector.empty()){
			//cv::Mat camPs = getOptimalNewCameraMatrix(A_L, D1, cv::Size(cam_W, cam_H), 0, cv::Size(cam_W, cam_H));
			cv::undistortPoints(left_points_ordered_distorted, left_points_ordered, A_L, D1, R1, P1);

			//cv::undistortPoints(left_points_vector, left_points_vector, A_L, D1, R1, P1);
			//setting_ptr->undistort_points(OVR::OV_CAMEYE_LEFT, cam_W, cam_H, &left_points_vector);
		}
		if (!right_points_vector.empty()){
			//cv::Mat camPs = getOptimalNewCameraMatrix(A_R, D2, cv::Size(cam_W, cam_H), 0, cv::Size(cam_W, cam_H));
			cv::undistortPoints(right_points_ordered_distorted, right_points_ordered, A_R, D2, R2, P2);
			//setting_ptr->undistort_points(OVR::OV_CAMEYE_RIGHT, cam_W, cam_H, &right_points_vector);
		}





}


int get_stereo_depth::find_world_points_triangulation(void){

		if (!same_points())
			return 0;
		/*cv::Point3f depth_insert;
		cv::Mat ee = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat s;
		cv::Mat P_matrix2;
		cv::Mat zeros_v = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
		cv::hconcat(ee, zeros_v, P_matrix1);
		P_matrix1 = A_L*P_matrix1;
		cv::hconcat(R, t, P_matrix2);
		P_matrix2 = A_R*P_matrix2;
		std::cout << " TRANSLATION TTTTT : " << t << std::endl;
		std::cout << "P1 matrix : " << P_matrix1 << std::endl;
		std::cout << "P2 matrix : " << P_matrix2 << std::endl;
		*/
		/*std::cout << pnts3D << std::endl;
		std::cout << std::endl;*/
		std::vector<cv::Point3f> solution_vector;


		//if (!left_points_ordered.empty())
		//	cv::undistortPoints(left_points_ordered, left_points_ordered, A_L, D1, R1, P1);
		//if (!right_points_ordered.empty())
		//	cv::undistortPoints(right_points_ordered, right_points_ordered, A_R, D2, R2, P2);



		for (int ll = 0; ll < left_points_ordered.size(); ll++){
			std::vector<cv::Point2f> dummyvec_left, dummyvec_right;
			//left_points_ordered[ll].y = cam_H - left_points_ordered[ll].y;
			//right_points_ordered[ll].y = cam_H - right_points_ordered[ll].y;
			dummyvec_left.push_back(left_points_ordered[ll]);
			dummyvec_right.push_back(right_points_ordered[ll]);
			cv::Mat pnts3D;
			
			cv::triangulatePoints(P1, P2, dummyvec_left, dummyvec_right, pnts3D);


			//std::cout << pnts3D.col(ll) << std::endl;
			//std::cout << "PNTS3D : " << std::endl;

			cv::Vec4f dummy1 = pnts3D.col(0);
			dummy1[0] = dummy1[0] / dummy1[3];

			dummy1[1] = -dummy1[1] / dummy1[3];
			dummy1[2] =1000.0- (dummy1[2] / dummy1[3] + 15.0 / 2.0);
			dummy1[3] = dummy1[3] / dummy1[3];

			std::cout << dummy1 << std::endl;

			solution_vector.push_back(cv::Point3f(dummy1[0], dummy1[1], dummy1[2]));
			std::cout << std::endl;

		}
		//std::cout << "FINAL SOLUTION TO VECTORS : " << solution_vector << std::endl;

	/*	if (solution_vector.size() == 2){
			cv::Point3f i1 = solution_vector[0];
			cv::Point3f i2 = solution_vector[1];
			double __norm__ = cv::norm(i1 - i2);
			std::cout << "Distance between two points " << __norm__ << std::endl;
			distance_two_points = __norm__;

			x_distance = (solution_vector[0].x + solution_vector[1].x) / 2.0;
			y_distance = (solution_vector[0].y + solution_vector[1].y) / 2.0;
			z_distance = (solution_vector[0].z + solution_vector[1].z) / 2.0;
		}*/
		world_points = solution_vector;
		//cv::Mat dummy1 = pnts3D.col(0); /////////////////
		//dummy1.row(0) = dummy1.row(0) / dummy1.row(3);
		//dummy1.row(1) = dummy1.row(1) / dummy1.row(3);
		//dummy1.row(2) = dummy1.row(2) / dummy1.row(3);
		//dummy1.row(3) = dummy1.row(3) / dummy1.row(3);

		//if (pnts3D.cols > 1){
		//	cv::Mat dummy2 = pnts3D.col(1); /////////////////
		//	dummy2.row(0) = dummy2.row(0) / dummy2.row(3);
		//	dummy2.row(1) = dummy2.row(1) / dummy2.row(3);
		//	dummy2.row(2) = dummy2.row(2) / dummy2.row(3);
		//	dummy2.row(3) = dummy2.row(3) / dummy2.row(3);
		//	double __norm__ = cv::norm(dummy1 - dummy2);

		//	std::cout << "Distance between two points " << __norm__ << std::endl;



		//}





		/*for (int i = 0; i < pnts3D.cols; i++){
			std::cout << i << "th " << std::endl;
			for (int j = 0; j < triang_out.rows; j++){
			std::cout << triang_out.at<double>(0, 0);

			}

			std::cout << std::endl;
			}*/



		/*cv::MatIterator_<double> _it = triang_out.begin<double>();
		for (; _it != triang_out.end<double>(); _it++){
		std::cout << *_it << std::endl;
		}*/
		/*for(auto cc = triang_out.begin();cc!= triang_out.end();++cc){
			cc->row(0)=cc->row(1).div(cc->row(3));
			std::cout <<(*cc)<<std::endl;

			}*/

#if 0
		for (int i = 0; i < left_points_ordered.size(); i++){
			//cv::Mat xl = (cv::Mat_<double>(3, 1) << left_points_ordered.at(i).x*A_L.at<double>(0, 0), left_points_ordered.at(i).y*A_L.at<double>(1, 1), 1.0);
			//cv::Mat xr = (cv::Mat_<double>(3, 1) << right_points_ordered.at(i).x*A_R.at<double>(0, 0), right_points_ordered.at(i).y*A_R.at<double>(1, 1), 1.0);
			cv::Mat P1 = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
			cv::Mat zeros_v = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
			/*	cv::Mat xl = (cv::Mat_<double>(3, 1) << left_points_ordered.at(i).x, left_points_ordered.at(i).y, 1.0);
				cv::Mat xr = (cv::Mat_<double>(3, 1) << right_points_ordered.at(i).x, right_points_ordered.at(i).y, 1.0);*/
			//cv::Mat xl = (cv::Mat_<double>(3, 1) << left_points_ordered.at(i).x - A_L.at<double>(0, 2), left_points_ordered.at(i).y - A_L.at<double>(1, 2), 1.0);
			//cv::Mat xr = (cv::Mat_<double>(3, 1) << right_points_ordered.at(i).x - A_R.at<double>(0, 2), right_points_ordered.at(i).y - A_R.at<double>(1, 2), 1.0);
			//cv::Mat P2 = (cv::Mat_<double>(3, 1) << 0.0, 0.0,0.0);
			cv::Mat xl = (cv::Mat_<double>(3, 1) << left_points_ordered.at(i).x, left_points_ordered.at(i).y, 1.0);
			cv::Mat xr = (cv::Mat_<double>(3, 1) << right_points_ordered.at(i).x, right_points_ordered.at(i).y, 1.0);
			//t.at<double>(0) = 0.005951;
			cv::Mat ee = cv::Mat::eye(3, 3, CV_64F);
			cv::Mat P_matrix1;
			cv::Mat P_matrix2;
			cv::hconcat(ee, zeros_v, P_matrix1);
			P_matrix1 = A_L*P_matrix1;
			cv::hconcat(R, t, P_matrix2);
			P_matrix2 = A_R*P_matrix2;
			cv::Mat triang_out;

			cv::triangulatePoints(P_matrix1, P_matrix2, left_points_ordered, right_points_ordered, triang_out);
			std::cout << triang_out << std::endl;
			cv::Mat dummy;
			cv::Mat P2 = -R.inv() * t;
			cv::Mat d1 = ee* A_L.inv()*xl;
			cv::Mat d2 = R.inv()*A_R.inv()*xr;

			d1 = d1 / cv::norm(d1);
			d2 = d2 / cv::norm(d2);
			std::cout << "xl : " << xl << std::endl;
			std::cout << "xr : " << xr << std::endl;
			std::cout << "D1 : " << d1 << std::endl;
			std::cout << "D2 : " << d2 << std::endl;
			std::cout << "R : " << R << std::endl;
			std::cout << "P2: " << P2 << std::endl;
			std::cout << "INTRINSIC LEFT: " << A_L << std::endl;
			std::cout << "INTRINSIC RIGHT: " << A_R << std::endl;


			/*dummy = xl;
			dummy = dummy*xr;*/
			cv::Mat p1mp2 = P1 - P2;

			cv::Mat k;

			cv::Mat ss = d1*(d2.dot(d2)) - d2*(d2.dot(d1));
			double divisor = d2.dot(d1) - d1.dot(d1);
			cv::Mat tt = (p1mp2.t()*ss) / divisor;
			/*std::cout << "P1 : " << P1 << std::endl;
			std::cout << "P2 : " <<P2 << std::endl;
			std::cout << "d1 : " << d1 << std::endl;
			std::cout << "d2 : " << d2 << std::endl;
			std::cout << "ss : " << ss << std::endl;
			std::cout << " p1mp2 : " << t << std::endl;
			*/
			cv::Mat Q1 = P1 + tt.at<double>(0)*d1;

			cv::Mat s = p1mp2.dot(d1) + tt*d1.dot(d1);
			s = s / (d2.dot(d1));
			cv::Mat Q2 = P2 + s.at<double>(0)*d2;
			cv::Mat Q1mQ2 = Q1 - Q2;
			double Q_DOT = Q1mQ2.dot(d2);
			std::cout << " Q_DOT : " << Q_DOT << std::endl;

			cv::Mat Q_ave = (Q1 + Q2) / 2.0;
			std::cout << " Q1 : " << Q1 << std::endl;
			std::cout << " Q2 : " << Q2 << std::endl;
			std::cout << std::endl;
			std::cout << std::endl;
			std::cout << std::endl;
			std::cout << "Q_AVE : " << Q_ave << std::endl;
		}

#endif // 0

		/*for (int i = 0; i < left_points_ordered.size(); i++){
			left_points_ordered.at(i).x = left_points_ordered.at(i).x - cam_W / 2.0;
			right_points_ordered.at(i).x = right_points_ordered.at(i).x - cam_W / 2.0;
			left_points_ordered.at(i).y = left_points_ordered.at(i).y - cam_H / 2.0;
			depth_insert.y = focal_length*focal_point_scale*norm_gap / (left_points_ordered.at(i).x - right_points_ordered.at(i).x);
			depth_insert.x = left_points_ordered.at(i).x *norm_gap / (left_points_ordered.at(i).x - right_points_ordered.at(i).x);
			depth_insert.z = left_points_ordered.at(i).y*norm_gap / (left_points_ordered.at(i).x - right_points_ordered.at(i).x);
			world_points.push_back(depth_insert);

			}*/


}

std::vector<cv::Point3f> get_stereo_depth::print_world_point(void){
		std::cout << "WORLD POINTS" << std::endl;
		std::cout << world_points << std::endl;
		return world_points;
}



std::vector<cv::Point3f> get_stereo_depth::rigid_pose_estimation(int num_pose_points, std::vector<cv::Point3f> real_3d_points){
		/*path_3D *path_3D_ptr;
		path_3D_ptr = new path_3D[num_pose_points];
		*/
		//This map is for physical paths in 3D.
		//The size will be determined by the number of points that we have on our markers.
		//This 
		std::map<int, path_3D> point_mapper_stereo;
		//std::map<int, path_3D> store_mapper;

		//This following map will store all potential 3D paths into a container.
		std::map<int, std::vector<path_3D>> store_mapper;
		int counter = 0;
		double epsilon = 0.016;
		for (int i = 0; i < num_pose_points; i++){
			for (int j = i + 1; j < num_pose_points; j++){
				path_3D temp;
				temp.id_begin = i;
				temp.id_end = j;
				temp.distance = cv::norm(real_3d_points[i] - real_3d_points[j]);
				point_mapper_stereo[counter] = temp;
				counter++;
			}
		}
		int numpath = num_pose_points*(num_pose_points - 1) / 2;
		//Looping through all of the possible points
		for (std::map<int, path_3D>::iterator point_it = point_mapper_stereo.begin(); point_it != point_mapper_stereo.end(); ++point_it){
			for (int j = 0; j < num_pose_points; j++){
				double error_temp = cv::abs(point_it->second.distance - distance_template[j]);

				point_it->second.errors.push_back(error_temp);
				path_3D path_temp = point_it->second;

				//If it is an acceptable point, we will put it in the points to consider in our cyclic path
				if (error_temp < epsilon){
					store_mapper[j].push_back(path_temp);
				}


			}
		}



		for (std::map<int, path_3D>::iterator point_it = point_mapper_stereo.begin(); point_it != point_mapper_stereo.end(); ++point_it){
			std::cout << point_it->first << "-> " << "(" << point_it->second.id_begin << "," << point_it->second.id_end << ")" << ".Distance : " << point_it->second.distance << std::endl;
			for (int l = 0; l < point_it->second.errors.size(); l++){
				std::cout << point_it->second.errors.at(l) << " ";
			}


			std::cout << std::endl;
		}


		//This variable will store all the possible loop indices. And will will compare 
		//will loop will give us the lowest difference in distance.
		//
#if 0 //FOR 3 nodes only
		std::vector<cv::Point3i> loop_indices;

		loop_indices.push_back(cv::Point3i(0, 1, 2));
		loop_indices.push_back(cv::Point3i(0, 2, 1));
		loop_indices.push_back(cv::Point3i(1, 2, 0));
		loop_indices.push_back(cv::Point3i(1, 0, 2));
		loop_indices.push_back(cv::Point3i(2, 0, 1));
		loop_indices.push_back(cv::Point3i(2, 1, 0));

#endif // 0 //FOR 3 nodes only
		std::vector<std::vector<int>> loop_indices;

		for (int i1 = 0; i1 < 4; i1++){
			for (int i2 = 0; i2 < 4; i2++){
				if (i2 != i1){
					for (int i3 = 0; i3 < 4; i3++){
						if ((i3 != i2) && (i3 != i1)){
							for (int i4 = 0; i4 < 4; i4++){
								if ((i4 != i3) && (i4 != i2) && (i4 != i1)){
									std::vector<int> dummy_indices;
									dummy_indices.push_back(i1);
									dummy_indices.push_back(i2);
									dummy_indices.push_back(i3);
									dummy_indices.push_back(i4);
									loop_indices.push_back(dummy_indices);
								}

							}
						}

					}
				}
			}

		}
		double distance = INFINITY;
		int d_counter = 0;
		//std::vector<path_3D> final_answer;

		if (store_mapper.size() == num_pose_points){
			for (auto loop_it = loop_indices.begin(); loop_it != loop_indices.end(); ++loop_it){

				//We will loop through the following different possibilities
				//cc->dd->ee
#if 0//if 3 balls to track
				for (auto cc = store_mapper.at(loop_it->x).begin(); cc != store_mapper.at(loop_it->x).end(); ++cc){
					for (auto dd = store_mapper.at(loop_it->y).begin(); dd != store_mapper.at(loop_it->y).end(); ++dd){
						for (auto ee = store_mapper.at(loop_it->z).begin(); ee != store_mapper.at(loop_it->z).end(); ++ee){
							double distance_test;
							distance_test = cv::abs(cc->distance - distance_template[0]) + cv::abs(dd->distance - distance_template[1]) + cv::abs(ee->distance - distance_template[2]);
							if (distance_test < distance){
								distance = distance_test;


								final_answer.clear();
								final_answer.push_back(*cc);
								final_answer.push_back(*dd);
								final_answer.push_back(*ee);
							}
						}
					}
				}
#endif // 0//if 3 balls to track
				for (auto cc = store_mapper.at(loop_it->at(0)).begin(); cc != store_mapper.at(loop_it->at(0)).end(); ++cc){
					for (auto dd = store_mapper.at(loop_it->at(1)).begin(); dd != store_mapper.at(loop_it->at(1)).end(); ++dd){
						for (auto ee = store_mapper.at(loop_it->at(2)).begin(); ee != store_mapper.at(loop_it->at(2)).end(); ++ee){
							for (auto ff = store_mapper.at(loop_it->at(3)).begin(); ff != store_mapper.at(loop_it->at(3)).end(); ++ff){
								double distance_test;
								distance_test = cv::abs(cc->distance - distance_template[0]) + cv::abs(dd->distance - distance_template[1]) + cv::abs(ee->distance - distance_template[2]) + cv::abs(ff->distance - distance_template[3]);
								std::vector<path_3D> check_vector;
								check_vector.push_back(*cc);
								check_vector.push_back(*dd);
								check_vector.push_back(*ee);
								check_vector.push_back(*ff);



								//int array_check[8] = { cc->id_begin, cc->id_end, dd->id_begin, dd->id_end, ee->id_begin, ee->id_end, ff->id_begin, ff->id_end };

								
								if (distance_test < distance){
									if (is_path_closed(check_vector)){
										distance = distance_test;


										final_answer.clear();
										final_answer.push_back(*cc);
										final_answer.push_back(*dd);
										final_answer.push_back(*ee);
										final_answer.push_back(*ff);
									}
								}
							}
						}
					}
				}

				//store_mapper.at(loop_it->x);

			}
			std::cout << "ANSWER" << std::endl;
			if (!final_answer.empty()){
				for (auto mm = final_answer.begin(); mm != final_answer.end() - 1; ++mm){
					//std::cout << "(" << mm->id_begin << "," << mm->id_end << ")" << ".Distance : " << mm->distance << std::endl;
					//case 1: (a,b) (c,b), this is the only case that has the ends the same
					if (mm->id_end == (mm + 1)->id_end){
						std::swap((mm + 1)->id_begin, (mm + 1)->id_end);
					}
					//case 2: (b,a) (b,c), only time that the first two are the same
					else if (mm->id_begin == (mm + 1)->id_begin){
						std::swap(mm->id_begin, mm->id_end);
					}

					//case 3:(b,a) (c,b), only that that the first in bracket 1 is the same as the second in bracket 2
					else if (mm->id_begin == (mm + 1)->id_end){
						std::swap(mm->id_begin, mm->id_end);
						std::swap((mm + 1)->id_begin, (mm + 1)->id_end);
					}


				}
			}


			std::cout << "ANSWER" << std::endl;

			//We will output the 3D points in order.
			std::vector<cv::Point3f> output_3d;

			for (auto mm = final_answer.begin(); mm != final_answer.end(); ++mm){
				std::cout << "(" << mm->id_begin << "," << mm->id_end << ")" << ".Distance : " << mm->distance << std::endl;
				output_3d.push_back(real_3d_points[mm->id_begin]);
			}
			ordered_world_points = output_3d;
			return output_3d;
		}
		std::vector<cv::Point3f> s;
		return s;
		/*	std::cout << "STORED MAPPER" << std::endl;
			double distance = INFINITY;
			for (auto mapper_it = store_mapper.begin(); mapper_it != store_mapper.end(); ++mapper_it){
			for (auto cc = mapper_it->second.begin(); cc != mapper_it->second.end(); ++cc){

			}
			}*/

}

bool get_stereo_depth::is_path_closed(std::vector<path_3D> check_vector_input){
		for (auto mm = check_vector_input.begin(); mm != check_vector_input.end() - 1; ++mm){
			//std::cout << "(" << mm->id_begin << "," << mm->id_end << ")" << ".Distance : " << mm->distance << std::endl;
			//case 1: (a,b) (c,b), this is the only case that has the ends the same
			if (mm->id_end == (mm + 1)->id_begin){

				if (mm->id_begin == (mm + 1)->id_end){
					return false;
				}
				/*else{
					std::swap(mm->id_begin, (mm + 1)->id_end);
				}*/
			} else if (mm->id_begin==(mm+1)->id_begin){
				if (mm->id_end == (mm + 1)->id_end){
					return false;
				}
			}else if (mm->id_end == (mm + 1)->id_end){
				std::swap((mm + 1)->id_begin, (mm + 1)->id_end);
			}
			//case 2: (b,a) (b,c), only time that the first two are the same
			else if (mm->id_begin == (mm + 1)->id_begin){
				std::swap(mm->id_begin, mm->id_end);
			}

			//case 3:(b,a) (c,b), only that that the first in bracket 1 is the same as the second in bracket 2
			else if (mm->id_begin == (mm + 1)->id_end){
				std::swap(mm->id_begin, mm->id_end);
				std::swap((mm + 1)->id_begin, (mm + 1)->id_end);
			}		
			else{
				return false;
			}
		}
		return true;
}


void get_stereo_depth::return_ordered_imagepoints(std::vector<cv::Point2f> *left_p, std::vector<cv::Point2f>  *right_p){
		//std::vector<cv::Point2f> temp_left, temp_right;
		for (auto mm = final_answer.begin(); mm != final_answer.end(); ++mm){
			//..std::cout << "(" << mm->id_begin << "," << mm->id_end << ")" << ".Distance : " << mm->distance << std::endl;
			left_p->push_back(left_points_ordered_distorted[mm->id_begin]);
			right_p->push_back(right_points_ordered_distorted[mm->id_begin]);
		}
}

void get_stereo_depth::clear_points(){
		left_points_ordered.clear();
		right_points_ordered.clear();
		world_points.clear();
		final_answer.clear();
}

std::vector<cv::Point2f>  get_stereo_depth::return_distorted_ordered_points(direction d_in){
		if (d_in = direction::LEFT){
			return left_points_ordered_distorted;
		}
		else if (d_in = direction::RIGHT){
			return right_points_ordered_distorted;
		}
}


void get_stereo_depth::get_image(cv::Mat *im_left_in, cv::Mat *im_right_in){
		im_left = *im_left_in;
		im_right = *im_right_in;

}


void get_stereo_depth::detect_key_points(){
		//------------------------LEFT-----------------------------
		
#if 1
		cv::resize(im_left, im_left, cv::Size(cam_W / 2.0, cam_H / 2.0));
		
		//cv::GaussianBlur(im_left, im_left, cv::Size(5, 5), 3, 3);

		cv::cvtColor(im_left, color_HSV, cv::COLOR_BGR2HSV);
		
		//cv::bilateralFilter(color_HSV, color_HSV_out, 8, 15, 15);
		//cv::bilateralFilter(color_HSV_out, color_HSV, 8, 15, 15);
		//cv::GaussianBlur(color_HSV, color_HSV, cv::Size(3, 3), 3, 3);

		cv::inRange(color_HSV, cv::Scalar(60 - 25, 100, 50), cv::Scalar(60 + 25, 255, 255), gray_image);
		gray_image = 255 - gray_image;
		cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 2, 2);
		cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 3, 3);

		cv::imshow("left_gary", gray_image);

		//cv::resize(gray, gray, cv::Size(camWidth / 2.0, camHeight / 2.0));

		detector_blob->detect(gray_image, left_points);
		//cv::drawKeypoints(gray_image, left_points, gray_image);
		//cv::imshow("gray left", gray_image);
		/*for (auto cc = keypoints_left.begin(); cc != keypoints_left.end(); ++cc){

		cc->pt = cc->pt*2.0;
		}*/



		cv::resize(im_right, im_right, cv::Size(cam_W / 2.0, cam_H / 2.0));

		
		//-------------------------RIGHT----------------------------
		cv::cvtColor(im_right, color_HSV, cv::COLOR_BGR2HSV);
		
	// color_HSV_out;
		//cv::bilateralFilter(color_HSV, color_HSV_out, 8, 15, 15);
		//cv::bilateralFilter(color_HSV_out, color_HSV, 8, 15, 15);
		//cv::GaussianBlur(color_HSV, color_HSV, cv::Size(3, 3), 3, 3);cvInRangeS(imgHSV, cvScalar(20, 100, 100), cvScalar(30, 255, 255),
		cv::inRange(color_HSV, cv::Scalar(60 - 25, 100, 50), cv::Scalar(60 + 25, 255, 255), gray_image);
		//cv::Mat image_yellow;
		//cv::inRange(color_HSV, cv::Scalar(15, 100, 100), cv::Scalar(35, 255, 255), image_yellow);
		//cv::imshow("yellow",image_yellow);
		gray_image = 255 - gray_image;
		cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 2, 2);
		cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 3, 3);
		detector_blob->detect(gray_image, right_points);
		//cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 3, 3);

		cv::imshow("right_gary", gray_image);
		cv::waitKey(1);


#endif // 0
		/*cv::Mat gray_left, gray_right;
		cv::GaussianBlur(im_right, im_right, cv::Size(5, 5), 3, 3);
		cv::GaussianBlur(im_left, im_left, cv::Size(5, 5), 3, 3);
		cv::cvtColor(im_left, gray_left, cv::COLOR_BGR2HSV);
		cv::cvtColor(im_right, gray_right, cv::COLOR_BGR2HSV);

		*/

		
		//-------------------------RIGHT----------------------------
		//cv::cvtColor(im_right, color_HSV, cv::COLOR_BGR2HSV);
		//cv::GaussianBlur(color_HSV, color_HSV, cv::Size(3, 3), 3, 3);cvInRangeS(imgHSV, cvScalar(20, 100, 100), cvScalar(30, 255, 255),
		//cv::inRange(color_HSV, cv::Scalar(60 - 25, 100, 50), cv::Scalar(60 + 25, 255, 255), gray_image);
		/// GPU DETECTION
		//cv::resize(im_left, im_left, cv::Size(cam_W / 1.5, cam_H / 1.5));
		//cv::resize(im_right, im_right, cv::Size(cam_W / 1.5, cam_H / 1.5));
		/*img1.upload(im_left);
		img2.upload(im_right);*/

		////cv::cuda::meanShiftFiltering(img1, img1, 2, 3);
		////cv::cuda::meanShiftFiltering(img2, img2, 2, 3);
		//for (int i = 0; i < 2; i++){
		//	cv::cuda::bilateralFilter(img1, img1, 8, 15, 15);
		//	cv::cuda::bilateralFilter(img2, img2, 8, 15, 15);
		//}
	
		//cv::cuda::cvtColor(img1, img1, cv::COLOR_BGR2HSV);
		//cv::cuda::cvtColor(img2, img2, cv::COLOR_BGR2HSV);
		//
		//img1.download(im_left);
		//img2.download(im_right);

		//cv::inRange(im_left, cv::Scalar(60 - 35, 100, 50), cv::Scalar(60 + 35, 255, 255), gray_image);
		//
		//gray_image = 255 - gray_image;
		//img1.upload(gray_image);
		//cv::inRange(im_right, cv::Scalar(60 - 35, 100, 50), cv::Scalar(60 + 35, 255, 255), gray_image);
		//
		//
		//gray_image = 255 - gray_image;
		//img2.upload(gray_image);
		////  
		//
		//cuda_gauss_filter->apply(img1, img1);
		//cuda_gauss_filter->apply(img2, img2);
		//std::vector<cv::Point3f> gpu_vector_left, gpu_vector_right;

		//cuda_gauss_filter->apply(img1, img1);
		//cuda_gauss_filter->apply(img2, img2);
		//img1.download(gray_image);
		//cv::imshow("left___", gray_image);
		//img2.download(gray_image);
		//cv::imshow("right_____", gray_image);
		//
		////hough_circle->detect(img1, d_circle_left);
		////hough_circle->detect(img2, d_circle_right);
		//
		//std::vector<cv::Point3f> circles_gpu_left, circles_gpu_right;
		//if (!d_circle_left.empty())
		//{
		//	circles_gpu_left.resize(d_circle_left.cols);
		//	//Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
		//	d_circle_left.download(circles_gpu_left);

		//	for (auto cc = circles_gpu_left.begin(); cc != circles_gpu_left.end(); ++cc){
		//		cv::circle(im_left, cv::Point2f(cc->x, cc->y), cc->z, cv::Scalar(1, 200, 100), 2);
		//		
		//	}
		//}

		//if (!d_circle_right.empty())
		//{
		//	circles_gpu_right.resize(d_circle_right.cols);
		//	//Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
		//	d_circle_right.download(circles_gpu_right);

		//	for (auto cc = circles_gpu_right.begin(); cc != circles_gpu_right.end(); ++cc){
		//		cv::circle(im_right, cv::Point2f(cc->x, cc->y), cc->z, cv::Scalar(1, 200, 100), 2);

		//	}
		//}
		//cv::imshow("left cirlce", im_left);
		//cv::imshow("right circle", im_right);

		//cv::resize(gray, gray, cv::Size(camWidth / 2.0, camHeight / 2.0));
		//detector_blob->detect(gray_image, right_points);
		//cv::drawKeypoints(gray_image, right_points,gray_image);
		//cv::imshow("gray rihgt", gray_image);
		


		//Chess Board
#if 0
		cv::resize(im_right, im_right, cv::Size(cam_W / 4.0, cam_H / 4.0));
		cv::resize(im_left, im_left, cv::Size(cam_W / 4.0, cam_H / 4.0));
		cv::Mat im_right_gray, im_left_gray;
		cv::Size pattersize(3, 4);
		cv::cvtColor(im_right, im_right_gray, CV_BGRA2GRAY);
		cv::cvtColor(im_left, im_left_gray, CV_BGRA2GRAY);
		//find_aruco_center_ovr(im_right);
		//find_aruco_center_ovr(im_left);
		bool left_found = cv::findChessboardCorners(im_left_gray, pattersize, current_corners_left);
		bool right_found = cv::findChessboardCorners(im_right_gray, pattersize, current_corners_right);
		if (left_found&&right_found){
			for (auto cc = current_corners_left.begin(); cc != current_corners_left.end(); ++cc){
				*cc = (*cc)*1.5;
			}
			for (auto cc = current_corners_right.begin(); cc != current_corners_right.end(); ++cc){
				*cc = (*cc)*1.5;
			}

			print_chess_board = true;
		}
		else{
			print_chess_board = false;
		}
#endif // 0

		get_points_keypoints(left_points, right_points);
}


void get_stereo_depth::stereo_SVD(void){

		std::vector<cv::Point2f> project_2D_left, project_2D_right;
		
		
		if (same_points()){
			find_correspondance(F); 
			//stereo_class.find_world_points();
			find_world_points_triangulation();

			std::vector<cv::Point3f> world_points = print_world_point();
			std::vector<cv::Point3f> ordered_world_points;


			pcl::PointCloud<pcl::PointXYZ>::Ptr position(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto cc = world_points.begin(); cc != world_points.end(); ++cc){
				pcl::PointXYZ insert;
				insert.x = cc->x / 1000.0;
				insert.y = cc->y / 1000.0;
				insert.z = cc->z / 1000.0;
				*cc = *cc / 1000.0;
				position->push_back(insert);
			}
			/*if (position->size() == 2){
				double dx = (position->at(0).x - position->at(1).x);
				double dy = (position->at(0).y - position->at(1).y);
				double dz = (position->at(0).z - position->at(1).z);

				double norm_ = sqrtf(dx*dx + dy*dy + dz*dz);
				cout << "NORM: ";
				cout << norm_ << endl;
			}*/
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr world_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
			if (world_points.size() == num_tracker_points){
				ordered_world_points = rigid_pose_estimation(num_tracker_points, world_points);


			}
			if ((world_points.size() == num_tracker_points) && (!world_points.empty())){
				if (ordered_world_points.size() == num_tracker_points){

					for (auto cc = ordered_world_points.begin(); cc != ordered_world_points.end(); ++cc){
						world_points_ptr->push_back(pcl::PointXYZ(cc->x, cc->y, cc->z));
					}
					/*SVD_estimate.estimateRigidTransformation(*green_points, *world_points_ptr, transformation_mat);
					pcl::transformPointCloud(*green_points, *temp_xyz, transformation_mat);
*/
					SVD_estimate.estimateRigidTransformation(*green_points, *world_points_ptr, transformation_mat);
					pcl::transformPointCloud(*green_points, *temp_xyz, transformation_mat);
					
				}
			}
		}
}


void get_stereo_depth::remap_image(OVR::OvrvisionSetting *ovr_setting_ptr,cv::Mat *im_left_output,cv::Mat *im_right_output){
		cv::remap(*im_left_output, *im_left_output, map1x[0], map1y[0], cv::INTER_LINEAR);
		cv::remap(*im_right_output, *im_right_output, map1x[1], map1y[1], cv::INTER_LINEAR);
		/**im_left_output_gpu.upload(*im_left_output);
		*im_right_output_gpu.upload(*im_right_output);
		*/

		std::vector<cv::Point2f> temp_left, temp_right;
		return_ordered_imagepoints(&temp_left, &temp_right);
		pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_pointer(new pcl::PointCloud<pcl::PointXYZ>);
		if (!temp_left.empty() && !temp_right.empty()){
			temp_left_old = temp_left;
			temp_right_old = temp_right;
		}
/*
		virtual_pointer->push_back(pcl::PointXYZ(0.09743, 0.0, 0.0));
		virtual_pointer->push_back(pcl::PointXYZ(0.09743 + 0.05, 0.0, 0.0));*/


		virtual_pointer->push_back(pcl::PointXYZ(110.47 / 1000.0, (-1.0+4.0) / 1000.0, -35.24 / 1000.0));

		virtual_pointer->push_back(pcl::PointXYZ(110.47 / 1000.0, (1.0 + 4.0) / 1000.0, -35.24 / 1000.0));


		virtual_pointer->push_back(pcl::PointXYZ((110.47 + 150.0) / 1000.0, (-1.0 + 19) / 1000.0, -36.0 / 1000.0));
		virtual_pointer->push_back(pcl::PointXYZ((110.47 + 150.0) / 1000.0, (1.0 + 19) / 1000.0, -36.0 / 1000.0));
		virtual_pointer->push_back(pcl::PointXYZ((110.47 + 150.0) / 1000.0, 19 / 1000.0, -36.0 / 1000.0));

		pcl::transformPointCloud(*virtual_pointer, *virtual_pointer, transformation_mat);
		std::vector<cv::Point3f> temp_xyz_vector;
		for (auto ll = temp_xyz->begin(); ll != temp_xyz->end(); ++ll){
			temp_xyz_vector.push_back(cv::Point3f(ll->x, ll->y, ll->z));
		}

	
		
		
		if (temp_left.size() >= num_tracker_points){
			//distorted_left_vector






			//std::vector<cv::Point3f>   ordered_world_points = return_ordered_world_point();
			/*pcl::PointCloud<pcl::PointXYZ>::Ptr image_points_left(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto l_p = distorted_left_vector.begin(); l_p != distorted_left_vector.end(); l_p++){
			image_points_left->push_back(pcl::PointXYZ(l_p->x, l_p->y, 1.0));
			}*/


			std::vector<cv::Point3f> green_cv_points;
			for (auto cc = virtual_pointer->begin(); cc != virtual_pointer->end(); ++cc){
				green_cv_points.push_back(cv::Point3f(cc->x, cc->y, cc->z));
			}
			probe_position = green_cv_points;
			cv::Mat rvec_left, tvec_left;
			

			

			cv::solvePnP(temp_xyz_vector, temp_left, A_L, D1, rvec_left, tvec_left, false, cv::SOLVEPNP_P3P);
			cv::projectPoints(green_cv_points, rvec_left, tvec_left, A_L, D1,  project_2D_left);
			//cout << "rvec: " << rvec_left << " tvec: " << tvec_left << endl;

		//	cout << transformation_mat << endl;
			ovr_setting_ptr->undistort_points(OVR::OV_CAMEYE_LEFT, cam_H, cam_W, &temp_left);
			ovr_setting_ptr->undistort_points(OVR::OV_CAMEYE_LEFT, cam_H, cam_W, &project_2D_left);

			
			if (ordered_world_points.size() == num_tracker_points){

				if (!project_2D_left.empty()){
					//ovr_setting->undistort_points(OVR::OV_CAMEYE_LEFT, camWidth, camHeight, &project_2D_left);
					//cv::line(*im_left_output, project_2D_left[0], project_2D_left[1], cv::Scalar(0, 200, 100), 5);
					cv::line(*im_left_output, project_2D_left[0], project_2D_left[2], cv::Scalar(255, 0, 255), 0.5, cv::LINE_AA);
					cv::line(*im_left_output, project_2D_left[1], project_2D_left[3], cv::Scalar(255, 0, 255), 0.5, cv::LINE_AA);
					cv::circle(*im_left_output, project_2D_left[4], 2, cv::Scalar(0, 255, 255), 5);
				}
			}
		}
		
		if (temp_right.size() >= num_tracker_points){
			//distorted_left_vector






			//std::vector<cv::Point3f>   ordered_world_points = return_ordered_world_point();
			/*pcl::PointCloud<pcl::PointXYZ>::Ptr image_points_left(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto l_p = distorted_right_vector.begin(); l_p != distorted_right_vector.end(); l_p++){
			image_points_right->push_back(pcl::PointXYZ(l_p->x, l_p->y, 1.0));
			}*/


			std::vector<cv::Point3f> green_cv_points;
			for (auto cc = virtual_pointer->begin(); cc != virtual_pointer->end(); ++cc){
				green_cv_points.push_back(cv::Point3f(cc->x, cc->y, cc->z));
			}
			probe_position = green_cv_points;
			cv::Mat rvec_right, tvec_right;
			cv::solvePnP(temp_xyz_vector, temp_right, A_R, D2, rvec_right, tvec_right, false, cv::SOLVEPNP_P3P);
			cv::projectPoints(green_cv_points, rvec_right, tvec_right, A_R, D2, project_2D_right);
			//cout << "rvec: " << rvec_right << " tvec: " << tvec_right << endl;




			ovr_setting_ptr->undistort_points(OVR::OV_CAMEYE_RIGHT, cam_W, cam_H, &temp_right);
			ovr_setting_ptr->undistort_points(OVR::OV_CAMEYE_RIGHT, cam_W, cam_H, &project_2D_right);
			if (ordered_world_points.size() == num_tracker_points){

				if (!project_2D_right.empty()){
					//ovr_setting->undistort_points(OVR::OV_CAMEYE_right, camWidth, camHeight, &project_2D_right);
					//cv::line(*im_right_output, project_2D_right[0], project_2D_right[1], cv::Scalar(0, 200, 100), 5);
					cv::line(*im_right_output, project_2D_right[0], project_2D_right[2], cv::Scalar(255, 0, 255), 0.5, cv::LINE_AA);
					cv::line(*im_right_output, project_2D_right[1], project_2D_right[3], cv::Scalar(255, 0, 255), 0.5, cv::LINE_AA);
					cv::circle(*im_right_output, project_2D_right[4], 2, cv::Scalar(0, 255, 255), 5);
				}
			}
		}
		int counter_text = 0;
		for (auto cc = temp_left.begin(); cc != temp_left.end(); ++cc){

			cv::circle(*im_left_output, *cc, 3, cv::Scalar(0, 200, 200), 2);
			cv::putText(*im_left_output, std::to_string(counter_text++), *cc, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(200, 100, 0), 2);

		}
		counter_text = 0;
		for (auto cc = temp_right.begin(); cc != temp_right.end(); ++cc){
			cv::circle(*im_right_output, *cc, 3, cv::Scalar(0, 200, 200), 2);
			cv::putText(*im_right_output, std::to_string(counter_text++), *cc, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(200, 100, 0), 2);
		}

		
		
}