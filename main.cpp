
// Your First C++ Program

#include <iostream>
// #include "./include/objectTracker.hpp"
// #include <mot_config_files.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

// double rotation_values[] = {0.998915, -0.00823395, -0.04583641,
// -0.04174363, 0.27802867, -0.9596653 ,
//  0.02064567, 0.9605375, 0.2773833 };
// cv::Mat_<double> rotation_matrix(3, 3, rotation_values);


// double transform_values[] = {-0.21408474,
//  1.4380101 ,
//  1.008065 };
// cv::Mat transform_matrix(1,3,cv::DataType<double>::type,transform_values);
using std::bind;
using std::placeholders::_1;

//ROS Calibrator Output camera_matrix=intrinsic values 
double intrinsic_values[] = {976.637135, 0.000000, 952.880210, 
                        0.000000, 983.500564, 649.387647, 
                        0.000000, 0.000000, 1.000000};
cv::Mat_<double> intrinsic_calibration_matrix(3, 3, intrinsic_values);


//distortion coefficients
// (k1,k2,p1,p2,k3)
double dist_coefficients[]= {-0.270602, 0.051047, -0.000236, 0.000302, 0.00000};
cv::Mat_<double> distortion_matrix(1, 5, dist_coefficients);
    


void calculate_shit( cv::Mat rot_m,cv::Mat input_tvec,cv::Mat img_points, std::vector<cv::Point3d > world_points);
void do_shit(){


    // labelling the position of corresponding feature points on the input image.
    std::vector<cv::Point2d>  srcImagePoints = {cv::Point2d(957.1643,  1161.1813),
    cv::Point2d(958.51874, 1187.1802 ),
    cv::Point2d(894.47974, 1163.0116 ),
    cv::Point2d(889.74634, 1188.9985),
    cv::Point2d(832.6407,  1162.851),
    cv::Point2d(821.0164,  1188.3483)};

    std::vector<cv::Point3d > world_points;
    world_points.push_back(cv::Point3d(0.255, 3.2,0));
    world_points.push_back(cv::Point3d(0.245,2.72,0));   
    world_points.push_back(cv::Point3d(-0.014,3.21,0));   
    world_points.push_back(cv::Point3d(0,2.73,0));   
    world_points.push_back(cv::Point3d(-0.32, 3.225, 0));   
    world_points.push_back(cv::Point3d(-0.32,2.75,0));   
    


    // std::vector<cv::Point2d>  undistorted_test = {cv::Point2d(957.44354, 1206.9542 ),
    // cv::Point2d(958.97003, 1241.543  ),
    // cv::Point2d(889.02637, 1210.124 ),
    // cv::Point2d( 883.08417, 1245.0016 ),
    // cv::Point2d(820.9494,  1212.3871 ),
    // cv::Point2d(806.45746, 1247.431)};    
    // cv::Mat test_undistorted(undistorted_test);

    double zero_dist[]= {0, 0, 0, 0, 0};
    cv::Mat_<double> zero_dist_mat(1, 5, zero_dist);
   
    cv::Mat_<cv::Point2d> distortedpoints(srcImagePoints);
    cv::Mat_<cv::Point2d> undistortedPoints;

    //    undistortedAndNormalizedPointMatrix = cv2.undistortPoints(imgpoints, cameraMatrix=Camera_matrix, distCoeffs= Dist_coefficients,P=Camera_matrix)
    cv::undistortPoints(distortedpoints, undistortedPoints, intrinsic_calibration_matrix,distortion_matrix,cv::noArray(), intrinsic_calibration_matrix);
    
    std::cout<<"Undistorted Points";
    cv::print(undistortedPoints);

    cv::Mat rvec(1,3,cv::DataType<double>::type);
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

    
    cv::solvePnP(world_points, undistortedPoints, intrinsic_calibration_matrix, zero_dist_mat, rvec, tvec);

    cv::Rodrigues(rvec,rotationMatrix);
    std::cout<<std::endl<<"Rvec"<<std::endl;
    cv::print(rvec);
    std::cout<<std::endl<<"Tvec"<<std::endl;
    cv::print(tvec);

    std::cout<<std::endl<<"Rot_Mat"<<std::endl;
    cv::print(rotationMatrix);


    calculate_shit(rotationMatrix,tvec,undistortedPoints,world_points);

}

void calculate_shit( cv::Mat rot_m,cv::Mat input_tvec,cv::Mat  img_points, std::vector<cv::Point3d > world_points ){
    int index=0;
 

    for (cv::MatIterator_<cv::Point2d> i = img_points.begin<cv::Point2d>(); i != img_points.end<cv::Point2d>(); ++i)
    {
        
        std::cout<<"img_x="<<std::to_string((*i).x)<<"img_y="<<std::to_string((*i).y)<<std::endl;
        cv::Mat uvPoint = (cv::Mat_<double>(3,1) <<(*i).x, (*i).y, 1); // u = 363, v = 222, got this point using mouse callback

        cv::Mat leftSideMat  = rot_m.inv() * intrinsic_calibration_matrix.inv() * uvPoint;
        cv::Mat rightSideMat = rot_m.inv() * input_tvec;

        double s = rightSideMat.at<double>(2,0)/leftSideMat.at<double>(2,0); 

        //std::cout << "P = " << rot_m.inv() * (s * intrinsic_calibration_matrix.inv() * uvPoint - input_tvec) << std::endl;
        cv::Mat b=rot_m.inv() * (s * intrinsic_calibration_matrix.inv() * uvPoint - input_tvec);
        //std::cout<<b.at<double>(1);
        //print(b);
        std::cout<<"x="<<std::to_string(b.at<double>(0))<<", x_offset"<<std::to_string(b.at<double>(0)-world_points[index].x)<<" ,y="
        <<std::to_string(b.at<double>(1))
        <<" ,y_offset="<<std::to_string(b.at<double>(1)-world_points[index].y)<<std::endl;
    }
}

void test_new_shit(cv::Mat rot_m,cv::Mat input_tvec,cv::Mat  img_points, std::vector<cv::Point3d > world_points ){
    
}

int main() {
    // FHAC::objectTracker myTracker;
    // myTracker.test("/home/fze2/Desktop/");
    do_shit();
 
    return 0;
}

