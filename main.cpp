
// Your First C++ Program

#include <iostream>
// #include "./include/objectTracker.hpp"
// #include <mot_config_files.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

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
    
bool debug_flag=false;



void getExtrinsic(cv::Mat_<cv::Point2d> distorted_points, std::vector<cv::Point3d > world_points, cv::Mat& rvec, cv::Mat& tvec,cv::Mat& rotation_matrix){

    double zero_dist[]= {0, 0, 0, 0, 0};
    cv::Mat_<double> zero_dist_mat(1, 5, zero_dist);
   
    cv::Mat_<cv::Point2d> undistortedPoints;
    cv::undistortPoints(distorted_points, undistortedPoints, intrinsic_calibration_matrix,distortion_matrix,cv::noArray(), intrinsic_calibration_matrix);
      
    cv::solvePnP(world_points, undistortedPoints, intrinsic_calibration_matrix, zero_dist_mat, rvec, tvec);

    cv::Rodrigues(rvec,rotation_matrix);

    if(debug_flag){
        std::cout<<std::endl<<"Extrinsic Calculation"<<std::endl;
        std::cout<<std::endl<<"RVEC"<<std::endl;
        cv::print(rvec);
        std::cout<<std::endl<<"TVEC"<<std::endl;
        cv::print(tvec);
        std::cout<<std::endl<<"ROTATION MATRIX"<<std::endl;
        cv::print(rotation_matrix);
    }

}

void old_calculate( cv::Mat rot_m,cv::Mat input_tvec,cv::Mat  img_points, std::vector<cv::Point3d > world_points ){
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
        index++;
    }
}

void calculate_projection_mat(cv::Mat rot_m,cv::Mat input_tvec){
    cv::Mat projection_matrix,projection_without_z;
    
    cv::hconcat(rot_m,input_tvec,rot_m);

    projection_matrix=intrinsic_calibration_matrix*rot_m;

    cv::Mat temp_erster_teil,temp_letzter_teil;
    projection_matrix(cv::Range(0, projection_matrix.rows), cv::Range(0, projection_matrix.cols-2 )).copyTo(temp_erster_teil);
    projection_matrix(cv::Range(0, projection_matrix.rows), cv::Range(projection_matrix.cols-1, projection_matrix.cols )).copyTo(temp_letzter_teil);
 
    cv::hconcat(temp_erster_teil,temp_letzter_teil,temp_erster_teil);

    temp_erster_teil.copyTo(projection_without_z);
   
    


    if(debug_flag){
        std::cout<<std::endl<<"CALCULATE PROJECTION"<<std::endl;
        std::cout<<std::endl<<"INPUT Rotation Matrix"<<std::endl;
        print(rot_m);
        std::cout<<std::endl<<"INPUT TVEC"<<std::endl;
        print(input_tvec);
        std::cout<<std::endl<<"INPUT Intrinsic_Calib"<<std::endl;        
        print(intrinsic_calibration_matrix);
        std::cout<<std::endl<<"Projection Matrix="<<std::endl;
        print(projection_matrix);
        std::cout<<std::endl<<"Projection Matrix without Z"<<std::endl;
        cv::print(projection_without_z);
        std::cout<<std::endl<<"Inverse Projection Matrix without Z"<<std::endl;
        print(projection_without_z.inv());
    }


}

double inverse_projection[] = {0.001033345173586479, 0.00018399363370939, -1.147897923609774,
 -6.54057468801172e-05, -0.0009437968757491468, 1.984220419748764,
 4.154784468431722e-05, 0.0008933007469281109, -0.8734918697727939};
cv::Mat_<double> inverse_projection_mat(3, 3, inverse_projection);

void test_projection(cv::Mat_<cv::Point2d> pointMatrix,std::vector<cv::Point3d > world_points){
    int index=0;

    cv::Mat undistortedPoints;
    cv::undistortPoints(pointMatrix, undistortedPoints,intrinsic_calibration_matrix,distortion_matrix,cv::noArray(),intrinsic_calibration_matrix);
    
    for(auto p:pointMatrix){
        cv::Mat uvPoint = (cv::Mat_<double>(3,1) <<p.x, p.y, 1); // u = 363, v = 222, got this point using mouse callback

        cv::Mat result=inverse_projection_mat*uvPoint;
        double xGround=result.at<double>(0)/result.at<double>(2);
        double yGround=result.at<double>(1)/result.at<double>(2);
        std::cout<<"x="<<std::to_string(xGround)<<", x_offset"<<std::to_string(xGround-world_points[index].x)<<" ,y="
        <<std::to_string(yGround)<<" ,y_offset="<<std::to_string(world_points[index].y-yGround)<<std::endl;
        
        index++;
    }
}



int main() {
    // FHAC::objectTracker myTracker;
    // myTracker.test("/home/fze2/Desktop/");

    // labelling the position of corresponding feature points on the input image.
    std::vector<cv::Point2d>  srcImagePoints = {cv::Point2d(957.1643,  1161.1813),
    cv::Point2d(958.51874, 1187.1802 ),
    cv::Point2d(894.47974, 1163.0116 ),
    cv::Point2d(889.74634, 1188.9985),
    cv::Point2d(832.6407,  1162.851),
    cv::Point2d(821.0164,  1188.3483)};
    cv::Mat_<cv::Point2d> distortedpoints(srcImagePoints);

    std::vector<cv::Point3d > world_points;
    world_points.push_back(cv::Point3d(0.255, 3.2,0));
    world_points.push_back(cv::Point3d(0.245,2.72,0));   
    world_points.push_back(cv::Point3d(-0.014,3.21,0));   
    world_points.push_back(cv::Point3d(0,2.73,0));   
    world_points.push_back(cv::Point3d(-0.32, 3.225, 0));   
    world_points.push_back(cv::Point3d(-0.32,2.75,0));

    cv::Mat rvec(1,3,cv::DataType<double>::type);
    
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat rotation_matrix(3,3,cv::DataType<double>::type);

    debug_flag=true;
    getExtrinsic(distortedpoints,world_points,rvec,tvec,rotation_matrix);
    calculate_projection_mat(rotation_matrix,tvec);
 
    return 0;
}

