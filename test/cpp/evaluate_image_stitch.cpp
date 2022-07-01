#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

const char* params
    = "{ help h         |       | print usage }"
      "{ @image1         |       | path to the source image }"
      "{ @image2         |       | path to the desired image }";

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, params);

    if (parser.has("help"))
    {
        parser.about("Evaluation for two camera pose based on image stitichin. \n");
        parser.printMessage();
        return 0;
    }

    //// get information: images
    Mat image_1 = imread( parser.get<String>(0));
    Mat image_2 = imread( parser.get<String>(1));

    // ======================= for desired frame ======================= //

    // get IMU pose in odom frame
    Eigen::Quaterniond q_O_I2(0.0592259,-0.994263,0.0670564,0.0586189);//w,x,y,z
    Eigen::Vector3d p_O_I2 = Eigen::Vector3d(-0.448964 ,0.0624866 ,0.0992306);
    Eigen::Matrix4d T_O_I2 = Eigen::Matrix4d::Identity();
    T_O_I2.block(0,0,3,3) = q_O_I2.toRotationMatrix();
    T_O_I2.block(0,3,3,1) = p_O_I2;
    // get IMU and camera transformation
    Eigen::Matrix4d T_I2_C2;
    T_I2_C2 <<  0.007862367268,  0.999968129426, -0.001386835635,  0.310759207458,
                0.999953212142, -0.007854419126,  0.005646381758,  0.122348956946,
                0.005635309016, -0.001431164674, -0.999983097387, -0.277866355171,
                0.000000000000,  0.000000000000,  0.000000000000,  1.000000000000;

    // ======================= for source frame ======================= //

    // get IMU pose in odom frame 
    Eigen::Quaterniond q_O_I1(0.00617853,-0.952931,-0.29449,0.0718312);//w,x,y,z
    Eigen::Vector3d p_O_I1 = Eigen::Vector3d(1.94397, -0.610414, 2.37808); 
    Eigen::Matrix4d T_O_I1 = Eigen::Matrix4d::Identity();
    T_O_I1.block(0,0,3,3) = q_O_I1.toRotationMatrix();
    T_O_I1.block(0,3,3,1) = p_O_I1;
    // get IMU and camera transformation
    Eigen::Matrix4d T_I1_C1 = T_I2_C2;

    // ======================= get two camera transformation ======================= //

    Eigen::Matrix4d T_C2_C1 = T_I2_C2.inverse() * T_O_I2.inverse() * T_O_I1 * T_I1_C1;
    Eigen::Matrix3d R_C2_C1 = T_C2_C1.block(0,0,3,3);
    Eigen::Vector3d p_C2_C1 = T_C2_C1.block(0,3,3,1);


    // ======================= get 3D points on camera frame 1 ======================= //
    Eigen::Vector4d P_D_1 = Eigen::Vector4d(0.642972, 0.642972 ,1.96,1);
    Eigen::Vector4d P_D_2 = Eigen::Vector4d(0.606702, -0.606702, 1.96, 1);
    Eigen::Vector4d P_D_3 = Eigen::Vector4d(-0.692431, -0.692432, 1.96, 1);

    Eigen::Matrix4d T_I1_D1;
    T_I1_D1 << -0.999928,   0.0116206, -0.00288846,   0.433828,
                0.0115923,    0.999886,  0.00962664,   0.086180,
                    0.003,  0.00959246,   -0.999949,  -0.302024,
                      0.0,         0.0,         0.0,        1.0;

    Eigen::Vector4d P_C_1 = T_I1_C1.inverse() * T_I1_D1 * P_D_1;
    Eigen::Vector4d P_C_2 = T_I1_C1.inverse() * T_I1_D1 * P_D_2;
    Eigen::Vector4d P_C_3 = T_I1_C1.inverse() * T_I1_D1 * P_D_3;


    // ======================= normal and distance ======================= //

    Eigen::Vector3d v1 = P_C_1.head(3) - P_C_2.head(3);
    Eigen::Vector3d v2 = P_C_1.head(3) - P_C_3.head(3);
    Eigen::Vector3d n = v1.cross(v2);
    n = n / n.norm();

    double d_inv = 1 / (1.96-0.1);

    // ======================= homography ======================= //

    // get camera intrinsics matrix
    Eigen::Matrix3d K;
    K <<1845.542490218492, 0, 825.6913274592704,
        0,  1846.046949112978, 605.8504151895138,
        0, 0, 1;

    // compute homography matrix
    Eigen::Matrix3d homography_euclidean = R_C2_C1 - p_C2_C1 * n.transpose() * d_inv;
    Eigen::Matrix3d homography = K * homography_euclidean * K.inverse();

    // convert to opencv format
    Mat homography_cv;
    eigen2cv(homography, homography_cv);
    homography_cv /= homography_cv.at<double>(2,2);

    std::cout<<"homography_cv: "<<homography_cv<<std::endl;

    Mat image_1_wrap;
    warpPerspective(image_1, image_1_wrap, homography_cv,cv::Size(2000,1000));
    imshow("Wraped image using homography computed from camera pose", image_1_wrap);

    imshow("source image", image_1);

    imshow("desired image", image_2);

    waitKey();

    return 0;
}

//! source: round 3-1, desired: round 1-1;
//  ./test_evaluation /home/lin/Desktop/temp/test/1614971729.354733.jpg /home/lin/Desktop/temp/test/1614971127.854733.jpg
