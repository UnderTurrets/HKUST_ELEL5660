#include <iostream>
#include <math.h>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <Eigen/SVD>
//Eigen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace aruco;
using namespace Eigen;

// global variables
double reproj_error;
MatrixXd cum_error = MatrixXd::Zero(6, 1);
int count_frame = 0;
int error_count = 0;


// Camera parameters
aruco::CameraParameters CamParam;
cv::Mat K, D;

// Aruco marker and detector
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;

// Topic subscribers and publishers
ros::Subscriber sub_img;
ros::Publisher pub_path_ref, pub_path;
nav_msgs::Path path_ref, path;

// This function publishes the pose results as a path.
void publishPath(
  const ros::Time& t,
  Vector3d& T,
  Quaterniond& Q,
  nav_msgs::Path& path,
  ros::Publisher& pub_path){
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = "world";
  ps_ptr->pose.position.x = T(0);
  ps_ptr->pose.position.y = T(1);
  ps_ptr->pose.position.z = T(2);
  ps_ptr->pose.orientation.x = Q.x();
  ps_ptr->pose.orientation.y = Q.y();
  ps_ptr->pose.orientation.z = Q.z();
  ps_ptr->pose.orientation.w = Q.w();

  path.header.stamp = t;
  path.header.frame_id = ps_ptr->header.frame_id;
  path.poses.push_back(*ps_ptr);
  pub_path.publish(path);
}

// Obtain the
cv::Point3f getPositionFromIndex(int idx, int nth){
  int idx_x = idx % 6, idx_y = idx / 6;
  double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
  double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
  return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

/* THIS IS WHAT YOU NEED TO WORK WITH */
void calculateReprojectionError(
  const vector<cv::Point3f> &pts_3, // the input 3D points
  const vector<cv::Point2f> &pts_2, // the input 2D features that are corresponding to the 3D points
  const cv::Mat R, // the under-estimated rotation matrix
  const cv::Mat t){ // the under-estimated translation{
  // 本地初始化重投影误差
  double sum=0;
  // 校正畸变，undistortPoints得到的是归一化相机坐标系
  vector<cv::Point2f> corrected_pts_2;
  cv::undistortPoints(pts_2, corrected_pts_2, K, D);
  for (size_t i = 0; i < pts_3.size(); i++){
    cv::Mat point_w(3, 1, CV_64FC1);
    point_w.at<double>(0, 0) = pts_3[i].x;
    point_w.at<double>(1, 0) = pts_3[i].y;
    point_w.at<double>(2, 0) = pts_3[i].z;
    // 转相机坐标系并归一化
    cv::Mat p_camera = (R * point_w + t);
    cv::Mat p_camera_homogeneous = p_camera/p_camera.at<double>(2);
    // 如果在像素坐标系下...
    cv::Mat p_pixel = K * p_camera_homogeneous;
    float fx = K.at<double>(0, 0);
    float fy = K.at<double>(1, 1);
    float cx = K.at<double>(0, 2);
    float cy = K.at<double>(1, 2);
    cv::Point2f temp_pts_2_pixel(fx*corrected_pts_2[i].x+cx,fy*corrected_pts_2[i].y+cy);
    sum = sum + sqrt(pow(temp_pts_2_pixel.x-p_pixel.at<double>(0),2)+pow(temp_pts_2_pixel.y-p_pixel.at<double>(1),2));
    
    // 如果在归一化相机坐标系下...
    // sum = sum + sqrt(pow(corrected_pts_2[i].x-p_camera_homogeneous.at<double>(0),2)+
    //                  pow(corrected_pts_2[i].y-p_camera_homogeneous.at<double>(1),2));
    }
    reproj_error = sum;
}

/* THIS IS THE POSE ESTIMATION FUNCTION YOU NEED TO IMPLEMENT */
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(
  const vector<int> &pts_id,
  const vector<cv::Point3f> &pts_3,
  const vector<cv::Point2f> &pts_2,
  const ros::Time& frame_time){
  /* Pose estimation using OpenCV's PnP function.
    * THIS IS YOUR WORK
    * The result will be used as reference. */
  // Use cv::solvePnPRansac() and cv::Rodrigues().
  cv::Mat r, rvec, t;
  cv::solvePnPRansac(pts_3, pts_2, K, D, rvec, t);
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d R_ref;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
        R_ref(i, j) = r.at<double>(i, j);
    }
  }
  Eigen::Vector3d eulerAngles_target = R_ref.eulerAngles(2, 0, 1);
  Eigen::VectorXd parameter_target(6,1);parameter_target << eulerAngles_target[0],eulerAngles_target[1],eulerAngles_target[2],t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0);
  
  // publish reference path. Use publishPath().
  Quaterniond Q_ref;
  Eigen::Vector3d t_ref;
  for(size_t i = 0; i < 3; i++)
    t_ref(i) = t.at<double>(i,0);
  Q_ref = R_ref.inverse();t_ref = -R_ref.inverse()*t_ref;
  publishPath(frame_time, t_ref, Q_ref, path_ref, pub_path_ref);

  /* THIS IS YOUR WORK*/
  /* Pose estimation using analytical Jacobian derivation.*/
  // Apply undistortion if needed. Use cv::undistortPoints().
  vector<cv::Point2f> corrected_pts_2;
  cv::undistortPoints(pts_2, corrected_pts_2, K, D);

  // Step 1: Work out the initial pose estimates using the linear solution.
  Eigen::MatrixXd A(2*pts_3.size(),9);
  for (uint i=0;i<pts_3.size();i++){
    A.block<2,9>(2*i,0) << pts_3[i].x, pts_3[i].y, 1, 0,0,0, -corrected_pts_2[i].x*pts_3[i].x,  -corrected_pts_2[i].x*pts_3[i].y, -corrected_pts_2[i].x,
                           0,0,0, pts_3[i].x, pts_3[i].y, 1, -corrected_pts_2[i].y*pts_3[i].x,  -corrected_pts_2[i].y*pts_3[i].y, -corrected_pts_2[i].y;
  }
  Eigen::JacobiSVD<MatrixXd> A_svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd init_solve = A_svd.matrixV().rightCols(1);
  Eigen::Matrix3d H_hat, H_orthogonal;
  H_hat << init_solve[0], init_solve[1], init_solve[2], init_solve[3], init_solve[4], init_solve[5], init_solve[6], init_solve[7], init_solve[8];
  if (H_hat(2,2)<0){
      H_hat = -H_hat;
  }
  H_orthogonal << H_hat.col(0), H_hat.col(1), H_hat.col(0).cross(H_hat.col(1));
  Eigen::JacobiSVD<MatrixXd> H_hat_svd(H_orthogonal, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d R_hat;Eigen::Vector3d T_hat;R_hat.setIdentity();T_hat.setZero();
  R_hat = H_hat_svd.matrixU()*(H_hat_svd.matrixV().transpose());
  T_hat = H_hat.col(2)/(H_hat.col(0).norm());

  // Step 2: Refine the pose estimates using non-linear optimization
  // 残差初始化
  Eigen::MatrixXd gamma(2, pts_3.size());
  // 雅可比矩阵的列表
  Eigen::MatrixXd JacobianMatrix[pts_3.size()];
  // 最大迭代次数
  int iter_threshold = 1000;
  // 待优化参数初始化
  Eigen::VectorXd parameters(6, 1);
  // 按psi,phi,theta顺序分解并连接它们
  Eigen::Vector3d eulerAngles = R_hat.eulerAngles(2, 0, 1);
  parameters << eulerAngles[0],eulerAngles[1],eulerAngles[2],T_hat[0],T_hat[1],T_hat[2];
  Eigen::MatrixXd tmp_J(2,6);
  for (int count = 0; count < iter_threshold ; count++){
    Eigen::MatrixXd An = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd bn = Eigen::MatrixXd::Zero(6,1);
    // 拆出参数简化计算
    double psi = parameters[0],phi = parameters[1],theta = parameters[2], t1=parameters[3], t2=parameters[4], t3=parameters[5];
    Eigen::Matrix3d temp_R;temp_R = AngleAxisd(psi, Vector3d::UnitZ()) * AngleAxisd(phi, Vector3d::UnitX()) * AngleAxisd(theta, Vector3d::UnitY());
    Eigen::Vector3d temp_T(t1,t2,t3);
    for (size_t i = 0;i<pts_3.size();i++){
      // compute residual,target在前，prediction在后
      Eigen::Vector3d pt_3_eigen(pts_3[i].x, pts_3[i].y, pts_3[i].z);
      Eigen::Vector3d p_camera = temp_R * pt_3_eigen + temp_T;
      Eigen::Vector3d p_camera_homogeneous = p_camera / p_camera[2];

      // 如果在像素坐标系下...
      // 貌似版本过低，无法使用这个函数，手动赋值
      Eigen::Matrix3d K_eigen;
      // Eigen::cv2eigen(K,K_eigen);
      for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j < 3; j++){
            K_eigen(i, j) = K.at<double>(i, j);
        }
      }
      Eigen::Vector3d p_pixel = K_eigen * p_camera_homogeneous;
      float fx = K.at<double>(0, 0);
      float fy = K.at<double>(1, 1);
      float cx = K.at<double>(0, 2);
      float cy = K.at<double>(1, 2);
      cv::Point2f temp_pts_2_pixel(fx*corrected_pts_2[i].x+cx,fy*corrected_pts_2[i].y+cy);
      gamma.block<2,1>(0,i) << temp_pts_2_pixel.x - p_pixel[0],
                              temp_pts_2_pixel.y - p_pixel[1];

      // 如果在归一化相机坐标系下...
      // gamma.block<2,1>(0,i) << corrected_pts_2[i].x - p_camera_homogeneous[0],
      //                         corrected_pts_2[i].y - p_camera_homogeneous[1];

      double x,y,z;x = pts_3[i].x;y = pts_3[i].y;z = pts_3[i].z;
      double dpcamera1_dpsi = (-sin(psi)*cos(theta)-cos(psi)*sin(phi)*sin(theta))*x
                        -cos(psi)*cos(phi)*y
                        + (-sin(psi)*cos(theta)+cos(theta)*cos(psi)*sin(phi))*z;
      double dpcamera3_dpsi = 0;
      double dpcamera1_dphi = -sin(psi)*cos(phi)*sin(theta)*x
                              +sin(phi)*sin(psi)*y
                              +cos(theta)*sin(psi)*cos(phi)*z;
      double dpcamera3_dphi = sin(phi)*sin(theta)*x
                              +cos(phi)*y
                              -sin(phi)*cos(theta)*z;
      double dpcamera1_dtheta = (-cos(psi)*sin(theta)-sin(psi)*sin(phi)*cos(theta))*x
                                +(-cos(psi)*sin(theta)-sin(theta)*sin(psi)*sin(phi))*z;
      double dpcamera3_dtheta = -cos(phi)*cos(theta)*x
                                -cos(phi)*sin(theta)*z;
      double dpcamera2_dpsi = (cos(theta)*cos(psi)-sin(psi)*sin(phi)*sin(theta))*x
                              -sin(psi)*cos(phi)*y
                              +(cos(psi)*sin(theta)+sin(psi)*sin(phi)*cos(theta))*z;
      double dpcamera2_dphi = cos(psi)*cos(phi)*sin(theta)*x
                              -cos(psi)*sin(phi)*y
                              -cos(psi)*cos(phi)*cos(theta)*z;
      double dpcamera2_dtheta = (-sin(theta)*sin(psi)+cos(psi)*sin(phi)*cos(theta))*x
                                +(sin(psi)*cos(theta)+cos(psi)*sin(theta)*sin(phi))*z;

      // 如果是在像素坐标系下...
      // du/dt1
      tmp_J(0, 3) = -fx*(1/p_camera[2]);
      // du/dt2
      tmp_J(0, 4) = 0;
      // du/dt3
      tmp_J(0, 5) = -fx*(-p_camera[0]/(p_camera[2]*p_camera[2]));
      // dv/dt1
      tmp_J(1, 3) = 0;
      // dv/dt2
      tmp_J(1, 4) = -fy*(1/p_camera[2]);
      // dv/dt3
      tmp_J(1, 5) = -fy*(-p_camera[1]/(p_camera[2]*p_camera[2]));
      // du/dpsi
      tmp_J(0, 0) = -fx*(dpcamera1_dpsi*p_camera[2]-p_camera[01]*dpcamera3_dpsi)/(p_camera[2]*p_camera[2]);
      // du/dphi
      tmp_J(0, 1) = -fx*(dpcamera1_dphi*p_camera[2]-p_camera[0]*dpcamera3_dphi)/(p_camera[2]*p_camera[2]);
      // du/dtheta
      tmp_J(0, 2) = -fx*(dpcamera1_dtheta*p_camera[2]-p_camera[0]*dpcamera3_dtheta)/(p_camera[2]*p_camera[2]);
      // dv/dpsi
      tmp_J(1, 0) = -fy*(dpcamera2_dpsi*p_camera[2]-p_camera[1]*dpcamera3_dpsi)/(p_camera[2]*p_camera[2]);
      // dv/dphi
      tmp_J(1, 1) = -fy*(dpcamera2_dphi*p_camera[2]-p_camera[1]*dpcamera3_dphi)/(p_camera[2]*p_camera[2]);
      // dv/dtheta
      tmp_J(1, 2) = -fy*(dpcamera2_dtheta*p_camera[2]-p_camera[1]*dpcamera3_dtheta)/(p_camera[2]*p_camera[2]);
      
      // 如果是在归一化相机坐标系下...
      // // du/dt1
      // tmp_J(0, 3) = -(1/p_camera[2]);
      // // du/dt2
      // tmp_J(0, 4) = 0;
      // // du/dt3
      // tmp_J(0, 5) = -(-p_camera[0]/(p_camera[2]*p_camera[2]));
      // // dv/dt1
      // tmp_J(1, 3) = 0;
      // // dv/dt2
      // tmp_J(1, 4) = -(1/p_camera[2]);
      // // dv/dt3
      // tmp_J(1, 5) = -(-p_camera[1]/(p_camera[2]*p_camera[2]));
      // // du/dpsi
      // tmp_J(0, 0) = -(dpcamera1_dpsi*p_camera[2]-p_camera[01]*dpcamera3_dpsi)/(p_camera[2]*p_camera[2]);
      // // du/dphi
      // tmp_J(0, 1) = -(dpcamera1_dphi*p_camera[2]-p_camera[0]*dpcamera3_dphi)/(p_camera[2]*p_camera[2]);
      // // du/dtheta
      // tmp_J(0, 2) = -(dpcamera1_dtheta*p_camera[2]-p_camera[0]*dpcamera3_dtheta)/(p_camera[2]*p_camera[2]);
      // // dv/dpsi
      // tmp_J(1, 0) = -(dpcamera2_dpsi*p_camera[2]-p_camera[1]*dpcamera3_dpsi)/(p_camera[2]*p_camera[2]);
      // // dv/dphi
      // tmp_J(1, 1) = -(dpcamera2_dphi*p_camera[2]-p_camera[1]*dpcamera3_dphi)/(p_camera[2]*p_camera[2]);
      // // dv/dtheta
      // tmp_J(1, 2) = -(dpcamera2_dtheta*p_camera[2]-p_camera[1]*dpcamera3_dtheta)/(p_camera[2]*p_camera[2]);
      JacobianMatrix[i] = tmp_J;
      An += tmp_J.transpose()*tmp_J;
      bn -= tmp_J.transpose()*gamma.col(i);
    }
    // update the optimization variables
    Eigen::VectorXd delta_p = An.inverse()*bn;
    parameters += delta_p;
    // std::cout << "update:" << delta_p<< endl;
    if (delta_p.cwiseAbs().sum() < 0.0001){
      break;
    }
    // 如果优化错误
    if ((parameters-parameter_target).norm()>0.4){
      parameters = parameter_target;
      break;
    }
  }

  Eigen::Matrix3d R;R = AngleAxisd(parameters[0], Vector3d::UnitZ()) * AngleAxisd(parameters[1], Vector3d::UnitX()) * AngleAxisd(parameters[2], Vector3d::UnitY());
  Eigen::Vector3d T(parameters[3],parameters[4],parameters[5]);

  // publish path
  Quaterniond Q_yourwork;
  Q_yourwork = R.transpose();
  Eigen::Vector3d T_wc = -R.transpose()*T;
  publishPath(frame_time, T_wc, Q_yourwork, path, pub_path);

  /* For quantitative evaluation */
  double diff_psi = R_ref.eulerAngles(2, 0, 1)(0) - R.eulerAngles(2, 0, 1)(0);
  if (diff_psi > M_PI)
  {
    diff_psi -= 2 * M_PI;
  }
  else if (diff_psi < - M_PI)
  {
    diff_psi += 2 * M_PI;
  }

  double diff_phi = R_ref.eulerAngles(2, 0, 1)(1) - R.eulerAngles(2, 0, 1)(1);
  if (diff_phi > M_PI)
  {
    diff_phi -= 2 * M_PI;
  }
  else if (diff_phi < -M_PI)
  {
    diff_phi += 2 * M_PI;
  }

  double diff_theta = R_ref.eulerAngles(2, 0, 1)(2) - R.eulerAngles(2, 0, 1)(2);
  if (diff_theta > M_PI)
  {
    diff_theta -= 2 * M_PI;
  }
  else if (diff_theta < -M_PI)
  {
    diff_theta += 2 * M_PI;
  }

  count_frame = count_frame + 1;
  cum_error(0, 0) = cum_error(0, 0) + pow(diff_psi, 2);
  cum_error(1, 0) = cum_error(1, 0) + pow(diff_phi, 2);
  cum_error(2, 0) = cum_error(2, 0) + pow(diff_theta, 2);
  cum_error(3, 0) = cum_error(3, 0) + pow(t.at<double>(0, 0) - T(0),2);
  cum_error(4, 0) = cum_error(4, 0) + pow(t.at<double>(1, 0) - T(1),2);
  cum_error(5, 0) = cum_error(5, 0) + pow(t.at<double>(2, 0) - T(2),2);
  ROS_INFO("RMSE X, Y, Z, roll, pitch, yaw: \n %f, %f, %f, %f, %f, %f",
            sqrt(cum_error(3, 0) / count_frame), sqrt(cum_error(4, 0) / count_frame), sqrt(cum_error(5, 0) / count_frame),
            sqrt(cum_error(1, 0) / count_frame), sqrt(cum_error(2, 0) / count_frame), sqrt(cum_error(0, 0) / count_frame));

  cv::Mat R_res_cv(3, 3, CV_64FC1);
  cv::eigen2cv(R, R_res_cv);

  cv::Mat t_res_cv(3, 1, CV_64FC1);
  cv::eigen2cv(T, t_res_cv);

  calculateReprojectionError(pts_3, pts_2, r, t);
  std::cout << "ref error" << reproj_error << std::endl;

  calculateReprojectionError(pts_3,pts_2,R_res_cv,t_res_cv);
  std::cout << "res error" << reproj_error << std::endl;
}

// Callback function for processing the incoming images.
//
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  /* Detect markers and obtain 3D-2D data association */
  double t = clock();
  cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  MDetector.detect(bridge_ptr->image, Markers);
  float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
  ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

  vector<int> pts_id;
  vector<cv::Point3f> pts_3;
  vector<cv::Point2f> pts_2;
  for (unsigned int i = 0; i < Markers.size(); i++)
  {
    // Obtain the ID of the marker
    int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

    char str[100];
    sprintf(str, "%d", idx);
    cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(-1));
    for (unsigned int j = 0; j < 4; j++)
    {
      sprintf(str, "%d", j);
      cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,1));
    }
    // j here is index of four corners
    for (unsigned int j = 0; j < 4; j++)
    {
      pts_id.push_back(Markers[i].id * 4 + j);
      pts_3.push_back(getPositionFromIndex(idx, j));
      pts_2.push_back(Markers[i][j]);
    }
  }

  /* Call your pose estimation function */
  if (pts_id.size() > 5)
    process(pts_id, pts_3, pts_2, img_msg->header.stamp);

  /* Render the detection result on the raw image */
  cv::imshow("in", bridge_ptr->image);
  cv::waitKey(10);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tag_detector");
  ros::NodeHandle n("~");

  sub_img = n.subscribe("image_raw", 100, img_callback);
  pub_path_ref = n.advertise<nav_msgs::Path>("path_ref", 10);
  pub_path = n.advertise<nav_msgs::Path>("path_yourwork", 10);

  //init aruco detector
  string cam_cal, board_config;
  n.getParam("cam_cal_file", cam_cal);
  n.getParam("board_config_file", board_config);
  CamParam.readFromXMLFile(cam_cal);
  TheBoardConfig.readFromFile(board_config);

  //init intrinsic parameters
  cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
  param_reader["camera_matrix"] >> K;
  param_reader["distortion_coefficients"] >> D;

  //init window for visualization
  cv::namedWindow("in",1);
  ros::spin();
}