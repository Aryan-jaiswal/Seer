//
// adapted from ros example and april tag examples - palash
//
#include <fstream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/rigidtransform.h"
#include <Eigen/Dense>
#include "popt_pp.h"
using std::ofstream;
using namespace cv;
using namespace std;
#include "seer/AprilTag.h" // rosmsg
#include "seer/AprilTagList.h" // rosmsg


static const std::string OPENCV_WINDOW = "Image window";

const double PI = 3.14159265358979323846;
const double TWOPI = 2.0*PI;
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> TransformType2;
/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
class AprilTagNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher odom_pub_;
  //ros::Publisher tag_list_pub;
  AprilTags::TagDetector* tag_detector;

  // allow configurations for these:
  AprilTags::TagCodes tag_codes;
  double camera_focal_length_x; // in pixels. late 2013 macbookpro retina = 700
  double camera_focal_length_y; // in pixels
  double tag_size; // tag side length of frame in meters
  bool  show_debug_image;
  cv::Mat image_grayR;
  cv::Mat image_grayL;
  //cv::Mat imgL;
  //cv::Mat imgR;

public:
  AprilTagNode() :
    it_(nh_),
    tag_codes(AprilTags::tagCodes36h11),
    tag_detector(NULL),
    camera_focal_length_y(414),
    camera_focal_length_x(414),
    tag_size(0.015), // 1 1/8in marker = 0.029m
    show_debug_image(false)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &AprilTagNode::imageCb, this);
    image_pub_ = it_.advertise("/seer_debug/output_video", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ground_truth", 1);
    //tag_list_pub = nh_.advertise<seer::AprilTagList>("/seers", 100);

    // Use a private node handle so that multiple instances of the node can
    // be run simultaneously while using different parameters.
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<double>("focal_length_px", camera_focal_length_x, 700.0);
    private_node_handle.param<double>("tag_size_cm", tag_size, 2.9);
    private_node_handle.param<bool>("show_debug_image", show_debug_image, false);

    camera_focal_length_y = camera_focal_length_x; // meh
    tag_size = tag_size / 100.0; // library takes input in meters


    cout << "got focal length " << camera_focal_length_x << endl;
    cout << "got tag size " << tag_size << endl;
    tag_detector = new AprilTags::TagDetector(tag_codes);
    if (show_debug_image) {
      cv::namedWindow(OPENCV_WINDOW);
    }

  }

  ~AprilTagNode()
  {
    if (show_debug_image) {
     cv::destroyWindow(OPENCV_WINDOW);
    }
  }
  TransformType2 CallBacknew()
  {
    vector<AprilTags::TagDetection> detectionsL =tag_detector->extractTags(image_grayL);
    vector<seer::AprilTag> tag_msgs;
    vector<AprilTags::TagDetection> detectionsR = tag_detector->extractTags(image_grayR);
  //  vector<seer::AprilTag> tag_msgs;
    if(detectionsR.size()>0 && detectionsL.size()>0){
      Eigen::Vector3d translationL ,translationR;
      Eigen::Matrix3d rotationL , rotationR;
      vector<Eigen::Vector3d>  P1s , P2s;
  }
  static void CallBackFunc(int event,int i,int j,int flags,void* userdata)
  {
    if(event == EVENT_LBUTTONDOWN)
    {
      nav_msgs::Odometry odom;
      for (size_t i = 0; i < detectionsL.size(); i++)
       {

      detectionsL[i].getRelativeTranslationRotation(tag_size,
                                               camera_focal_length_x,
                                               camera_focal_length_y,
                                               image_grayL.cols / 2,
                                               image_grayL.rows / 2,
                                               translationL,
                                               rotationL);

     odom.header.stamp = ros::Time::now();
     odom.header.frame_id = "cam";
     odom.pose.pose.position.x = translationL(1);
     odom.pose.pose.position.y = -translationL(2);
     odom.pose.pose.position.z = 2.89698162598 - translationL(0);
     cout << detectionsL[i].id << " : " << odom.pose.pose.position.x << ":" << odom.pose.pose.position.y << " :" << odom.pose.pose.position.z << endl;
     P1s[i] << odom.pose.pose.position.x ,  odom.pose.pose.position.y , odom.pose.pose.position.z ;
   }
   for(size_t i=0;i<detectionsR.size();i++){


     detectionsR[i].getRelativeTranslationRotation(tag_size,
                                              camera_focal_length_x,
                                              camera_focal_length_y,
                                              image_grayR.cols / 2,
                                              image_grayR.rows / 2,
                                              translationR,
                                              rotationR);
     //nav_msgs::Odometry odom;
     odom.header.stamp = ros::Time::now();
     odom.header.frame_id = "cam";
     odom.pose.pose.position.x = translationR(1);
     odom.pose.pose.position.y = -translationR(2);
     odom.pose.pose.position.z = 2.89698162598 - translationR(0);
     cout << detectionsR[i].id << " : " << odom.pose.pose.position.x << ":" << odom.pose.pose.position.y << " :" << odom.pose.pose.position.z << endl;
     P2s[i] << odom.pose.pose.position.x ,  odom.pose.pose.position.y , odom.pose.pose.position.z ;
   }
 }

  if(event==EVENT_RBUTTONDOWN)
  {
    TransformType RT= computeRigidTransform( P1s, P2s);
    std::cout << RT.first << endl;
	  std::cout << (RT.second)[0] << "  " << (RT.second)[1] << "  " << (RT.second)[2] << endl;
	  cout << endl;
  }

  }
}
void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
 {
   cv::Mat tmpL;
   //cv_bridge::CvImagePtr tmpR;
   cv::Mat tmpR;
   tmpL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
   tmpR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  if (tmpL.empty() || tmpR.empty())
   return;
  calibrate(tmpL ,tmpR);
 }
//void calibrate(cv_bridge::CvImagePtr cvL_ptr,cv_bridge::CvImagePtr cvR_ptr)
void calibrate(cv::Mat cvL_ptr,cv::Mat cvR_ptr)
{
  //cv::Mat image_grayL;
  cv::cvtColor(cvL_ptr, image_grayL, CV_BGR2GRAY);


  cv::cvtColor(cvR_ptr, image_grayR, CV_BGR2GRAY);
  //int  i=0;
    namedWindow("Left_Input",WINDOW_AUTOSIZE);
    setMouseCallback("Left_Input", CallBackFunc,NULL);
    imshow("Left_Input",image_grayL);
    waitKey(0);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //processCvImage(cv_ptr);
  }



 };

/*************************************************    MAIN    ************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seer_node");
   ros::NodeHandle nh;
   AprilTagNode atn;
  int mode;
  static struct poptOption options[] =
  {
    { "mode",'m',POPT_ARG_INT,&mode,0,"Set m=0 for calibration, m=1 for testing","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh,"/usb_cam/image_raw" , 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/usb_cam2/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
if(mode==0)
 {
  sync.registerCallback(boost::bind(&AprilTagNode::imgCallback, atn, _1, _2));
 }
/*elseif(mode==1)
 {
    PointsType P2ss , error;
    P2ss=R*P1s+t;
    error=P2s-P2ss;
    cout<< error;
  }*/

  ros::spin();
  return 0;
}
