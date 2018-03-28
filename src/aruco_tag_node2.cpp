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
#include <libconfig.h++>      //create config files
#include "aruco.h"
#include "cvdrawingutils.h"
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "popt_pp.h"        //to take command from terminal
#include "rigidtransform.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"


using namespace cv;
using namespace std;
using namespace libconfig;
using namespace Eigen;
using std::ofstream;

static const std::string OPENCV_WINDOW = "IMAGE WINDOW";
const double PI = 3.14159265358979323846;
const double TWOPI = 2.0 * PI;


geometry_msgs::Quaternion quat_data;   //publish message in mocap

inline double standardRad(double t){         //Normalize angle between [-pi,pi]
	if (t >= 0.) 
    	t = fmod(t + PI, TWOPI) - PI;
    else 
    	t = fmod(t - PI, -TWOPI) + PI;
  
  return t;
}
/*
*Convert Rotation matrix to Euler angles
*/
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,double& roll) {
  yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
  roll = standardRad(atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

class ArucoTagNode {
public:
    int iDetectMode,iMinMarkerSize,iCorrectionRate,iShowAllCandidates,iEnclosed,iThreshold,
    	iCornerMode,iDictionaryIndex,waittime,m_mode;
    string dictionaryString;

    ArucoTagNode() {
      MDetector = new aruco::MarkerDetector();
      it_ = new image_transport::ImageTransport(nh_);

      imu_data = nh_.subscribe("/mavros/imu/data", 1, &ArucoTagNode::quaternion_data, this);
  		image_pub_ = it_->advertise("/seer_debug/output_video", 1);
  		odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ground_truth", 1);

  		// Use a private node handle so that multiple instances of the node can
  		// be run simultaneously while using different parameters.
			ros::NodeHandle private_node_handle("~");
			private_node_handle.param<double>("focal_length_px", camera_focal_length_x,
			                                  570.0);
			private_node_handle.param<bool>("show_debug_image", show_debug_image,
			                                false);
			private_node_handle.param<double>("tag_size_cm", tag_size, 16);
			private_node_handle.param<string>("Dictionary_String", dictionaryString, "ARUCO");

			camera_focal_length_y = camera_focal_length_x;  // hard_coded,calibrate your camera and edit it.:p

			TheCameraParameters.readFromXMLFile("/home/aryan/catkin_ws/src/Seer/seer_cam.yml");         //write the path to your param file here
			tag_size = tag_size/100;   //to get real world coordinates in metres
			iCorrectionRate = 0;
			iMinMarkerSize = 0;
			seq = 0 ,cnt = 0;
			F<< 1,0,0,
				  0,1,0,
				  0,0,1;
			cout << "got focal length(fx) " << camera_focal_length_x << endl;
						cout << "got focal length(fy) " << camera_focal_length_y << endl;
			cout << "got tag size " << tag_size << endl;
			cout << "got Dictionary_String " << dictionaryString << endl;

    		 if (show_debug_image) {
    		 	cv::namedWindow(OPENCV_WINDOW);
    		 }
    		ROS_INFO("Done");
    	 }
    	~ArucoTagNode() {					//destructor
    	// 	if(show_debug_image) {
    	// 		cv::destroyWindow(OPENCV_WINDOW);
    	// 	}
     //    delete MDetector;
     //    delete it_;
    	 }
      void quaternion_data(const sensor_msgs::Imu::ConstPtr& msg) {

        quat_data.w = msg->orientation.w;
        quat_data.x = msg->orientation.x;
        quat_data.y = msg->orientation.y;
        quat_data.z = msg->orientation.z;
     }

      void imgCallback(const sensor_msgs::ImageConstPtr& msg_left,
                   const sensor_msgs::ImageConstPtr& msg_right) {
        std::cout << "Edddd\n";
      }
      private:
      ros::NodeHandle nh_;
      image_transport::ImageTransport* it_;
      image_transport::Subscriber image_sub_;
      image_transport::Publisher image_pub_;
      ros::Subscriber imu_data;
      ros::Publisher odom_pub_;
      ros::Publisher mocap;
      aruco::MarkerDetector* MDetector;
      //aruco::VideoCapture TheVideoCapturer;
      vector<aruco::Marker> TheMarkersL;
      vector<aruco::Marker> TheMarkersR;
      Mat TheInputImage,TheInputImageGrey, TheInputImageCopy;

      aruco::CameraParameters TheCameraParameters;   //stores the parameters ;)
      double camera_focal_length_x;  // in pixels. 
      double camera_focal_length_y;  // in pixels
      double tag_size;               // side of the square in meters
      bool show_debug_image;
      vector<Eigen::Vector3d> pos1, pos2 ,pos2_calc ,P3s,P4s;
      Eigen::Matrix3d F;
      int seq,cnt;
};
int main(int argc , char ** argv) {

  ros::init( argc, argv, "seer_node" );
  ros::NodeHandle nh;
  ArucoTagNode atn;
  int mode;
  ////////////////////////////////////////////////////////////////////////////
  ///Command line parser to use different modes using Popt(how I don't know)//
  ///////////////////////////////////////////////////////////////////////////
  static struct poptOption options[] = {
      {"mode", 'm', POPT_ARG_INT, &mode, 0,
       "Set m=0 for calibration, m=1 for testing and error calculation,m = 2 for transmission of data to pixhawk", "NUM"},
      POPT_AUTOHELP{NULL, 0, 0, NULL, 0, NULL, NULL}};
  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {}
  ///////////////////////////////////////////////////
  //Subscribing to usb_cam_node for input image data//
  ////////////////////////////////////////////////////
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/usb_cam/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/usb_cam2/image_raw", 1);
  /////////////////////////////////////////////////////
  //Synchronising time between the the input images///
  ////////////////////////////////////////////////////
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);

  atn.m_mode = mode; //setting the mode
  cout<<"Callback time"<<endl;
  
  sync.registerCallback(boost::bind(& ArucoTagNode::imgCallback, atn , _1, _2));    //call back function
  cout<<"After callback"<<endl;
  ros::spin();
  return 0;                       
}

