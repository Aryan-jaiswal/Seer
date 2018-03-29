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
			private_node_handle.param<double>("tag_size_cm", tag_size, 13.25);
			private_node_handle.param<string>("Dictionary_String", dictionaryString, "ARUCO_MIP_36h12");

			camera_focal_length_y = camera_focal_length_x;  // hard_coded,calibrate your camera and edit it.:p

			TheCameraParameters.readFromXMLFile("/home/ash/catkin_ws/src/Seer/seer_cam.yml");         //write the path to your param file here
      tag_size = 13.25;
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
    		if(show_debug_image) {
    			cv::destroyWindow(OPENCV_WINDOW);
    		}
        //delete MDetector;
        //delete it_;
    	}
/*
	struct   TimerAvrg{
		std::vector<double> times;
		size_t curr=0,n;
		std::chrono::high_resolution_clock::time_point begin,end;
		TimerAvrg(int _n=30){
			n=_n;times.reserve(n);   
		}
		inline void start(){
			begin= std::chrono::high_resolution_clock::now();
		}
		inline void stop(){
			end= std::chrono::high_resolution_clock::now();
			double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
			if ( times.size()<n) 
				times.push_back(duration);
			else{ 
				times[curr]=duration;
				curr++;
				if (curr>=times.size()) 
					curr=0;
			}
		}
		double getAvrg(){
			double sum=0;
			for(auto t:times) 
				sum+=t;
			return sum/double(times.size());
		}
	};
  */
	


  	void poseCallback(float a , float b , float c )
  	{
  		static tf::TransformBroadcaster br;
  		tf::Transform transform;           //publish position and orientation
  		transform.setOrigin( tf::Vector3(a,b,c) );
      //////////////////////////////////////////////////////////////////////////////////////////////////
      /////hardcoding this as we will take data from IMU for mocap..Just for  visualising the positiom//
      /////////////////////////////////////////////////////////////////////////////////////////////////
  		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);   
      q.normalize();
      transform.setRotation(q);
   		geometry_msgs::PoseStamped msg;     //message type for mocap
  		mocap = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",1);   //queue_size=1 to avoid any latency
  		msg.header.stamp = ros::Time::now();
  		msg.header.seq = seq;               //must increment every time message is published
  		msg.pose.position.x = a;
  		msg.pose.position.y = b;
  		msg.pose.position.z = c;
  		msg.pose.orientation.w = quat_data.w;
  		msg.pose.orientation.x = quat_data.x;
  		msg.pose.orientation.y = quat_data.y;
  		msg.pose.orientation.z = quat_data.z;


  		//std::cout << quat_data.w << "," << quat_data.x << std::endl;       //for debugging
  		mocap.publish(msg);
  		seq += 1;
  		//////////////////////////////////////////////////////////
  		//uncomment below to increase fps in pixhwak(life hack!)//
  		//////////////////////////////////////////////////////////
  		
  		//mocap.publish(msg);
  		//seq += 1;

  		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ArucoTag"));   //publishing in world
    }

    	/*
    	*Returns the euclidean distance between two coordinates
    	*/

    float Eucl_distance(Eigen::Vector3d v1,Eigen::Vector3d v2) {
  		return sqrt(pow((v1[0]-v2[0]),2)+pow((v1[1]-v2[1]),2)+pow((v1[2]-v1[2]),2));
		}

		/*
		*Reading the imu message by subscribing to the topic and storing as 
		*Quaternion.
		*/
		 void quaternion_data(const sensor_msgs::Imu::ConstPtr& msg) {

		    quat_data.w = msg->orientation.w;
		    quat_data.x = msg->orientation.x;
		    quat_data.y = msg->orientation.y;
		    quat_data.z = msg->orientation.z;
		 }
		 void setParamsFromGlobalVariables(aruco::MarkerDetector &md){


			md.setDetectionMode((aruco::DetectionMode)iDetectMode,float(iMinMarkerSize)/1000.);
			md.getParameters().setCornerRefinementMethod( (aruco::CornerRefinementMethod) iCornerMode);

			md.getParameters().detectEnclosedMarkers(iEnclosed);
			md.getParameters().ThresHold=iThreshold;
			md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
		}

		 void calibrate(char key, Mat& image_grayL, Mat& image_grayR) {

		 	//float resize_factor = 1;
		 	iDictionaryIndex = (uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
		 	MDetector->setDictionary(dictionaryString,float(iCorrectionRate)/10.0 );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
     	iThreshold = MDetector->getParameters().ThresHold;
     	iCornerMode = MDetector->getParameters().cornerRefinementM;

     	setParamsFromGlobalVariables(*MDetector);

    	TheMarkersL = MDetector->detect(image_grayL, TheCameraParameters, tag_size);
    	TheMarkersR = MDetector->detect(image_grayR, TheCameraParameters, tag_size);

      image_grayR.copyTo(TheInputImageCopyR);
      image_grayL.copyTo(TheInputImageCopyL);

    // if (iShowAllCandidates){
    //     auto candidates=MDetector->getCandidates();
    //     for(auto cand:candidates) {
    //         aruco::Marker(cand,-1).draw(TheInputImageCopyR, Scalar(255, 0, 255));
    //         aruco::Marker(cand,-1).draw(TheInputImageCopyL, Scalar(255, 0, 255));
    //       }
    // }

    // for (unsigned int i = 0; i < TheMarkersR.size(); i++)
    // {
    //     cout << TheMarkersR[i] << endl;
    //     TheMarkersR[i].draw(TheInputImageCopyR, Scalar(0, 0, 255),2,true);
    // }
    //    for (unsigned int i = 0; i < TheMarkersL.size(); i++)
    // {
    //     cout << TheMarkersR[i] << endl;
    //     TheMarkersL[i].draw(TheInputImageCopyL, Scalar(0, 0, 255),2,true);
    // }


    // draw a 3d cube in each marker if there is 3d info
      for (unsigned int i = 0; i < TheMarkersR.size(); i++)
      {
          aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
          aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
      }
     
      
      for (unsigned int i = 0; i < TheMarkersL.size(); i++)
      {
          aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
          aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
      }
      \
      cv::imshow("Right_Detected",TheInputImageCopyR);
      cv::imshow("Left_Detected",TheInputImageCopyL);
      // cout<<"Size: "<<TheMarkersR.size();
      // cout<<"SizeL: "<<TheMarkersL.size();
    	double positionL[3];   //stores the position from left camera 
    	double orientationL[4];  //stores the quaternion
    	double positionR[3];   //stores the position from right camera
    	double orientationR[4];  //stores the quaternion
    	if (TheMarkersR.size() > 0 && TheMarkersL.size() > 0) {

    		Eigen::Vector3d translationL, translationR;
  			Eigen::Matrix3d rotationL, rotationR ;

  			static const char *file = "/home/ash/catkin_ws/src/Seer/T_matrix.cfg";
  			Config cfg;
  			Setting &root = cfg.getRoot();

  			if(!root.exists("Matrix"))
  				root.add("Matrix",Setting::TypeGroup);

    			Setting &Matrix = root["Matrix"];

    			if(key != 27) {


          
    				TheMarkersL[0].OgreGetPoseParameters(positionL,orientationL);
    				/*
    				*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
    				*/
    				double* ptrL = &positionL[0];   //pointer to the first position
    				std::vector<double> v_positionL(ptrL , ptrL + 3);   //double array to vector of doubles
    				double* v_ptrL = &v_positionL[0];
    				Eigen::Map<Eigen::Vector3d> translationL(v_ptrL, 3);   //mapping from vevtor of doubles to eigen Vector3d
  					pos1.push_back(Eigen::Vector3d(translationL(0 ), -translationL(1), 3.517- translationL(2)));       //converting to camera coordinate axes and making origin as ground

  					TheMarkersR[0].OgreGetPoseParameters(positionR,orientationR);
      				/*
      				*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
      				*/
      				double* ptrR = &positionR[0];   //pointer to the first position
      				std::vector<double> v_positionR(ptrR , ptrR + 3);   //double array to vector of doubles
    					double* v_ptrR = &v_positionR[0];
    					Eigen::Map<Eigen::Vector3d> translationR(v_ptrR , 3);   //mapping from vevtor of doubles to eigen Vector3d
    					pos2.push_back(Eigen::Vector3d(translationR(0), -translationR(1), 3.517- translationR(2)));        //converting to camera coordinate axes and making origin as ground
    					
    					ROS_INFO("Grabbed a set of points");

				}
				else  {
					TransformType RT = computeRigidTransform(pos1, pos2);

					std::cout << RT.first << endl; 
          			std::cout << (RT.second)[0] << "  " << (RT.second)[1] << "  "<< (RT.second)[2] << endl;
          			cout<<endl; //just to look good
          			Setting &R1=Matrix.add("R1",Setting::TypeList);
          			Setting &R2=Matrix.add("R2",Setting::TypeList);
          			Setting &R3=Matrix.add("R3",Setting::TypeList);
          			Setting &T=Matrix.add("T",Setting::TypeList);
              		for(int i=0;i<3;i++){
				        R1.add(Setting::TypeFloat) = (RT.first)(0,i);
				        R2.add(Setting::TypeFloat) = (RT.first)(1,i);
				        R3.add(Setting::TypeFloat) = (RT.first)(2,i);
				        T.add(Setting::TypeFloat) = (RT.second)[i];
					}
					cfg.writeFile(file);
          cerr<<"New Configuration successfully written to: "<<file <<endl;

          			pos1.clear();
          			pos2.clear();
				}
			}
			return;
		}
		 void calib_error(char key, Mat& image_grayL, Mat& image_grayR) {

		 	//float resize_factor = 1;
		 	iDictionaryIndex = (uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
		 	MDetector->setDictionary(dictionaryString,float(iCorrectionRate)/10.0 );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
         	iThreshold = MDetector->getParameters().ThresHold;
         	iCornerMode = MDetector->getParameters().cornerRefinementM;

         	setParamsFromGlobalVariables(*MDetector);
          double positionL[3];   //stores the position from left camera 
          double orientationL[4];  //stores the quaternion
          double positionR[3];   //stores the position from right camera
          double orientationR[4];  //stores the quaternion
          

        	TheMarkersL = MDetector->detect(image_grayL, TheCameraParameters, tag_size);
        	TheMarkersR = MDetector->detect(image_grayR, TheCameraParameters, tag_size);

          image_grayR.copyTo(TheInputImageCopyR);
          image_grayL.copyTo(TheInputImageCopyL);


          // draw a 3d cube in each marker if there is 3d info
            for (unsigned int i = 0; i < TheMarkersR.size(); i++)
            {
                aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
                aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
            }
           
            
            for (unsigned int i = 0; i < TheMarkersL.size(); i++)
            {
                aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
                aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
            }
            \
            cv::imshow("Right_Detected",TheInputImageCopyR);
            cv::imshow("Left_Detected",TheInputImageCopyL);

        	if (TheMarkersR.size() > 0 && TheMarkersL.size() > 0) {

        		Eigen::Vector3d translationL, translationR;
        		Eigen::Matrix3d rotationL, rotationR;	

        		static const char *file ="/home/ash/catkin_ws/src/Seer/T_matrix.cfg";
        		Config cfg;
        		try {
          			cfg.readFile(file);
        		}
        		catch(const FileIOException &fioex) {
          			std::cerr<<"I/O error while reading file"<<endl;
          			return;
          		}

          		if(key != 27) {

          			TheMarkersL[0].OgreGetPoseParameters( positionL, orientationL);
        				/*
        				*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
        				*/
        				double* ptrL = &positionL[0];   //pointer to the first position
        				std::vector<double> v_positionL(ptrL , ptrL + 3);   //double array to vector of doubles
      					double* v_ptrL = &v_positionL[0];
      					Eigen::Map<Eigen::Vector3d> translationL(v_ptrL, 3);   //mapping from vevtor of doubles to eigen Vector3d
      					pos1.push_back(Eigen::Vector3d(translationL(0), -translationL(1), 3.517- translationL(2)));        //converting to camera coordinate axes and making origin as ground

  				    	TheMarkersR[0].OgreGetPoseParameters(positionR, orientationR);
        				/*
        				*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
      				*/
      				  double* ptrR = &positionR[0];   //pointer to the first position
        				std::vector<double> v_positionR(ptrR , ptrR + 3);   //double array to vector of doubles
      					double* v_ptrR = &v_positionR[0];
      					Eigen::Map<Eigen::Vector3d> translationR(v_ptrR , 3);   //mapping from vevtor of doubles to eigen Vector3d
      					pos2.push_back(Eigen::Vector3d(translationR(0), -translationR(1), 3.517- translationR(2)));        //converting to camera coordinate axes and making origin as ground
      					
					       ROS_INFO("Grabbed a set of points");

          		}
          		else {
          			vector<float> r1,r2,r3,t;
            		float error;
            
            		Eigen::Matrix3d R;
            		Eigen::Vector3d T;

            		const Setting &R1_set = cfg.lookup("Matrix.R1");
           			const Setting &R2_set = cfg.lookup("Matrix.R2");
           			const Setting &R3_set = cfg.lookup("Matrix.R3");
           			const Setting &T_set = cfg.lookup("Matrix.T");
           			for(int n = 0 ; n < R1_set.getLength(); n++) {

             			r1.push_back(R1_set[n]);
			            r2.push_back(R2_set[n]);
			            r3.push_back(R3_set[n]);
			            t.push_back(T_set[n]);
			        }

			        for(int i = 0; i < 3; i++) {

              			T[i] = t[i];

              			for(int j = 0; j < 3; j++){

                			if(i == 0) R(i,j) = r1[j];
                			if(i == 1) R(i,j) = r2[j];
                			if(i == 2) R(i,j) = r3[j];
                		}
                	}
                	Eigen::Vector3d V;
              		for(size_t i = 0; i < pos1.size(); i++) {
                			V = R*pos1[i]+T;
              				pos2_calc.push_back(V);
          			}
          			for(size_t i = 0; i < pos2.size(); i++){
                		error=Eucl_distance(pos2_calc[i],pos2[i]);
                		cerr<<"Error for "<< i+1 << "Point is:"<<error<<endl;
                	}

                	cout<<endl;
                }
            }
            return;

		 }
/*
*Collects data from the image and publish it in mocap for pixhawk
*/
		 void pixhawk( Mat& image_grayL, Mat& image_grayR ) {          

		 	//float resize_factor = 1;
		 	iDictionaryIndex = (uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
		 	MDetector->setDictionary(dictionaryString,float(iCorrectionRate)/10.0 );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
         	iThreshold = MDetector->getParameters().ThresHold;
         	iCornerMode = MDetector->getParameters().cornerRefinementM;

         	setParamsFromGlobalVariables(*MDetector);

        	TheMarkersL = MDetector->detect(image_grayL, TheCameraParameters, tag_size);
        	TheMarkersR = MDetector->detect(image_grayR, TheCameraParameters, tag_size);

          image_grayR.copyTo(TheInputImageCopyR);
          image_grayL.copyTo(TheInputImageCopyL);



          // draw a 3d cube in each marker if there is 3d info
          for (unsigned int i = 0; i < TheMarkersR.size(); i++)
          {
              aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
              aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyR, TheMarkersR[i], TheCameraParameters);
          }
         
          
          for (unsigned int i = 0; i < TheMarkersL.size(); i++)
          {
              aruco::CvDrawingUtils::draw3dCube(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
              aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopyL, TheMarkersL[i], TheCameraParameters);
          }
          cv::imshow("Right_Detected",TheInputImageCopyR);
          cv::imshow("Left_Detected",TheInputImageCopyL);

          double positionL[3];   //stores the position from left camera 
          double orientationL[4];  //stores the quaternion
          double positionR[3];   //stores the position from right camera
          double orientationR[4];  //stores the quaternion
          

        	Eigen::Vector3d translationL, translationR;
        	Eigen::Matrix3d rotationL, rotationR;

        	if (TheMarkersR.size() > 0 && TheMarkersL.size() == 0) {


        		TheMarkersR[0].OgreGetPoseParameters( positionR, orientationR);
    				/*
    				*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
    				*/
    				double* ptrR = &positionR[0];   //pointer to the first position
    				std::vector<double> v_positionR(ptrR , ptrR + 3);   //double array to vector of doubles
  					double* v_ptrR = &v_positionR[0];
  					Eigen::Map<Eigen::Vector3d> translationR(v_ptrR , 3);   //mapping from vevtor of doubles to eigen Vector3d
  					pos2.push_back(Eigen::Vector3d(translationR(0), -translationR(1), 3.517- translationR(2)));        //converting to camera coordinate axes and making origin as ground

  					poseCallback( positionR[0], -positionR[1],3.517 - positionR[2] );
            pos2.clear();

				  }
  			   else if (TheMarkersR.size() == 0 && TheMarkersL.size() > 0 ) {

  				    static const char *file ="//home/ash/catkin_ws/src/Seer/T_matrix.cfg";
  		        Config cfg;
  		        try
  		        {
  		          cfg.readFile(file);
  		        }
  		        catch(const FileIOException &fioex)
  		        {
  		          std::cerr<<"I/O error while reading file mode 2"<<endl;
  		          return;
		        }

		        TheMarkersL[0].OgreGetPoseParameters( positionL, orientationL);
      			/*
      			*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
      			*/
      			double* ptrL = &positionL[0];   //pointer to the first position
      			std::vector<double> v_positionL(ptrL , ptrL + 3);   //double array to vector of doubles
    				double* v_ptrL = &v_positionL[0];
    				Eigen::Map<Eigen::Vector3d> translationL(v_ptrL, 3);   //mapping from vevtor of doubles to eigen Vector3d
    				pos1.push_back(Eigen::Vector3d(translationL(0), -translationL(1), 3.517- translationL(2)));        //converting to camera coordinate axes and making origin as ground

				    vector<float> r1,r2,r3,t;
	            //float error;
	            Eigen::Matrix3d R;
	            Eigen::Vector3d T;

	            const Setting &R1_set = cfg.lookup("Matrix.R1");
	            const Setting &R2_set = cfg.lookup("Matrix.R2");
	            const Setting &R3_set = cfg.lookup("Matrix.R3");
	            const Setting &T_set = cfg.lookup("Matrix.T");
	            for(int n=0 ; n < R1_set.getLength(); n++)
	            {
	              r1.push_back(R1_set[n]);
	              r2.push_back(R2_set[n]);
	              r3.push_back(R3_set[n]);
	              t.push_back(T_set[n]);

	              for(int i = 0; i < 3; i++) 
	              {
	              	T[i] = t[i];
	                for(int j = 0; j < 3; j++) 
	                {
	                	if(i == 0) R(i,j) = r1[j];
	                	if(i == 1) R(i,j) = r2[j];
	                	if(i == 2) R(i,j) = r3[j];
	               }
	             }
	         }
	         Eigen::Vector3d V;
             for( size_t i = 0; i < pos1.size(); i++)
              {
                V = R*pos1[i]+T;
              }

              pos2_calc.push_back(V);

              poseCallback(pos2_calc[cnt][0] ,pos2_calc[cnt][1] ,pos2_calc[cnt][2] );

              cnt++;
              pos1.clear();
              //pos2_calc.clear();
          }
          else if(TheMarkersR.size() > 0 && TheMarkersL.size() > 0) {

          		TheMarkersL[0].OgreGetPoseParameters( positionL, orientationL);
      			/*
      			*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
      			*/
      			double* ptrL = &positionL[0];   //pointer to the first position
      			std::vector<double> v_positionL(ptrL , ptrL + 3);   //double array to vector of doubles
    				double* v_ptrL = &v_positionL[0];
    				Eigen::Map<Eigen::Vector3d> translationL(v_ptrL, 3);   //mapping from vevtor of doubles to eigen Vector3d
    				pos1.push_back(Eigen::Vector3d(translationL(0), -translationL(1), 3.517- translationL(2)));        //converting to camera coordinate axes and making origin as ground
    				TheMarkersR[0].OgreGetPoseParameters( positionR, orientationR);
          			/*
      			*Converting from double array to Eigen3d and pushing as vector of eigen_vectors
      			*/
      			double* ptrR = &positionR[0];   //pointer to the first position
      			std::vector<double> v_positionR(ptrR , ptrR + 3);   //double array to vector of doubles
    				double* v_ptrR = &v_positionR[0];
    				Eigen::Map<Eigen::Vector3d> translationR(v_ptrR , 3);   //mapping from vevtor of doubles to eigen Vector3d
    				pos2.push_back(Eigen::Vector3d(translationR(0), -translationR(1), 3.517- translationR(2)));        //converting to camera coordinate axes and making origin as ground

    				poseCallback( positionR[0], -positionR[1], 3.517 - positionR[2] );
            pos1.clear();
            pos2.clear();
          }
          return;
      }

      void imgCallback(const sensor_msgs::ImageConstPtr& msg_left,
                   const sensor_msgs::ImageConstPtr& msg_right) {

	    cv::Mat tmpL, tmpR, image_grayL, image_grayR;
	    
	    tmpL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
	    tmpR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
	    //cout<<"in callback"<<endl;
      dictionaryString = "ARUCO_MIP_36h12";
	    if (TheCameraParameters.isValid()){
            TheCameraParameters.resize(tmpL.size());
           // cout<<"Valid!!"<<endl;
          }
      else
        cout<<"parameters not valid"<<endl;
	    
	    if (tmpL.empty() || tmpR.empty()) {
        return;

      }
	    ///////////////////////////////////////////////////////////////////
		  //Mode to check the error between calculated and detected values///
		  ///////////////////////////////////////////////////////////////////

	    if (m_mode == 1) {
		    cv::cvtColor(tmpL, image_grayL, CV_BGR2GRAY);
		    cv::cvtColor(tmpR, image_grayR, CV_BGR2GRAY);

		    imshow("Left_Input_Calibration_Error", image_grayL);
		    imshow("Right_Input_Calibration_Error", image_grayR);

		    char key = (char)cv::waitKey(30);
		    if (key > 0) {

	        calib_error(key, image_grayL, image_grayR);
		    }
	    }
	    //////////////////////////////////////
	    //Mode to calibrate the two cameras///
	    /////////////////////////////////////
	    else if(m_mode == 0) {


	      //cv::cvtColor(tmpL, image_grayL, CV_BGR2GRAY);
		    //cv::cvtColor(tmpR, image_grayR, CV_BGR2GRAY);

		    imshow("Left_Input_Calibration", tmpL);
		    imshow("Right_Input_Calibration", tmpR);
        //cout<<"Calibration Mode Started"<<endl;
		    char key = (char)cv::waitKey(30);
		    if (key > 0) {
		      calibrate(key, tmpL, tmpR);

		    }
	    
	    }
	    //////////////////////////////////////
	    //Mode to transmit data to pixhawk///
	    /////////////////////////////////////
	    else if(m_mode == 2)
	    {
	     // cv::cvtColor(tmpL, image_grayL, CV_BGR2GRAY);
	     //cv::cvtColor(tmpR, image_grayR, CV_BGR2GRAY);

	      imshow("Left_Input", tmpL);
	      imshow("Right_Input", tmpR);
	      char key = (char)cv::waitKey(30);
	      //if(key > 0)
	      	pixhawk(tmpL, tmpR);
	      //}
	  }
    else 
      delete MDetector;
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
    Mat TheInputImage,TheInputImageGrey, TheInputImageCopyR, TheInputImageCopyL;

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
	message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/camera1/usb_cam/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/camera2/usb_cam2/image_raw", 1);
	/////////////////////////////////////////////////////
	//Synchronising time between the the input images///
	////////////////////////////////////////////////////
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);

	atn.m_mode = mode; //setting the mode
 // cout<<"Callback time"<<endl;
  
	sync.registerCallback(boost::bind(& ArucoTagNode::imgCallback, atn , _1, _2));    //call back function
  //cout<<"After callback"<<endl;
	ros::spin();
	return 0;                       
}


/*TODO
*read camera calibration file--done
*set global parameters--done
*check resize use--done
check video input--done subscribing
check global validity of markers l and R--yup
marker.rvec'--done
marker.tvec;--done
checkif marker[0]||marker[marker.id] gives correct value--done
*/