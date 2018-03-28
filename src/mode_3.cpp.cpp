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
#include <libconfig.h++>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include <Eigen/Dense>
#include "popt_pp.h"
#include "seer/AprilTag.h"      // rosmsg
#include "seer/AprilTagList.h"  // rosmsg
#include "rigidtransform.h"