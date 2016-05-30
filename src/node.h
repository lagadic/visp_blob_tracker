#ifndef __VP_BLOBS_TRACKER_NODE_H__
#define __VP_BLOBS_TRACKER_NODE_H__

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>

#include "visp/vpConfig.h"
#include "visp3/core/vpImage.h"
#include "visp3/core/vpCameraParameters.h"

#include "vpBlobsTargetTracker.h"



namespace visp_blobs_tracker{
        class Node{
        private:
                boost::mutex lock_;
                ros::NodeHandle n_;
                unsigned long queue_size_;
                std::string tracker_config_path_;
                std::string model_description_;
                std::string color_file_path_;
                std::string model_name_;
                std::string camera_frame_name_;
                bool debug_display_;
                std::vector <vpPoint> points;
                double dist_point_;
                vpBlobsTargetTracker tracker_;
                bool status_tracker_;
                bool pub_des_pose_;
                vpHomogeneousMatrix cMh_d_;
                vpHomogeneousMatrix cMh_d_offset;
                bool first_time;
                bool move_des_pose;
                double d_t;
                double d_r;
                vpHomogeneousMatrix cMo;

                cv_bridge::CvImagePtr cv_ptr;
                cv::Mat cvI_;
                vpImage<unsigned char> I_; // Image used for debug display
                std_msgs::Header image_header_;
                bool got_image_;
                vpCameraParameters cam_;
                unsigned int lastHeaderSeq_;
                int freq_;

            void waitForImage();
            void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
            const void manageInputKey(const std::string s);

        public:
                Node();
                void spin();
        };
};
#endif
