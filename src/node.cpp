#include "node.h"
#include "names.h"


#include <opencv2/highgui/highgui.hpp>


#include "visp_tracker/MovingEdgeSites.h"
#include "visp_tracker/KltPoints.h"

//visp includes
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpTime.h>
#include <visp/vpImageConvert.h>


#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>

#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

namespace visp_blobs_tracker{
Node::Node() :
  n_("~"),
  queue_size_(1),
  tracker_config_path_(),
  model_description_(),
  color_file_path_(),
  model_name_(),
  camera_frame_name_(),
  debug_display_(false),
  points(4),
  tracker_(),
  status_tracker_(false),
  cv_ptr(),
  cvI_(),
  I_(),
  image_header_(),
  got_image_(false),
  cam_(),
  lastHeaderSeq_(0),
  cMh_d_(),
  cMh_d_offset(),
  first_time(false),
  move_des_pose(false),
  d_t(0.01),
  d_r(0.),
  cMo()
{
  //get the tracker configuration file
  //this file contains all of the tracker's parameters, they are not passed to ros directly.
  n_.param<std::string>("tracker_config_path", tracker_config_path_, "");
  n_.param<bool>("debug_display", debug_display_, false);
  n_.param<std::string>("color_file_path", color_file_path_, "");
  n_.param<std::string>("model_name", model_name_, "larm");
  n_.param<std::string>("camera_frame_name", camera_frame_name_, "camera");
  ROS_INFO("model full path=%s",color_file_path_.c_str());
  n_.param("distance_points", dist_point_, 0.025/2);
  n_.param("frequency", freq_, 30);
  ROS_INFO("Distance points=%f",dist_point_);
  n_.param<bool>("pub_desired_pose", pub_des_pose_, false);
  if (pub_des_pose_)
    ROS_INFO("Publisher desired pose enabled");


  //Set points coordinates
  points[2].setWorldCoordinates(-dist_point_,-dist_point_, 0) ;
  points[1].setWorldCoordinates(-dist_point_,dist_point_, 0) ;
  points[0].setWorldCoordinates(dist_point_,dist_point_, 0) ;
  points[3].setWorldCoordinates(dist_point_,-dist_point_,0) ;


}



void Node::waitForImage(){
  while ( ros::ok ()){
    if(got_image_) return;
    ros::spinOnce();
  }
}



const void Node::manageInputKey(const std::string s)
{
  if (s == "h")
  {
    tracker_.setManualBlobInit(true);
    tracker_.setForceDetection(true);
  }
  else if (s == "+")
  {
    unsigned int value = tracker_.getGrayLevelMaxBlob() +10;
    tracker_.setGrayLevelMaxBlob(value);
    std::cout << "Set to "<< value << "the value of  " << std::endl;

  }
  else if (s == "-")
  {
    unsigned int value = tracker_.getGrayLevelMaxBlob()-10;
    tracker_.setGrayLevelMaxBlob(value-10);
    std::cout << "Set to "<< value << " GrayLevelMaxBlob. " << std::endl;

  }
  else if (s == "r")
  {
    cMh_d_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

    d_t = 0.0;
    d_r = 0.1;
    std::cout << "Rotation mode. " << std::endl;
  }
  else if (s == "t")
  {
    cMh_d_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

    d_t = 0.01;
    d_r = 0.0;
    std::cout << "Translation mode. " << std::endl;
  }

  else if (s == "d")
  {
    move_des_pose = !move_des_pose;
  }

  if (move_des_pose)
  {
    if (s == "4") //-y
    {
      cMh_d_offset.buildFrom(0.0, -d_t, 0.0, 0.0, -d_r, 0.0) ;
    }

    else if (s == "6")  //+y
    {
      cMh_d_offset.buildFrom(0.0, d_t, 0.0, 0.0, d_r, 0.0) ;
    }

    else if (s == "8")  //+x
    {
      cMh_d_offset.buildFrom(d_t, 0.0, 0.0, d_r, 0.0, 0.0) ;
    }

    else if (s == "2") //-x
    {
      cMh_d_offset.buildFrom(-d_t, 0.0, 0.0, -d_r, 0.0, 0.0) ;
    }

    else if (s == "7")//-z
    {
      cMh_d_offset.buildFrom(0.0, 0.0, -d_t, 0.0, 0.0, -d_r) ;
    }
    else if (s == "9") //+z
    {
      cMh_d_offset.buildFrom(0.0, 0.0, d_t, 0.0, 0.0, d_r) ;
    }

    cMh_d_ = cMh_d_ * cMh_d_offset;
    cMh_d_offset.eye();
  }


}



//records last received image
void Node::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
  boost::mutex::scoped_lock(lock_);
  image_header_ = image->header;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // I_ = visp_bridge::toVispImageRGBa(*image); //make sure the image isn't worked on by locking a mutex
  cam_ = visp_bridge::toVispCameraParameters(*cam_info);

  got_image_ = true;
}

void Node::spin(){

  //create display
  vpDisplayX* d = NULL;
  //  if(debug_display_)
  d = new vpDisplayX();
  bool click_done = false;
  vpMouseButton::vpMouseButtonType button;


  //subscribe to ros topics and prepare a publisher that will publish the pose
  message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(n_, image_topic, queue_size_);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(n_, camera_info_topic, queue_size_);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
  image_info_sync.registerCallback(boost::bind(&Node::frameCallback,this, _1, _2));
  ros::Publisher object_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
  ros::Publisher object_des_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_des_position_topic, queue_size_);
  ros::Publisher status_publisher = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);

  //wait for an image to be ready
  waitForImage();
  {
    //when an image is ready tell the tracker to start searching for patterns
    boost::mutex::scoped_lock(lock_);
    //if(debug_display_) {
    vpImageConvert::convert(cv_ptr->image, I_);
    d->init(I_); //also init display
    vpDisplay::setTitle(I_, "Visp display");
    // }

  }

  tracker_.setName(model_name_);
  tracker_.setCameraParameters(cam_);
  tracker_.setPoints(points);
  if (model_name_ == "larm")
    tracker_.setLeftHandTarget(true);
  else if (model_name_ == "rarm")
    tracker_.setLeftHandTarget(false);
  // objects[i].setManualBlobInit(true);

  if(!tracker_.loadHSV(color_file_path_))
  {
    ROS_ERROR("Error opening the file %s", color_file_path_.c_str());
    return;
  }

  geometry_msgs::PoseStamped msg_pose;
  geometry_msgs::PoseStamped msg_des_pose;
  std_msgs:: Int8 status;
  ros::Rate rate(freq_);
  unsigned int cont = 0;




  while(ros::ok()){

    if (lastHeaderSeq_ != image_header_.seq)
    {

      double t = vpTime::measureTimeMs();
      boost::mutex::scoped_lock(lock_);
      vpImageConvert::convert(cv_ptr->image, I_);
      vpDisplay::display(I_);
      click_done = vpDisplay::getClick(I_, button, false);

      char key[10];
      bool ret = vpDisplay::getKeyboardEvent(I_, key, false);
      std::string s = key;

      if (ret)
        manageInputKey(s);

      bool status_tracker_ =  tracker_.track(cv_ptr->image,I_);

      if (status_tracker_) {
        cMo = tracker_.get_cMo();
        vpDisplay::displayFrame(I_, cMo, cam_, 0.05, vpColor::none, 2);
        //cMo.print();

        if (first_time)
        {
          cMh_d_ = cMo;
          first_time = false;
        }

        if (!move_des_pose)
            cMh_d_ = cMo;



        //std::cout << "Status:" << status_tracker_ << std::endl;


        // Publish pose
        ros::Time now = ros::Time::now();
        msg_pose.header.stamp = now;
        msg_pose.header.frame_id = camera_frame_name_;
        msg_pose.pose = visp_bridge::toGeometryMsgsPose(cMo); //convert
        object_pose_publisher.publish(msg_pose);


        if (pub_des_pose_)
        {
          vpDisplay::displayFrame(I_, cMh_d_, cam_, 0.09, vpColor::none, 1);
          // Publish desired pose
          msg_des_pose.header.stamp = now;
          msg_des_pose.header.frame_id = camera_frame_name_;
          msg_des_pose.pose = visp_bridge::toGeometryMsgsPose(cMh_d_); //convert
          object_des_pose_publisher.publish(msg_des_pose);
        }

        //Publish status
        status.data = 1;
        status_publisher.publish(status);
      }
      else
      {
        first_time = true;
        status.data = 0;
        status_publisher.publish(status);
        ROS_DEBUG_THROTTLE(2, "Any blob target is detected");
      }
      //  // Publish resulting pose.
      //  if (object_pose_publisher.getNumSubscribers	() > 0)
      //  {
      //    ps.header = image_header_;
      //    ps.header.frame_id = tracker_ref_frame;
      //    object_pose_publisher.publish(ps);
      //  }


      //  // Publish state machine status.
      //  if (status_publisher.getNumSubscribers	() > 0)
      //  {
      //    std_msgs:: Int8 status;
      //    //   status.data = (unsigned char)(*(t_->current_state()));
      //    status_publisher.publish(status);
      //  }


      vpDisplay::flush(I_);
      cont ++;

      lastHeaderSeq_ = image_header_.seq;
      //std::cout << "Tracking done in " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }

    ros::spinOnce();
    rate.sleep();

  } //end while

  if (d != NULL)
  {
    delete d;
  }

} // end spin



}
