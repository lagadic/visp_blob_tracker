#include "node.h"

int main(int argc,char** argv){
  ros::init(argc, argv, "visp_blob_target_tracker");
  visp_blobs_tracker::Node().spin();
	return 0;
}
