
#include <ros/ros.h>
#include <frame_tracker/frame_tracker.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "frame_tracker_node");

	FrameTracker frame_tracker_;
	frame_tracker_.initialization();

	ros::spin();
}
