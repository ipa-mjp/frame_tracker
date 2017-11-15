
#include <ros/ros.h>
#include <frame_tracker/interactive_marker.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "interactive_marker_node");

	InteractiveMarker interactiv_markers;
	interactiv_markers.initialize();
	ros::spin();

}
