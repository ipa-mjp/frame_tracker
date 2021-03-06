
// Containts implimentation client of service, interactive marker node.

#ifndef INTERACTIVE_FRAME_H
#define INTERACTIVE_FRAME_H

#include <string>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <frame_tracker/GetFrameTrackingInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>



class InteractiveMarker
{
public:
    InteractiveMarker() {;}
    ~InteractiveMarker() {;}

    bool initialize();

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    ros::Timer timer_;
    double update_rate_;

    std::string root_frame_;
    std::string chain_tip_link_;
    std::string tracking_frame_;    // the frame tracking the target
    std::string target_frame_;      // the frame to be tracked

    bool movable_trans_;
    bool movable_rot_;

    interactive_markers::InteractiveMarkerServer* ia_server_;
    visualization_msgs::InteractiveMarker int_marker_;
    visualization_msgs::InteractiveMarker int_marker_menu_;
    interactive_markers::MenuHandler menu_handler_;

    bool tracking_;
    bool lookat_;
    tf::StampedTransform target_pose_;
    boost::mutex mutex_;

    ros::ServiceClient start_tracking_client_;
    ros::ServiceClient stop_client_;

    void updateMarker(const std::string& frame);
    void sendTransform(const ros::TimerEvent& event);
    void startTracking(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void stopTracking(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void menuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};

#endif
