#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "ros/ros.h"
#include "fs_msgs/Track.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

namespace track_handle {

class TrackHandle {

 public:
  TrackHandle(ros::NodeHandle &nodeHandle);
  int getNodeRate() const;
  void publishNearestCones();


 private:
  ros::NodeHandle nodeHandle_;
  std::string map_publish_topic_name_;
  std::string testing_only_track_subscribe_topic_name_;
  std::string marker_publish_topic_name_;
  int node_rate_;
  int max_cone_distance_;
  fsd_common_msgs::Map initial_map;
  tf::TransformListener tfListener;
  ros::Publisher mapPublisher_;
  ros::Publisher markerPublisher_;
  ros::Subscriber testingOnlyTrackSubscriber_;
  void testingOnlyTrackCb(const fs_msgs::Track &msg);
  void initializeSubscribers();
  void initializePublishers();
  void loadParameters();
  void publishMarkers(fsd_common_msgs::Map nearest_cones);
  fsd_common_msgs::Map getNearestCones();
  visualization_msgs::Marker getConeTextMarker(int id, fsd_common_msgs::Cone cone);
  visualization_msgs::Marker getConeMarker(int id, fsd_common_msgs::Cone cone, bool is_yellow);
};
}