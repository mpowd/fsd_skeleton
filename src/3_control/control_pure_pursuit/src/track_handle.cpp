#include <ros/ros.h>
#include "track_handle.hpp"
typedef track_handle::TrackHandle TrackHandle;

namespace track_handle
{

TrackHandle::TrackHandle(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  loadParameters();
  initializeSubscribers();
  initializePublishers();
}
void TrackHandle::loadParameters()
{
  ROS_INFO("loading track handle parameters");
  if (!nodeHandle_.param<std::string>("map_publish_topic_name", map_publish_topic_name_, "/estimation/slam/map"))
  {
    ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << map_publish_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("testing_only_track_subscribe_topic_name",
                                      testing_only_track_subscribe_topic_name_, "/fsds/testing_only/track"))
  {
    ROS_WARN_STREAM("Did not load testing_only_track_subscribe_topic_name. Standard value is: "
                    << testing_only_track_subscribe_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("marker_publish_topic_name", marker_publish_topic_name_,
                                      "/pure_pursuit/nearest_cones_marker"))
  {
    ROS_WARN_STREAM("Did not load marker_publish_topic_name. Standard value is: " << marker_publish_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1))
  {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
  if (!nodeHandle_.param("max_cone_distance", max_cone_distance_, 1))
  {
    ROS_WARN_STREAM("Did not load max_cone_distance. Standard value is: " << max_cone_distance_);
  }
}

void TrackHandle::initializeSubscribers()
{
  ROS_INFO("subscribe to topics");
  testingOnlyTrackSubscriber_ =
      nodeHandle_.subscribe(testing_only_track_subscribe_topic_name_, 1, &TrackHandle::testingOnlyTrackCb, this);
}

void TrackHandle::initializePublishers()
{
  ROS_INFO("publish to topics");
  mapPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Map>(map_publish_topic_name_, 1);
  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(marker_publish_topic_name_, 1);
}
int TrackHandle::getNodeRate() const
{
  return node_rate_;
}

void TrackHandle::testingOnlyTrackCb(const fs_msgs::Track& msg)
{
  fsd_common_msgs::Map map;
  for (const fs_msgs::Cone cone : msg.track)
  {
    fsd_common_msgs::Cone newCone;
    newCone.position = cone.location;
    switch (cone.color)
    {
      case 0:  // BLUE
        newCone.color.data = "BLUE";
        map.cone_blue.push_back(newCone);
        break;
      case 1:  // YELLOW
        newCone.color.data = "YELLOW";
        map.cone_yellow.push_back(newCone);
        break;
      case 2:  // ORANGE BIG
        break;
      case 3:  // ORANGE SMALL
        newCone.color.data = "ORANGE";
        map.cone_orange.push_back(newCone);
        break;
      case 4:  // UNKNOWN
        ROS_INFO("UNKNOWN CONE COLOR");
        break;
      default:
        ROS_INFO("INVALID CONE COLOR");
        break;
    }
  }
  initial_map = map;
}

void TrackHandle::publishNearestCones()
{
  if (initial_map.cone_blue.size() > 0 && initial_map.cone_yellow.size() > 0)
  {
    fsd_common_msgs::Map nearest_cones = getNearestCones();
    mapPublisher_.publish(nearest_cones);
    publishMarkers(nearest_cones);
    nearest_cones.cone_blue.clear();
    nearest_cones.cone_yellow.clear();
  }
}

fsd_common_msgs::Map TrackHandle::getNearestCones()
{
  fsd_common_msgs::Map map;

  for (auto c : initial_map.cone_blue)
  {
    try
    {
      geometry_msgs::PointStamped cone_odom, cone_fscar;
      cone_odom.point.x = c.position.x;
      cone_odom.point.y = c.position.y;
      cone_odom.point.z = c.position.z;
      cone_odom.header.frame_id = "odom";
      tfListener.transformPoint("fsds/FSCar", cone_odom, cone_fscar);

      const double distance = std::hypot(cone_fscar.point.x, cone_fscar.point.y);

      if (distance < max_cone_distance_ && cone_fscar.point.x > 0)
      {
        map.cone_blue.push_back(c);
      }
    }
    catch (tf::TransformException& ex)
    {
      // ROS_ERROR("%s", ex.what());
    }
  }
  for (auto c : initial_map.cone_yellow)
  {
    try
    {
      geometry_msgs::PointStamped cone_odom, cone_fscar;
      cone_odom.point.x = c.position.x;
      cone_odom.point.y = c.position.y;
      cone_odom.point.z = c.position.z;
      cone_odom.header.frame_id = "odom";
      tfListener.transformPoint("fsds/FSCar", cone_odom, cone_fscar);

      const double distance = std::hypot(cone_fscar.point.x, cone_fscar.point.y);

      if (distance < max_cone_distance_ && cone_fscar.point.x > 0)
      {
        map.cone_yellow.push_back(c);
      }
    }
    catch (tf::TransformException& ex)
    {
      // ROS_ERROR("%s", ex.what());
    }
  }
  publishMarkers(map);
  return map;
}

void TrackHandle::publishMarkers(fsd_common_msgs::Map nearest_cones)
{
  visualization_msgs::MarkerArray markers;
  int id = 2;
  for (auto c : nearest_cones.cone_blue)
  {
    markers.markers.push_back(getConeMarker(++id, c, false));
    markers.markers.push_back(getConeTextMarker(++id, c));
  }
  for (auto c : nearest_cones.cone_yellow)
  {
    markers.markers.push_back(getConeMarker(++id, c, true));
    markers.markers.push_back(getConeTextMarker(++id, c));
  }
  markerPublisher_.publish(markers);
  markers.markers.clear();
}

visualization_msgs::Marker TrackHandle::getConeMarker(int id, fsd_common_msgs::Cone cone, bool is_yellow)
{
  visualization_msgs::Marker marker;
  marker.color.r = is_yellow ? 1.0 : 0.0;
  marker.color.g = is_yellow ? 1.0 : 0.0;
  marker.color.b = is_yellow ? 0.0 : 1.0;
  marker.color.a = 1.0;
  marker.pose.position.x = cone.position.x;
  marker.pose.position.y = cone.position.y;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.5;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "odom";
  return marker;
}

visualization_msgs::Marker TrackHandle::getConeTextMarker(int id, fsd_common_msgs::Cone cone)
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "odom";
  text_marker.header.stamp = ros::Time::now();
  text_marker.id = id;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;

  text_marker.pose.position.x = cone.position.x;
  text_marker.pose.position.y = cone.position.y;
  text_marker.pose.position.z = 1.0;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;

  text_marker.text = "(" + std::to_string(cone.position.x) + "/" + std::to_string(cone.position.y) + ")";

  text_marker.scale.x = 0.2;
  text_marker.scale.y = 0.2;
  text_marker.scale.z = 0.3;

  text_marker.color.r = 0.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 1.0;
  return text_marker;
}
}  // namespace track_handle

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trackHandle");
  ros::NodeHandle nodeHandle("~");
  TrackHandle trackHandle(nodeHandle);
  ros::Rate loop_rate(trackHandle.getNodeRate());
  while (ros::ok())
  {
    trackHandle.publishNearestCones();
    ros::spinOnce();    // Keeps node alive basically
    loop_rate.sleep();  // Sleep for loop_rate
  }
  return 0;
}
