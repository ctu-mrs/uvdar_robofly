#ifndef __ACTIVE_RADAR_NODELET_H__
#define __ACTIVE_RADAR_NODELET_H__

#include <cstdint>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <unordered_map>
#include <mrs_lib/param_loader.h>
#include <boost/container/flat_map.hpp>

#include "ranging_client.h"

namespace active_radar {

class ActiveRadarNodelet : public nodelet::Nodelet {
protected:
  bool running;

  // Here's our node handle!
  ros::NodeHandle nh;

  // Init publisher and subscribers here
  ros::Publisher range_pub;

  std::string m_node_name;

  uint64_t last_broadcast_ts;

  // UWB related settings
  int uwb_mac_addr;
  int uwb_pan_id;
  bool requests;

  double correction = 0;

  int uwb_fd;

  std::thread recv_thread;

  boost::container::flat_map<uint16_t, class RangingClient> clients;

public:
  // Constructor and destructor
  ActiveRadarNodelet();
  ~ActiveRadarNodelet();

  // Nodelet initialization called by the Nodelet manager
  virtual void onInit();

  // Set up the UWB socket
  bool initUWB();

  // Set time-sensitive networking settings for the UWB
  bool setTSN();

  // Threads
  void recvThread();

  void rangeCB(uint16_t id, double range, double std_dev);

  void msgCB(const std_msgs::StringConstPtr &msg);
};
} // namespace active_radar

#endif // __ACTIVE_RADAR_NODELET_H__
