#include <cstdint>
#include <cstdio>
#include <errno.h>
#include <mrs_lib/scope_timer.h>
#include <socket_utils/socket_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "active_radar.h"
#include "ranging_client.h"
#include <mrs_msgs/RangeWithCovarianceIdentified.h>
#include <sensor_msgs/Range.h>

ros::Rate r = ros::Rate(10);

namespace active_radar {

ActiveRadarNodelet::ActiveRadarNodelet() {
  this->running = false;
  this->uwb_fd = -1;
}

ActiveRadarNodelet::~ActiveRadarNodelet() {
  this->running = false;

  printf("Closing UWB socket\n");
  if (!(this->uwb_fd < 0))
    close(this->uwb_fd);

  this->recv_thread.join();
  this->clients.clear();
}

void ActiveRadarNodelet::onInit() {
  // Init nodehandle
  this->nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  this->m_node_name = "ActiveRadar";

  mrs_lib::ParamLoader pl(this->nh, this->m_node_name);

  this->requests = true;

  // Load parameters
  pl.loadParam("uwb_mac_addr", this->uwb_mac_addr);
  pl.loadParam("uwb_pan_id", this->uwb_pan_id);
  pl.loadParam("requests", this->requests);

  pl.loadParam("measurement_correction", correction, double(-0.30));

  this->range_pub = this->nh.advertise<mrs_msgs::RangeWithCovarianceIdentified>("range", 1);

  if (!this->initUWB()) {
    NODELET_ERROR("Failed to initialize UWB");
    return;
  }

  if (!this->setTSN()) {
    NODELET_ERROR("Failed to set TSN, check if your kernel supports it");
    return;
  }

  NODELET_INFO("UWB initiliazed with MAC: 0x%X, PAN: 0x%X", this->uwb_mac_addr,
               this->uwb_pan_id);

  this->recv_thread = std::thread(&ActiveRadarNodelet::recvThread, this);
  if (!this->recv_thread.joinable()) {
    NODELET_ERROR("Failed to start recv thread");
    return;
  }

  this->clients = boost::container::flat_map<uint16_t, class RangingClient>();
  this->running = true;

  uint8_t empty_buf[17 + 1] = {0};
  empty_buf[0] = RANGING_MSG_TYPE;

  struct sockaddr_ieee802154 dst;

  memset(&dst, 0, sizeof(dst));
  dst.family = AF_IEEE802154;
  dst.addr.pan_id = 0xabcd;

  dst.addr.addr_type = IEEE802154_ADDR_SHORT;
  dst.addr.short_addr = 0xffff;

  uint64_t tx_timestamp;

  while (ros::ok()) {
    ros::spinOnce();

    if (!this->requests) {
      r.sleep();
      continue;
    }

    sendto_ts(this->uwb_fd, empty_buf, sizeof(empty_buf), 0,
              (struct sockaddr *)&dst, sizeof(dst), &this->last_broadcast_ts);

    for (auto &client : this->clients) {
      if(client.second.initiator)
      {
        client.second.setTxTime(this->last_broadcast_ts);
      }
    }

    r.sleep();
  }
}

bool ActiveRadarNodelet::initUWB() {
  int rc;

  this->uwb_fd = socket(PF_IEEE802154, SOCK_DGRAM, 0);
  if (this->uwb_fd < 0) {
    NODELET_ERROR("Failed to open UWB socket");
    return false;
  }

  struct sockaddr_ieee802154 phy_addr;

  memset(&phy_addr, 0, sizeof(phy_addr));
  phy_addr.family = AF_IEEE802154;
  phy_addr.addr.pan_id = this->uwb_pan_id;

  phy_addr.addr.addr_type = IEEE802154_ADDR_SHORT;
  phy_addr.addr.short_addr = this->uwb_mac_addr;

  rc = bind(this->uwb_fd, (struct sockaddr *)&phy_addr, sizeof(phy_addr));
  if (rc < 0) {
    NODELET_ERROR("Failed to bind UWB socket");
    return false;
  }

  // Set timeout to 100ms for better sanity checking of the nodelet
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 100 * 1000;

  rc = setsockopt(this->uwb_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                  sizeof(timeout));

  if (rc < 0) {
    NODELET_ERROR("Failed to set timeout on UWB socket");
    return false;
  }

  return true;
}

bool ActiveRadarNodelet::setTSN() {
  int rc;
  int off = 0;
  int on = 1;

  const int timestamping_flags =
      SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RX_SOFTWARE |
      SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_TX_SOFTWARE |
      SOF_TIMESTAMPING_RAW_HARDWARE;

  rc = setsockopt(this->uwb_fd, SOL_SOCKET, SO_TIMESTAMPING_NEW,
                  &timestamping_flags, sizeof(timestamping_flags));
  if (rc) {
    NODELET_ERROR(
        "setsockopt SO_TIMESTAMPING is not supported by your Linux kernel");
    return false;
  }

  rc = setsockopt(this->uwb_fd, PF_IEEE802154, 0, &off, sizeof(off));
  if (rc) {
    NODELET_ERROR("Failed to enable txtime");
    return false;
  }

  rc = setsockopt(this->uwb_fd, SOL_SOCKET, SO_SELECT_ERR_QUEUE, &on,
                  sizeof(on));
  if (rc) {
    NODELET_ERROR("Failed to enable error queue");
    return false;
  }

  return true;
}

void ActiveRadarNodelet::rangeCB(uint16_t id, double range, double std_dev) {
  NODELET_INFO("Range from 0x%X: %.2f m +- %f", id, range, std_dev);

  mrs_msgs::RangeWithCovarianceIdentified msg;
  sensor_msgs::Range *range_msg = &msg.range;

  range_msg->header.stamp = ros::Time::now();
  range_msg->header.frame_id = "uwb";
  range_msg->radiation_type = 3;
  range_msg->field_of_view = 2*M_PI;
  range_msg->min_range = 0.0;
  range_msg->max_range = 100.0;
  range_msg->range = range + correction;

  msg.id = id;
  msg.variance = std_dev*std_dev;

  this->range_pub.publish(msg);

  return;
}

void ActiveRadarNodelet::recvThread() {
  int rc;
  uint8_t rx_buf[MAX_PACKET_LEN + 1];
  struct sockaddr_ieee802154 dst, src;
  memset(&src, 0, sizeof(src));
  src.family = AF_IEEE802154;
  src.addr.pan_id = this->uwb_pan_id;
  src.addr.addr_type = IEEE802154_ADDR_SHORT;

  uint64_t rx_ts;

  while (true) {
    rc = recv_ts(this->uwb_fd, rx_buf, sizeof(rx_buf), 0,
                 (struct sockaddr *)&src, (socklen_t)sizeof(src), &rx_ts);

    if (!this->running)
      return;

    if (rc < 0) {
      if (errno == EAGAIN) {
        continue;
      } else {
        NODELET_ERROR("Failed to receive packet");
        continue;
      }
      continue;
    }

    if (src.addr.addr_type != IEEE802154_ADDR_SHORT) {
      NODELET_INFO("Received message from non short address");
      continue;
    }

    uint8_t msg_type = rx_buf[0];

    std::vector<uint8_t> rx_vec(rx_buf + 1, rx_buf + rc);

    if (msg_type != RANGING_MSG_TYPE) {
      NODELET_INFO("Received non ranging message");
      continue;
    }

    uint16_t client_addr = src.addr.short_addr;    

    if (this->clients.find(client_addr) == this->clients.end()) {

      bool initiator = this->uwb_mac_addr > client_addr && this->requests;
      NODELET_INFO("New client detected: 0x%X | Switching to role of %s", client_addr, initiator ? "initiator" : "responder");

      // create lambda function to this->rangeCB and pass it to Ranging client

      auto rangeCB = [this, client_addr](double range, double std_dev) {
        this->rangeCB(client_addr, range, std_dev);
      };

      this->clients.emplace(client_addr, RangingClient(rangeCB, initiator));
      if(initiator)
      {
        this->clients[client_addr].setTxTime(this->last_broadcast_ts);
      }
    }

    std::pair<std::vector<uint8_t>, uint64_t> tx =
        this->clients[client_addr].update(rx_vec, rx_ts);

    // insert packet info to the beginning

    if (tx.first.size() > 0) {
      tx.first.insert(tx.first.begin(), RANGING_MSG_TYPE);

      rc = sendto_delayed(this->uwb_fd, tx.first.data(), tx.first.size(), 0,
                          (struct sockaddr *)&src, sizeof(src), tx.second);
      if (rc < 0) {
        NODELET_ERROR("Failed to send packet");
      }
    }
  }
}

void ActiveRadarNodelet::msgCB(const std_msgs::String::ConstPtr &msg) {
  NODELET_INFO_STREAM(msg->data);
}
} // namespace active_radar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_radar::ActiveRadarNodelet, nodelet::Nodelet)
