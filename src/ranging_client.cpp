#include "ranging_client.h"
#include <cstdint>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define DWT_TIME_UNITS ((double)15.65e-12) //!< = 15.65e-12 s
#define SPEED_OF_LIGHT ((double)299702547.0)
#define UUS_TO_DWT_TIME 64267

struct ranging_pkt_t {
  uint8_t packet_number;
  uint32_t RoundA;
  uint32_t DelayA;
  uint32_t RoundB;
  uint32_t DelayB;
  float power;
};

bool ranging_pkt_decode(std::vector<uint8_t> &data, struct ranging_pkt_t *pkt) {
  if (data.size() != 17)
    return false;

  pkt->packet_number = data[0];
  pkt->RoundB = (uint32_t)data[1] | (uint32_t)data[2] << 8 |
                (uint32_t)data[3] << 16 | (uint32_t)data[4] << 24;
  pkt->DelayB = (uint32_t)data[5] | (uint32_t)data[6] << 8 |
                (uint32_t)data[7] << 16 | (uint32_t)data[8] << 24;
  pkt->DelayA = (uint32_t)data[9] | (uint32_t)data[10] << 8 |
                (uint32_t)data[11] << 16 | (uint32_t)data[12] << 24;
  pkt->power = (uint32_t)data[13] | (uint32_t)data[14] << 8 |
               (uint32_t)data[15] << 16 | (uint32_t)data[16] << 24;

  return true;
}

std::vector<uint8_t> ranging_pkt_encode(struct ranging_pkt_t *pkt) {
  std::vector<uint8_t> data;
  data.push_back(pkt->packet_number);

  data.push_back((uint8_t)(pkt->RoundA & 0xFF));
  data.push_back((uint8_t)((pkt->RoundA >> 8) & 0xFF));
  data.push_back((uint8_t)((pkt->RoundA >> 16) & 0xFF));
  data.push_back((uint8_t)((pkt->RoundA >> 24) & 0xFF));

  data.push_back((uint8_t)(pkt->DelayA & 0xFF));
  data.push_back((uint8_t)((pkt->DelayA >> 8) & 0xFF));
  data.push_back((uint8_t)((pkt->DelayA >> 16) & 0xFF));
  data.push_back((uint8_t)((pkt->DelayA >> 24) & 0xFF));

  data.push_back((uint8_t)(pkt->DelayB & 0xFF));
  data.push_back((uint8_t)((pkt->DelayB >> 8) & 0xFF));
  data.push_back((uint8_t)((pkt->DelayB >> 16) & 0xFF));
  data.push_back((uint8_t)((pkt->DelayB >> 24) & 0xFF));

  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);

  return data;
}

/**
 * @brief Calculate TOD from Ra, Da, Rb, Db
 *
 * @param data pointer to ranging packet
 * @return float calculated time of flight
 */
double ToF_DS(const struct ranging_pkt_t *data) {
  double Ra, Rb, Da, Db;

  Ra = (double)data->RoundA;
  Da = (double)data->DelayA;
  Rb = (double)data->RoundB;
  Db = (double)data->DelayB;

  double tof =
      (double)DWT_TIME_UNITS * (Ra * Rb - Da * Db) / (Ra + Da + Rb + Db);

  return tof;
}

namespace active_radar {

std::pair<std::vector<uint8_t>, uint64_t>
RangingClient::update(std::vector<uint8_t> &rx_vec, uint64_t rxTime) {
  struct ranging_pkt_t ranging_pkt = {0, 0, 0, 0, 0, 0};

  if (!ranging_pkt_decode(rx_vec, &ranging_pkt)) {
    return std::make_pair(std::vector<uint8_t>(0), 0);
  }


  // Ignore even packets if i'm the initiator
  // and ignore odd packets if i'm the responder
  if((this->initiator && ranging_pkt.packet_number % 2 == 0) || (!this->initiator && ranging_pkt.packet_number % 2 != 0))
  {
    return std::make_pair(std::vector<uint8_t>(0), 0);
  }

  ranging_pkt.RoundA = rxTime - this->txTime;

  if (ranging_pkt.packet_number >= 3) {
    double tof = ToF_DS(&ranging_pkt);
    double distance = tof * SPEED_OF_LIGHT;

    if (this->range_cb != nullptr && distance > 0.00 && distance < 100.00) {
      this->range_cb(distance, 0.01);
    }
  }

  if (ranging_pkt.packet_number >= 4)
  {
    return std::make_pair(std::vector<uint8_t>(0), 0);
  }

  ranging_pkt.packet_number++;

  this->txTime = rxTime + (12000 * UUS_TO_DWT_TIME);
  this->txTime &= 0xfffffffe00;
  ranging_pkt.DelayA = this->txTime - rxTime;

  std::vector<uint8_t> tx_vec = ranging_pkt_encode(&ranging_pkt);

  return std::make_pair(tx_vec, this->txTime >> 8);
}

} // namespace active_radar
