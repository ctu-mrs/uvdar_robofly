#ifndef __RANGING_CLIENT_H__
#define __RANGING_CLIENT_H__

#include <cstdint>
#include <stdint.h>
#include <vector>
#include <functional>

namespace active_radar {
#define RANGING_MSG_TYPE 0x10

class RangingClient
{
    private:
        int expected_pkt_num = 0;
        uint64_t txTime;

        std::function<void(double, double)> range_cb;

    public:
        bool initiator;

        RangingClient() : expected_pkt_num(-1) {}
        ~RangingClient() = default;

        RangingClient(std::function<void(double, double)> cb, bool initiator)  {
            this->expected_pkt_num = -1;
            this->range_cb = std::move(cb);

            this->initiator = initiator;
        }

        std::pair<std::vector<uint8_t>, uint64_t> update(std::vector<uint8_t>& rx_vec, uint64_t rxTime);

        void setTxTime(uint64_t txTime) { this->txTime = txTime; }

        void setRangeCallback(std::function<void(double, double)> cb) { range_cb = std::move(cb); }
};

}


#endif // __RANGING_CLIENT_H__
