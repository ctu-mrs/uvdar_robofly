#ifndef SOCKET_UTILS_H
#define SOCKET_UTILS_H

#include <sys/socket.h>
#include <linux/net_tstamp.h>
#include <time.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IEEE802154_ADDR_LEN 8
#define MAX_PACKET_LEN 127
#define EXTENDED 0

enum {
  IEEE802154_ADDR_NONE = 0x0,
  IEEE802154_ADDR_SHORT = 0x2,
  IEEE802154_ADDR_LONG = 0x3,
};

struct ieee802154_addr_sa {
  int addr_type;
  uint16_t pan_id;
  union {
    uint8_t hwaddr[IEEE802154_ADDR_LEN];
    uint16_t short_addr;
  };
};

struct sockaddr_ieee802154 {
  sa_family_t family;
  struct ieee802154_addr_sa addr;
};

// Function declarations
ssize_t sendto_delayed(int __fd, const void *__buf, size_t __n, int __flags,
                       const struct sockaddr *__addr, socklen_t __addr_len,
                       uint64_t txtime);

ssize_t sendto_ts(int __fd, const void *__buf, size_t __n, int __flags,
                  const struct sockaddr *__addr, socklen_t __addr_len,
                  uint64_t *txtime);

ssize_t recv_ts(int __fd, void *__buf, size_t __n, int __flags,
                struct sockaddr *__addr, socklen_t __addr_len,
                uint64_t *rxtime);

#ifdef __cplusplus
}
#endif

#endif // SOCKET_UTILS_H
