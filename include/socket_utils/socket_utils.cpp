#include "socket_utils.h"
#include <string.h>
#include <stdio.h>
#include <sys/select.h>
#include <sys/uio.h>
#include <errno.h>

ssize_t sendto_delayed(int __fd, const void *__buf, size_t __n, int __flags,
                       const struct sockaddr *__addr, socklen_t __addr_len,
                       uint64_t txtime) {
  struct msghdr msg;
  struct iovec iov;
  char cmsg_buf[CMSG_SPACE(sizeof(u_int64_t))];

  memset(&msg, 0, sizeof(msg));
  memset(&iov, 0, sizeof(iov));
  memset(cmsg_buf, 0, sizeof(cmsg_buf));

  msg.msg_name = (void *)__addr;
  msg.msg_namelen = __addr_len;

  iov.iov_base = (void *)__buf;
  iov.iov_len = __n;

  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);

  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SO_TXTIME;
  cmsg->cmsg_len = CMSG_LEN(sizeof(uint64_t));

  memcpy(CMSG_DATA(cmsg), &txtime, sizeof(uint64_t));

  return sendmsg(__fd, &msg, __flags);
}

ssize_t sendto_ts(int __fd, const void *__buf, size_t __n, int __flags,
                  const struct sockaddr *__addr, socklen_t __addr_len,
                  uint64_t *txtime) {
  int ret = sendto(__fd, __buf, __n, __flags, __addr, __addr_len);

  if (ret < 0)
    return ret;

  struct msghdr msg;
  struct iovec iov;
  char cmsg_buf[256];
  char rx_buf[MAX_PACKET_LEN + 1];

  memset(&msg, 0, sizeof(msg));
  memset(&iov, 0, sizeof(iov));
  memset(cmsg_buf, 0, sizeof(cmsg_buf));

  msg.msg_name = (void *)__addr;
  msg.msg_namelen = __addr_len;

  iov.iov_base = rx_buf;
  iov.iov_len = sizeof(rx_buf);

  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  fd_set rfds;

  FD_ZERO(&rfds);
  FD_SET(__fd, &rfds);

  struct timeval tv;
  fd_set set;
  tv.tv_sec = 0;
  tv.tv_usec = 500 * 1000;

  ret = select(__fd + 1, &rfds, NULL, NULL, &tv);

  if (ret < 0) {
    printf("select error\n");
    return ret;
  }

  *txtime = -1;

  while((ret = recvmsg(__fd, &msg, MSG_ERRQUEUE)))
  {
    if (ret < 0 || txtime == NULL)
      return 0;

    struct cmsghdr *cmsg;

    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
        cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET &&
          cmsg->cmsg_type == SO_TIMESTAMPING_NEW) {
        struct timespec *timestamp = (struct timespec *)CMSG_DATA(cmsg);
        *txtime = (timestamp[2].tv_sec * 1000000000L) + timestamp[2].tv_nsec;
      }
    }
  }

  return ret;
}

ssize_t recv_ts(int __fd, void *__buf, size_t __n, int __flags,
                struct sockaddr *__addr, socklen_t __addr_len,
                uint64_t *rxtime) {
  struct msghdr msg;
  struct iovec iov;
  char cmsg_buf[CMSG_SPACE(3 * sizeof(struct timespec))];

  memset(&msg, 0, sizeof(msg));
  memset(&iov, 0, sizeof(iov));
  memset(cmsg_buf, 0, sizeof(cmsg_buf));

  iov.iov_base = __buf;
  iov.iov_len = __n;

  msg.msg_name = __addr;
  msg.msg_namelen = __addr_len;

  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  ssize_t ret = recvmsg(__fd, &msg, __flags | SO_TIMESTAMPING);

  if (ret < 0 || rxtime == NULL)
    return ret;

  *rxtime = -1;

  struct cmsghdr *cmsg;

  for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
       cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET &&
        cmsg->cmsg_type == SO_TIMESTAMPING_NEW) {
      struct timespec *timestamp = (struct timespec *)CMSG_DATA(cmsg);

      *rxtime = (timestamp[2].tv_sec * 1000000000L) + timestamp[2].tv_nsec;
    }
  }

  return ret;
}
