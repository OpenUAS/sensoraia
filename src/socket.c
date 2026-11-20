/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file socket.c
 * @brief Loopback UDP implementation for USHER's local development transport.
 * @details The fixed port permits the local bridge and test tools to find each other
 * without configuration. All I/O is non-blocking so local consumers cannot delay the
 * flight-controller UART event loop.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>
#include <string.h>
#include "socket.h"


#define SOCKET_PORT 32000

static int socket_fd = -1;
static struct sockaddr_in socket_server;

int socket_init(int is_server)
{
  if (socket_fd >= 0) {
    errno = EALREADY;
    return -1;
  }
  // Initialize socket
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("Socket: socket");
    return -1;
  }
  if (fcntl(socket_fd, F_SETFD, FD_CLOEXEC) != 0) {
    close(socket_fd);
    socket_fd = -1;
    return -1;
  }
  int status_flags = fcntl(socket_fd, F_GETFL);
  if (status_flags < 0 || fcntl(socket_fd, F_SETFL, status_flags | O_NONBLOCK) != 0) {
    close(socket_fd);
    socket_fd = -1;
    return -1;
  }

  memset(&socket_server, 0, sizeof(socket_server));
  socket_server.sin_family = AF_INET;
  socket_server.sin_port = htons(SOCKET_PORT);
  socket_server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  if (is_server) {
    if (bind(socket_fd, (struct sockaddr *)&socket_server, sizeof(socket_server)) != 0) {
      int bind_errno = errno;
      if (bind_errno == EADDRINUSE) {
        fprintf(stderr,
                "USHER:\tanother USHER instance or payload listener already owns "
                "127.0.0.1:%d\n"
                "USHER:\tstop the existing process before starting another instance\n"
                "USHER:\tfor the managed MORA service, use: "
                "sudo -n systemctl stop usher.service\n",
                SOCKET_PORT);
      } else {
        fprintf(stderr, "USHER:\tfailed to bind local payload port 127.0.0.1:%d: %s\n",
                SOCKET_PORT, strerror(bind_errno));
      }
      close(socket_fd);
      socket_fd = -1;
      errno = bind_errno;
      return -1;
    }
  }
  return 0;
}

void socket_deinit(void)
{
  if (socket_fd >= 0) close(socket_fd);
  socket_fd = -1;
}

int socket_get_fd(void)
{
  return socket_fd;
}

int socket_recv(char *buffer, int len)
{
  if (!buffer || len <= 0) { errno = EINVAL; return -1; }
  if (socket_fd < 0) { errno = ENOTCONN; return -1; }
  struct iovec vector = {.iov_base = buffer, .iov_len = (size_t)len};
  struct msghdr message = {
    .msg_name = &socket_server,
    .msg_namelen = sizeof(socket_server),
    .msg_iov = &vector,
    .msg_iovlen = 1
  };
  ssize_t received = recvmsg(socket_fd, &message, 0);
  if (received >= 0 && (message.msg_flags & MSG_TRUNC) != 0) {
    errno = EMSGSIZE;
    return -1;
  }
  return (int)received;
}

void socket_send(char *buffer, int len)
{
  if (len < 0 || (!buffer && len)) { errno = EINVAL; return; }
  if (socket_fd < 0) { errno = ENOTCONN; return; }
  socklen_t slen = sizeof(socket_server);
  sendto(socket_fd, buffer, len, 0, (struct sockaddr *)&socket_server, slen);
}
