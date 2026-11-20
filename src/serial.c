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
 * @file serial.c
 * @brief Configure USHER's non-blocking flight-controller UART.
 * @details This module owns only the receive-side descriptor setup. Transmit
 * buffering and concurrent writers are handled by serial_tx.c. Raw 8-bit mode is
 * necessary because USHER frames are binary and can contain bytes that terminal
 * line discipline would otherwise translate or consume.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

/****************************************************************************/
/* Open serial device for requested protocoll */
/****************************************************************************/

/** Shared UART descriptor used by legacy receive/transmit integration code. */
int fd = -1;

int serial_init(const char *port_name)
{
  struct termios orig_termios, cur_termios;
  int failure = -1;

  const speed_t baud_rate = B115200;

  if (!port_name || !port_name[0]) { errno = EINVAL; return -1; }
  fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

  if (fd == -1) {
    fprintf(stderr, "USHER:\tfailed to open serial device %s: %s\n", port_name, strerror(errno));
    return -1;
  }

  if (tcgetattr(fd, &orig_termios)) {
    failure = -2;
    goto failed;
  }
  cur_termios = orig_termios;

  /* input modes - turn off input processing */
  cur_termios.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                           | ICRNL | IXON | IXANY | IXOFF | IMAXBEL);
  /* pas IGNCR sinon il vire les 0x0D */
  cur_termios.c_iflag |= BRKINT;

  /* output_flags - turn off output processing */
  cur_termios.c_oflag  &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);

  /* control modes */
  cur_termios.c_cflag &= ~(CSIZE | CSTOPB | CREAD | PARENB | PARODD | HUPCL | CLOCAL);
  cur_termios.c_cflag |= CREAD | CS8 | CLOCAL;

  /* local modes */
  cur_termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO);
  cur_termios.c_lflag |= NOFLSH;
  cur_termios.c_cc[VMIN] = 1;
  cur_termios.c_cc[VTIME] = 0;

  if (cfsetispeed(&cur_termios, baud_rate) || cfsetospeed(&cur_termios, baud_rate)) {
    failure = -3;
    goto failed;
  }

  if (tcsetattr(fd, TCSANOW, &cur_termios)) {
    failure = -4;
    goto failed;
  }

  return 0;
failed:
  {
    const int error = errno;
    close(fd);
    fd = -1;
    fprintf(stderr, "USHER:\tfailed to configure serial device %s: %s\n", port_name, strerror(error));
    errno = error;
    return failure;
  }
}


