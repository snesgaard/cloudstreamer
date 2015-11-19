#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <zmq.h>

#include "util.h"

static volatile int terminate = 0;
// Interrupt handling should probably be done more cleverly.
// This will do for now
void intHandler(int dummy) {
  terminate = 1;
}

int main(int argc, char ** argv) {
  int err = 0;
  info("Starting cloudsteamer");
  signal(SIGINT, intHandler);
  info("Initializing server");
  void * context = zmq_init(1);
  if (!context) {
    error("Failed to create context");
    return 1;
  }
  void * socket = zmq_socket(context, ZMQ_SUB);
  if (!socket) {
    error("Failed to create socket");
    return 1;
  }
  char host[100];
  sprintf(host, "ipc:///tmp/datastream");
  errguard(err, zmq_bind(socket, host));
  info("Created subscriber at host address %s", host);
  while (!terminate) {
    info("Waiting for message");
    zmq_msg_t msg;
    errguard(err, zmq_msg_init(&msg));
    int more;
    size_t more_size;
    int suberr = 0;
    do {
      errguard(suberr, zmq_recv(socket, &msg, 0));
      errguard(suberr, zmq_getsockopt (socket, ZMQ_RCVMORE, &more, &more_size));
    } while(more && !suberr);
    info("Got message %lu", zmq_msg_size(&msg));
    errguard(err, zmq_msg_close(&msg));
  }

  info("Releasing resources");
  errguard(err, zmq_close(socket));
  errguard(err, zmq_term(context));

  return 0;
}
