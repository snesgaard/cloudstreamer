#include "network.h"

const static int chunksize = 2500;
const char * uploadreqest = "upload";

static int send_credit(void * socket, std::string dealid, int points) {
  int datastart = points;
  int datasize = chunksize;
  int err = 0;
  int flags = ZMQ_DONTWAIT | ZMQ_SNDMORE;
  zmq_send(socket, dealid.c_str(), dealid.size(), flags);
  zmq_send(socket, &datastart, sizeof(datastart), flags);
  zmq_send(socket, &datasize, sizeof(datasize), ZMQ_DONTWAIT);
  return err;
}

int fetch_clouds(
  void * socket, const StateMap * smap, const CloudMap * cmap, int * termflag
) {
  int err = 0;
  zmq_msg_t dealid;
  zmq_msg_t msg;
  errguard(err, zmq_msg_init(&dealid));
  errguard(err, zmq_msg_init(&msg));
  while(!err && !(*termflag)) {

  }
  errguard(err, zmq_msg_init(&dealid));
  errguard(err, zmq_msg_close(&msg));
  return err;
}
