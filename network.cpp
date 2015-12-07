#include "network.h"

const static int timeout = 3000; // Defined in ms
const static int cunksize = 2500;
const static int maxdatasize = chunksize * 3 * sizeof(float);
const static int idsize = 100;
std::string id_msg(idsize, 0);
std::string data_msg(maxdatasize, 0);
// Commands
const std::string upload_request = "upload";
const std::string extrinsic_request = "extrinsic";

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

static int act(
  void * socket, const StateMap * smap, const CloudMap * cmap, std::string * id,
  std::string * data
) {
  int err = 0;
  TransferState * state;
  // Find the client state
  auto itstate = smap->find(*id);
  if (itstate == smap.end()) {
    TransferState * nstate = new TransferState();
    (*smap)[*id] = nstate;
    state = nstate;
  } else {
    state = *itstate;
  }
  // Now are in the middle of a transfer that hasn't timed out
  return err;
}

int fetch_clouds(
  void * socket, const StateMap * smap, const CloudMap * cmap, int * termflag
) {
  int err = 0;
  int size;
  while(!err && !(*termflag)) {
    size = zmq_recv(socket, id_msg.c_str(), idsize, ZMQ_DONTWAIT);
    if (size == -1) break;
    id_msg.resize(size);
    size = zmq_recv(socket, data_msg.c_str(), maxdatasize, ZMQ_DONTWAIT);
    if (size == -1) break;
    data_msg.resize(size);
  }
  return err;
}
