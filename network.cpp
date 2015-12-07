#include "network.h"
#include "util.h"

const static int timeout = 3000; // Defined in ms
const static int chunksize = 2500;
const static int maxdatasize = chunksize * 3 * sizeof(float);
const static int idsize = 100;
static char id_msg[idsize];
static char data_msg[maxdatasize];
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

static int add_data(
  StateMap * smap, CloudMap * cmap_in, CloudMap * cmap_out, std::string id,
  const char * data, const int data_size
) {
  const int psize = 3 * sizeof(float);
  for (int i = psize; i <= data_size; i += psize) {
    float * fdata = (float *)(data + i - psize);
    // add fdata[0], fdata[1], fdata[2]
  }
  if (data_size / psize < chunksize) {
    // here we have the end of our data so we need to aboort
  } else {
    // Send new transfer token
  }
  return 0;
}

static int act(
  void * socket, StateMap * smap, CloudMap * cmap_in, CloudMap * cmap_out,
  std::string id, const char * data, const int data_size
) {
  int err = 0;
  TransferState * state;
  // Find the client state
  auto itstate = smap->find(id);
  if (itstate == smap->end()) {
    TransferState * nstate = new TransferState();
    smap->insert(std::make_pair(id, nstate));
    state = nstate;
  } else {
    state = itstate->second;
  }
  // Now are we in the middle of a transfer that hasn't timed out
  if (state->next_chunk > -1) {
    return add_data(smap, cmap_in, cmap_out, id, data, data_size);
  }
  // Now proceed to interpreting commands if we are not transfering
  if (!strcmp(upload_request.c_str(), data)) {
    // Allocate new pointcloud
    // Send transfer credit
  } else {
    info("Retrieved unsupported command from client");
    err = 1; // Insert meaningful error code here
  }
  return err;
}

void cleartimeouts(const StateMap * smap) {

}

int fetch_clouds(
  void * socket, StateMap * smap, CloudMap * cmap_in, CloudMap * cmap_out,
  int * termflag
) {
  int err = 0;
  int isize;
  int dsize;
  while(!err && !(*termflag)) {
    cleartimeouts(smap);
    isize = zmq_recv(socket, id_msg, idsize, ZMQ_DONTWAIT);
    if (isize == -1) break;
    //id_msg.resize(size);
    dsize = zmq_recv(socket, data_msg, maxdatasize, ZMQ_DONTWAIT);
    if (dsize == -1) break;
    err = act(
      socket, smap, cmap_in, cmap_out, std::string(id_msg, isize), data_msg,
      dsize
    );
    //data_msg.resize(size);
  }
  return err;
}
