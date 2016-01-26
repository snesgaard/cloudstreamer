#include "network.h"
#include "util.h"

const static int maxcredit = 10;
const static int timeout = 3000; // Defined in ms
const static int chunksize = 2500;
const static int maxdatasize = chunksize * 3 * sizeof(float);
const static int idsize = 100;
const static int maxcmdsize = 20;
static char id_msg[idsize + 1];
static char data_msg[maxdatasize];
// Commands
const std::string upload_request = "upload";
const std::string uploadrgb_request = "uploadrgb";
const std::string extrinsic_request = "extrinsic";

TransferState::TransferState(){
  this->next_chunk = -1;
}

static int send_credit(void * socket, std::string dealid, int points) {
  int datastart = points;
  int datasize = chunksize;
  int err = 0;
  int flags = ZMQ_DONTWAIT | ZMQ_SNDMORE;
  zmq_send(socket, dealid.c_str(), dealid.size() - 1, flags);
  zmq_send(socket, &datastart, sizeof(datastart), flags);
  zmq_send(socket, &datasize, sizeof(datasize), ZMQ_DONTWAIT);
  return err;
}

static int add_data(
  void * socket, TransferState * state, CloudMap * cmap_in, CloudMap * cmap_out,
  std::string id, const char * data, const int data_size
) {
  const int psize = 3 * sizeof(float);
  auto it = cmap_in->find(id);
  if (it == cmap_in->end()) return 1;
  auto cloud = it->second;
  // info("sizes %i %i %i %i", data_size, psize, data_size / psize, chunksize);
  for (int i = psize; i <= data_size; i += psize) {
    //info("%i %i", i, data_size);
    float * fdata = (float *)(data + i - psize);
    //info("%f %f %f", fdata[0], fdata[1], fdata[2]);
    cloud->push_back(pcl::PointXYZ(fdata[0], fdata[1], fdata[2]));
  }
  if (data_size / psize < chunksize) {
    // Move cloud to out map and set state indicating we have finizes transfering
    cmap_in->erase(it);
    cmap_out->insert(std::make_pair(id, it->second));
    state->next_chunk = -1;
    //info("done with size %lu and isize %u %u", cloud->size(), id.size(), it->first.size());
  } else {
    // Send new transfer token
    send_credit(socket, id, state->next_chunk);
    state->next_chunk += chunksize;
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
    return add_data(socket, state, cmap_in, cmap_out, id, data, data_size);
  }
  // Now proceed to interpreting commands if we are not transfering
  //std::string deb(data, 20);
  //info("Got dis %i %s", data_size, deb.c_str());
  if (!strcmp(upload_request.c_str(), data)) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>
    );
    cmap_in->insert(std::make_pair(id, cloud));
    // Send transfer credit
    for (int c = 0; c < maxcredit; c++) {
      send_credit(socket, id, c * chunksize);
    }
    state->next_chunk = maxcredit * chunksize;
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
  volatile int * termflag
) {
  int err = 0;
  int isize;
  int dsize;
  while(!err && !(*termflag)) {
    cleartimeouts(smap);
    memset(id_msg, 0, idsize);
    isize = zmq_recv(socket, id_msg, idsize, ZMQ_DONTWAIT);
    if (isize == -1) break;
    //info("idsize %i", isize);

    // id_msg.resize(size);
    memset(data_msg, 0, maxcmdsize + 1);
    dsize = zmq_recv(socket, data_msg, maxdatasize, ZMQ_DONTWAIT);
    // info("got data %i", dsize);
    if (dsize == -1) break;
    // MAybe error guard here
    err = act(
      socket, smap, cmap_in, cmap_out, std::string(id_msg, isize + 1), data_msg,
      dsize
    );
    //data_msg.resize(size);
  }
  //info("lololo");
  //info("ending %i", cmap_out->size());
  for (auto it = cmap_out->begin(); it != cmap_out->end(); it++) {
    // Signal end of transfers
    //info("sending ending credits %i %lu", it->first.size(), it->second->size());
    send_credit(socket, it->first, -1);
    //info("done");
  }
  return err;
}
