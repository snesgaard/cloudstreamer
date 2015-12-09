#ifndef __NETWORK_H__
#define __NETWORK_H__

#include <zmq.h>

#include <map>
#include <unordered_map>
#include <string>
#include <chrono>
#include <pcl/common/common_headers.h>

struct TransferState{
  TransferState();
  int next_chunk;
  // Insert timeouts here
};

typedef std::unordered_map<std::string, TransferState *> StateMap;
typedef std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;

int fetch_clouds(
  void * socket, StateMap * smap, CloudMap * cmap_in, CloudMap * cmap_out,
  volatile int * termflag
);


#endif // __NETWORK_H__
