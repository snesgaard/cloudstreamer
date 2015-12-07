#ifndef __NETWORK_H__
#define __NETWORK_H__

#include <zmq.h>

#include <map>
#include <string>
#include <chrono>
#include <pcl/common/common_headers.h>

struct TransferState{
  int next_chunk;
  // std::chrono::duration::milliseconds last_active; Make timout timer here
};

typedef std::map<std::string, TransferState *> StateMap;
typedef std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;

int fetch_clouds(
  void * socket, const StateMap * smap, const CloudMap * cmap_in,
  const CloudMap * cmap_out, int * termflag
);


#endif // __NETWORK_H__
