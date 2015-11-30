#ifndef __NETWORK_H__
#define __NETWORK_H__

#include <zmq.h>

#include <map>
#include <string>

struct TransferState{
  int next_chunk;
  double last_active;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

typedef std::map<std::string, TransferState *> StateMap;
typedef std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;

int fetch_clouds(
  void * socket, const StateMap * smap, const CloudMap * cmap, int * termflag
);


#endif // __NETWORK_H__
