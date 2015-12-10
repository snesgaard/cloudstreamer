#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <zmq.h>

#include <vector>
#include <thread>
#include <chrono>
#include <future>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "network.h"
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
  void * socket = zmq_socket(context, ZMQ_ROUTER);
  if (!socket) {
    error("Failed to create socket");
    return 1;
  }
  char host[200];
  sprintf(host, "ipc:///tmp/datastream");
  //errguard(err, zmq_setsockopt(socket, ZMQ_SUBSCRIBE, NULL, 0));
  int linger = 100;
  errguard(err, zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger)));
  errguard(err, zmq_bind(socket, host));

  info("Created subscriber at host address %s", host);
  auto viewer = pcl::visualization::PCLVisualizer("3D Viewer");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(-10, 0, 0, 1, 0, 0);
  /*
  cloudfuture = std::async(
    std::launch::async, recv_cloud_begin, socket, backcloud
  );
  */
  StateMap smap;
  CloudMap cmap_in;
  CloudMap cmap_out;
  auto future_err = std::async(
    std::launch::async, fetch_clouds, socket, &smap, &cmap_in, &cmap_out,
    &terminate
  );
  while (!viewer.wasStopped() && !terminate)
  {
    auto status = future_err.wait_for(std::chrono::milliseconds(16));
    if (status == std::future_status::ready) {
      auto cmap_tmp = cmap_out;
      cmap_out.clear();
      future_err = std::async(
        std::launch::async, fetch_clouds, socket, &smap, &cmap_in, &cmap_out,
        &terminate
      );
      for (auto it = cmap_tmp.begin(); it != cmap_tmp.end(); it++) {
        //viewer.updatePointCloud(it->second, it->first);
        //viewer.removePointCloud(it->first);
        if (!viewer.updatePointCloud(it->second, it->first)) {
          viewer.addPointCloud(it->second, it->first);
        }
      }
    }
    viewer.spinOnce(5);
  }
  terminate = true;
  info("Releasing resources");
  future_err.wait();
  viewer.close();
  errguard(err, zmq_close(socket));
  errguard(err, zmq_term(context));

  return 0;
}
