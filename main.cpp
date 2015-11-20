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

#include "util.h"

const int servercredit = 10;
const int pchunk = 10000;

static volatile int terminate = 0;
// Interrupt handling should probably be done more cleverly.
// This will do for now
void intHandler(int dummy) {
  terminate = 1;
}

std::vector<float> recv_float_array(void * socket) {
  int err = 0;
  std::vector<float> arr;
  zmq_msg_t msg;
  errguard(err, zmq_msg_init(&msg));
  int more;
  size_t more_size;
  do {
    int data_size = zmq_msg_recv(&msg, socket, 0);
    errguard(err, zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &more_size));
    if (data_size != -1) {
      float * data = (float *)zmq_msg_data(&msg);
      for (uint i = 0; i < data_size / sizeof(float); i++) {
        arr.push_back(data[i]);
      }
    }
  } while(more);
  errguard(err, zmq_msg_close(&msg));
  return arr;
}

int send_credit(void * socket, zmq_msg_t dealid, int points) {
  int datastart = points;
  int datasize = pchunk;
  int err = 0;
  int flags = ZMQ_DONTWAIT | ZMQ_SNDMORE;
  zmq_send(socket, zmq_msg_data(&dealid), zmq_msg_size(&dealid), flags);
  zmq_send(socket, &datastart, sizeof(datastart), flags);
  zmq_send(socket, &datasize, sizeof(datasize), ZMQ_DONTWAIT);
  return err;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr recv_cloud_end(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, zmq_msg_t dealid
) {
  int err = 0;
  errguard(err, zmq_msg_close(&dealid));
  // info("transfer compelte");
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr recv_cloud_run(
  void * socket, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, zmq_msg_t dealid,
  int points, zmq_msg_t datamsg
) {
  int idsize = zmq_msg_recv(&dealid, socket, 0);
  int datasize = zmq_msg_recv(&datamsg, socket, 0);
  if (datasize != -1) {
    // info("got %i", datasize);
    float * data = (float *)zmq_msg_data(&datamsg);
    for (int i = 0; i + 2 < datasize / sizeof(float); i += 3) {
      cloud->push_back(pcl::PointXYZ(data[i], data[i + 1], data[i + 2]));
    }
  }
  if (datasize < pchunk * 3 * sizeof(float)) {
    int err = 0;
    errguard(err, send_credit(socket, dealid, -1));
    errguard(err, zmq_msg_close(&datamsg));
    return recv_cloud_end(cloud, dealid);
  }
  send_credit(socket, dealid, points);
  return recv_cloud_run(socket, cloud, dealid, points + pchunk, datamsg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr recv_cloud_begin(
  void * socket, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
) {
  int err = 0;
  cloud->clear();
  zmq_msg_t dealid;
  zmq_msg_t msg;
  errguard(err, zmq_msg_init(&dealid));
  errguard(err, zmq_msg_init(&msg));
  const char * request = "upload";
//  info("Listening");
  do {
    int idsize = zmq_msg_recv(&dealid, socket, ZMQ_DONTWAIT);
    // If an id was recieved, proceed to handle the rrequest
    if (idsize != -1) {
      int msgsize = -1;
      // Wait for up to 50 ms for the rquest to arrive
      for (int i = 0; i < 10 && msgsize == -1; i++) {
        msgsize = zmq_msg_recv(&msg, socket, ZMQ_DONTWAIT);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      // If message matched, exit and continue to data transfer
      if (msgsize != -1 && !strcmp(request, (char *)zmq_msg_data(&msg))) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while(!terminate);

  errguard(err, zmq_msg_close(&msg));

  if (terminate) {
    return recv_cloud_end(cloud, dealid);
  }
  //info("Request recieved");
  for (int i = 0; i < servercredit; i++) {
    errguard(err, send_credit(socket, dealid, i * pchunk));
  }

  zmq_msg_t datamsg;
  errguard(err, zmq_msg_init(&datamsg));
  return recv_cloud_run(socket, cloud, dealid, servercredit * pchunk, datamsg);
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
  errguard(err, zmq_connect(socket, host));

  info("Created subscriber at host address %s", host);
  auto viewer = pcl::visualization::PCLVisualizer("3D Viewer");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  pcl::PointCloud<pcl::PointXYZ>::Ptr frontcloud(
    new pcl::PointCloud<pcl::PointXYZ>
  );
  pcl::PointCloud<pcl::PointXYZ>::Ptr backcloud(
    new pcl::PointCloud<pcl::PointXYZ>
  );
  // Test
  /*
  for (int i = 0; i < 10; i++) {
    frontcloud->push_back(pcl::PointXYZ(i, i, i));
  }
  */
  const char * cloudkey = "prime";
  viewer.addPointCloud(frontcloud, cloudkey);
  std::future<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudfuture;
  cloudfuture = std::async(
    std::launch::async, recv_cloud_begin, socket, backcloud
  );
  while (!viewer.wasStopped() && !terminate)
  {
    auto status = cloudfuture.wait_for(std::chrono::milliseconds(16));
    if (status == std::future_status::ready) {
      viewer.removePointCloud(cloudkey);
      backcloud = frontcloud;
      frontcloud = cloudfuture.get();
      viewer.addPointCloud(frontcloud, cloudkey);
      cloudfuture = std::async(
        std::launch::async, recv_cloud_begin, socket, backcloud
      );
    }
    viewer.spinOnce (100);
  }
  terminate = true;
  info("Releasing resources");
  cloudfuture.wait();
  viewer.close();
  errguard(err, zmq_close(socket));
  errguard(err, zmq_term(context));

  return 0;
}
