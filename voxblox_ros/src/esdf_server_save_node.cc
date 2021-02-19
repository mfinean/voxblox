#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string save_path = "/home/mark/Documents/gpu_voxels_analysis/heatmaps/voxblox_cow_and_lady_sdf.txt";
  Eigen::Vector3d origin(-11.2, -11.2, 0);
  Eigen::Vector3d grid_dims(448, 448, 128);

  voxblox::EsdfServer node(nh, nh_private);

  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed;

  while(true)
  {
  ros::spinOnce();

  end = std::chrono::system_clock::now();

  elapsed = end - begin;

  if(elapsed.count() - 220 >= 0){
      node.saveSDF(save_path, origin, grid_dims);
      break;
  }

  }

  return 0;
}
