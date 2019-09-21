/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H

#include "async_grpc/execution_context.h"
#include "async_grpc/server.h"
#include "cartographer/cloud/map_builder_context_interface.h"
#include "cartographer/cloud/map_builder_server_interface.h"
#include "cartographer/cloud/proto/map_builder_server_options.pb.h"
#include "cartographer/cloud/local_trajectory_uploader.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/time.h"
#include "cartographer/location/laser_location.h"

namespace cartographer {
namespace cloud {

class MapBuilderServer;

class MapBuilderContext : public MapBuilderContextInterface {
 public:
  MapBuilderContext(MapBuilderServer* map_builder_server);
  common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
  sensor_data_queue() override;

  void EnqueueSensorData(std::string& sensor_id, std::string& client_id,
                         transform::Rigid3d& pose_data,sensor::TimedPointCloudData& data) override;


 private:
  MapBuilderServer* map_builder_server_;
  std::map</*trajectory_id=*/int, /*client_id=*/std::string> client_ids_;
};

class MapBuilderServer{
 public:

  MapBuilderServer(
      const std::string& configuration_directory, const std::string& configuration_basename);
  ~MapBuilderServer() {}

  // Starts the gRPC server, the 'LocalTrajectoryUploader' and the SLAM thread.
  void Start() ;

  // Waits for the 'MapBuilderServer' to shut down. Note: The server must be
  // either shutting down or some other thread must call 'Shutdown()' for this
  // function to ever return.
  void WaitForShutdown() ;


  // Shuts down the gRPC server, the 'LocalTrajectoryUploader' and the SLAM
  // thread.
  void Shutdown() ;

  void ProcessLocalSlamData(sensor::TimedPointCloudData& time_point_cloud);
  std::unique_ptr<cartographer::location::Laserlocation>  location_;
  common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>
      incoming_data_queue_;
 private:

  void ProcessSensorDataQueue();
  void StartGlobalMapThread();
  void StartLocalSlamThread();

  bool shutting_down_ = false;
  std::unique_ptr<std::thread> globle_thread_;
   std::unique_ptr<std::thread>local_thread_;
  std::unique_ptr<async_grpc::Server> grpc_server_;

  absl::Mutex subscriptions_lock_;
  int current_subscription_index_ = 0;
  std::unique_ptr<LocalTrajectoryUploaderInterface> local_trajectory_uploader_;
  int starting_submap_index_ = 0;

};

}  // namespace cloud
}  // namespace cartographer

#include "cartographer/cloud/map_builder_context_impl.h"

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H
