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

#include "cartographer/cloud/map_builder_server.h"
#include "cartographer/cloud/handlers/add_rangefinder_data_handler.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {
namespace {



constexpr int kMaxMessageSize = 100 * 1024 * 1024;  // 100 MB
const common::Duration kPopTimeout = common::FromMilliseconds(100);

}  // namespace
MapBuilderContext::MapBuilderContext(
    MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
MapBuilderContext::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

void MapBuilderContext::EnqueueSensorData(std::string& sensor_id, std::string& client_id,transform::Rigid3d& pose_data,
                                                      sensor::TimedPointCloudData& data) {
  map_builder_server_->incoming_data_queue_.Push(
      absl::make_unique<Data>(Data{sensor_id,client_id,pose_data,data}));
}

MapBuilderServer::MapBuilderServer(
    const std::string& configuration_directory, const std::string& configuration_basename)
{
     location_=absl::make_unique<cartographer::location::Laserlocation>(configuration_directory,configuration_basename);
     proto::MapBuilderServerOptions map_builder_server_options=LoadMapBuilderServerOptions(configuration_directory,configuration_basename);


     if (!map_builder_server_options.uplink_server_address().empty()) {
       local_trajectory_uploader_ = CreateLocalTrajectoryUploader(
           map_builder_server_options.uplink_server_address());
     }
     else
     {
         async_grpc::Server::Builder server_builder;
         server_builder.SetServerAddress(map_builder_server_options.server_address());
          LOG(INFO) << "server_address:"<<map_builder_server_options.server_address();
         server_builder.SetNumGrpcThreads(
             map_builder_server_options.num_grpc_threads());
         server_builder.SetNumEventThreads(
             map_builder_server_options.num_event_threads());
         server_builder.SetMaxSendMessageSize(kMaxMessageSize);
         server_builder.SetMaxReceiveMessageSize(kMaxMessageSize);
          server_builder.RegisterHandler<handlers::AddRangefinderDataHandler>();
           grpc_server_ = server_builder.Build();

           grpc_server_->SetExecutionContext(
                  absl::make_unique<MapBuilderContext>(this));
     }

}

void MapBuilderServer::Start() {
  shutting_down_ = false;
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Start();
    StartLocalSlamThread();
  }
  else
  {
    StartGlobalMapThread();
    grpc_server_->Start();
  }
}

void MapBuilderServer::WaitForShutdown() {
  grpc_server_->WaitForShutdown();
  if (globle_thread_) {
    globle_thread_->join();
  }
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Shutdown();
  }
}

void MapBuilderServer::Shutdown() {
  shutting_down_ = true;
  grpc_server_->Shutdown();
  if (globle_thread_) {
    globle_thread_->join();
    globle_thread_.reset();
  }
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Shutdown();
    local_trajectory_uploader_.reset();
  }
}

void MapBuilderServer::ProcessSensorDataQueue() {
  LOG(INFO) << "Starting SLAM thread.";
  while (!shutting_down_) {
    std::unique_ptr<MapBuilderContextInterface::Data> sensor_data =
        incoming_data_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
   //   TODO add data process
         location_->ProcessMapUpdate(sensor_data->pose_data,sensor_data->data);
    }
  }
}
void MapBuilderServer::ProcessLocalSlamData(sensor::TimedPointCloudData& time_point_cloud)
{
     location_->ProcessLaserdata(time_point_cloud);
      std::unique_ptr<LocalTrajectoryUploaderInterface::SensorData> sensor_data=
              absl::make_unique<LocalTrajectoryUploaderInterface::SensorData>();
    transform::Rigid3d pose_estimate =
               transform::Embed3D(location_->current_pose);
     transform::proto::Rigid3d pose_data=cartographer::transform::ToProto(pose_estimate);
    sensor::TimedPointCloudData data_=location_->GetTimePointCloudDataFilter();
    sensor::proto::TimedPointCloudData data_proto=cartographer::sensor::ToProto(data_);
     //sensor_data->pose_data=pose_data;
    sensor_data->pose_data.set_allocated_rotation(pose_data.mutable_rotation());//= pose_data;
     sensor_data->pose_data.set_allocated_translation(pose_data.mutable_translation());
    sensor_data->data.set_timestamp(common::ToUniversal(data_.time));
    sensor_data->data.set_allocated_origin(data_proto.mutable_origin());
     sensor_data->data.mutable_point_data()->Reserve(data_.ranges.size());
     for (const TimedRangefinderPoint& range : data_.ranges) {
        *sensor_data->data.add_point_data() = ToProto(range);
      }
    local_trajectory_uploader_->EnqueueSensorData(std::move(sensor_data));
     LOG(INFO) << "Enqueue data";
}
void MapBuilderServer::StartGlobalMapThread() {
  CHECK(!globle_thread_);
  // Start the SLAM processing thread.
  globle_thread_ = absl::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}
void MapBuilderServer::StartLocalSlamThread() {

}


}  // namespace cloud
}  // namespace cartographer
