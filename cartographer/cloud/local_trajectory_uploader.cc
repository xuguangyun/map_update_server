/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/cloud/local_trajectory_uploader.h"

#include <map>
#include <thread>

#include "absl/memory/memory.h"
#include "async_grpc/client.h"
#include "cartographer/cloud/handlers/add_rangefinder_data_handler.h"
#include "cartographer/common/blocking_queue.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace {

using absl::make_unique;

constexpr int kConnectionTimeoutInSeconds = 10;
constexpr int kConnectionRecoveryTimeoutInSeconds = 60;
constexpr int kTokenRefreshIntervalInSeconds = 60;
const common::Duration kPopTimeout = common::FromMilliseconds(100);

// This defines the '::grpc::StatusCode's that are considered unrecoverable
// errors and hence no retries will be attempted by the client.
const std::set<::grpc::StatusCode> kUnrecoverableStatusCodes = {
    ::grpc::DEADLINE_EXCEEDED,
    ::grpc::NOT_FOUND,
    ::grpc::UNAVAILABLE,
    ::grpc::UNKNOWN,
};



class LocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  struct TrajectoryInfo {
    const mapping::proto::TrajectoryBuilderOptions trajectory_options;
    const std::string client_id;
  };

 public:
  LocalTrajectoryUploader(const std::string& uplink_server_address);
  ~LocalTrajectoryUploader();

  // Starts the upload thread.
  void Start() final;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown() final;

  void EnqueueSensorData(std::unique_ptr<SensorData> sensor_data) final;
  void TryRecovery();

 private:
  void ProcessSendQueue();
  std::shared_ptr<::grpc::Channel> client_channel_;
  int batch_size_;
   TrajectoryInfo local_trajectory_id_to_trajectory_info_;
  common::BlockingQueue<std::unique_ptr<SensorData>> send_queue_;
  bool shutting_down_ = false;
  std::unique_ptr<std::thread> upload_thread_;
};

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string& uplink_server_address) {
  auto channel_creds = grpc::InsecureChannelCredentials();
  client_channel_ = ::grpc::CreateChannel(uplink_server_address, channel_creds);
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() +
      std::chrono::seconds(kConnectionTimeoutInSeconds);
  LOG(INFO) << "Connecting to uplink " << uplink_server_address;
  if (!client_channel_->WaitForConnected(deadline)) {
    LOG(ERROR) << "Failed to connect to " << uplink_server_address;
  }
  else
  {
      LOG(INFO) << "Connecting to uplink ok" << uplink_server_address;
  }
}

LocalTrajectoryUploader::~LocalTrajectoryUploader() {}

void LocalTrajectoryUploader::Start() {
  CHECK(!shutting_down_);
  CHECK(!upload_thread_);
  upload_thread_ =
      make_unique<std::thread>([this]() { this->ProcessSendQueue(); });
}

void LocalTrajectoryUploader::Shutdown() {
  CHECK(!shutting_down_);
  CHECK(upload_thread_);
  shutting_down_ = true;
  upload_thread_->join();
}

void LocalTrajectoryUploader::TryRecovery() {
  if (client_channel_->GetState(false /* try_to_connect */) !=
      grpc_connectivity_state::GRPC_CHANNEL_READY) {
    LOG(INFO) << "Trying to re-connect to uplink...";
    std::chrono::system_clock::time_point deadline =
        std::chrono::system_clock::now() +
        std::chrono::seconds(kConnectionRecoveryTimeoutInSeconds);
    if (!client_channel_->WaitForConnected(deadline)) {
      LOG(ERROR) << "Failed to re-connect to uplink prior to recovery attempt.";
      return;
    }
  }

  while (true) {
    if (shutting_down_) {
      return;
    }
}
}

void LocalTrajectoryUploader::ProcessSendQueue() {
  LOG(INFO) << "Starting uploader thread.";
  proto::AddRangefinderDataRequest request;
  while (!shutting_down_) {
    auto sensor_data = send_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {

       request.set_allocated_timed_point_cloud_data(&(sensor_data->data));

       request.set_allocated_local_pose(&(sensor_data->pose_data));

    //  request.sensor_metadata().set_client_id(&(local_trajectory_id_to_trajectory_info_.client_id));

//        async_grpc::Client<handlers::AddRangefinderDataSignature> client(
//            client_channel_, common::FromSeconds(kConnectionTimeoutInSeconds),
//            async_grpc::CreateUnlimitedConstantDelayStrategy(
//                common::FromSeconds(1), kUnrecoverableStatusCodes));
async_grpc::Client<handlers::AddRangefinderDataSignature> client(client_channel_);
        if ( client.Write(request)) {
          LOG(INFO) << "Uploaded " <<request.ByteSize()
                    << " bytes of sensor data.";
          request.Clear();
          continue;
        }
        // Unrecoverable error occurred. Attempt recovery.
        request.Clear();
        TryRecovery();
      }
    }

}

void LocalTrajectoryUploader::EnqueueSensorData(
    std::unique_ptr<SensorData> sensor_data) {
  send_queue_.Push(std::move(sensor_data));
}

}  // namespace

std::unique_ptr<LocalTrajectoryUploaderInterface> CreateLocalTrajectoryUploader(
    const std::string& uplink_server_address) {
  return make_unique<LocalTrajectoryUploader>(uplink_server_address);
}

}  // namespace cloud
}  // namespace cartographer
