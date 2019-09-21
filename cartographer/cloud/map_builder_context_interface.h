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

#ifndef CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H
#define CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H

#include "async_grpc/execution_context.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/rigid_transform.h"


namespace cartographer {
namespace cloud {

class MapBuilderServer;
class MapBuilderContextInterface : public async_grpc::ExecutionContext {
 public:
  struct Data { 
     std::string sensor_id ;
     std::string client_id;
    transform::Rigid3d pose_data;
    sensor::TimedPointCloudData data;
  };

  MapBuilderContextInterface() = default;
  ~MapBuilderContextInterface() = default;

  MapBuilderContextInterface(const MapBuilderContextInterface&) = delete;
  MapBuilderContextInterface& operator=(const MapBuilderContextInterface&) =
      delete;
  virtual common::BlockingQueue<std::unique_ptr<Data>>& sensor_data_queue() = 0;

  virtual void EnqueueSensorData(std::string& sensor_id, std::string& client_id,
                                 transform::Rigid3d& pose_data,sensor::TimedPointCloudData& data) = 0;

};

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H
