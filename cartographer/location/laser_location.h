#ifndef CARTOGRAPHER_LOCATION_LASER_LOCATION_H_
#define CARTOGRAPHER_LOCATION_LASER_LOCATION_H_

//#include "ros/ros.h"
#include <string>
//#include "sensor_msgs/MultiEchoLaserScan.h"

#include <cstring>
#include <stdexcept>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cartographer/common/time.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

#include "cartographer/mapping/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/probability_grid.h"
#include "cartographer/mapping/probability_grid_range_data_inserter_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/location/laser_option.h"
#include <SDL/SDL_image.h>

using namespace cartographer;
using namespace mapping;
using namespace common;
using namespace sensor;
using namespace cartographer::mapping::scan_matching;

namespace cartographer {
namespace location {

class Laserlocation
{
 public:
    Laserlocation( const std::string& configuration_directory, const std::string& configuration_basename) ;

    bool LoadMap(std::string& mapfile_);
    void ProcessLaserdata(sensor::TimedPointCloudData& time_point_cloud);
    void InitializeExtrapolator(const common::Time time) ;
    void AddImuData(const sensor::ImuData& imu_data);
    void AddOdometry(const sensor::OdometryData& odometry_data);
    void ProcessMapUpdate(transform::Rigid3d& pose_now,sensor::TimedPointCloudData& time_point_cloud);
    sensor::TimedPointCloudData GetTimePointCloudDataFilter();
    transform::Rigid2d current_pose;        //
    std::unique_ptr<ProbabilityGrid> probability_grid_;
    double  score_location;
    options location_option_;
    std::unique_ptr<FastCorrelativeScanMatcher2D>
            fast_correlative_scan_matcher_;
     bool location_ok=false;
     bool realtimeLocationValid = false;
 private:
    sensor::RangeData tranformToRangeData(transform::Rigid2d& pose_now, sensor::TimedPointCloudData& time_point_cloud);
    std::unique_ptr<RealTimeCorrelativeScanMatcher2D>
         real_time_correlative_scan_matcher_;

    std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
    std::unique_ptr<sensor::AdaptiveVoxelFilter>  adapt_filter_;
    std::unique_ptr<PoseExtrapolator> extrapolator_;
     std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;

    transform::Rigid2d fast_csm_pose;
    transform::Rigid2d realtime_csm_pose;
    transform::Rigid2d prediction_pose;
    ValueConversionTables conversion_tables;
     sensor::TimedPointCloudData time_point_cloud_filter;

public:
      float  score_fast_csm;
      float  score_realtime_csm;
};

}//namespace location

}//namespace cartographer
#endif
