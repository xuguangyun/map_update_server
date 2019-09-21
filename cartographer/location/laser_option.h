#ifndef CARTOGRAPHER_LOCATION_LASER_OPTION_H_
#define CARTOGRAPHER_LOCATION_LASER_OPTION_H_
#include "cartographer/mapping/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/cloud/map_builder_server_options.h"
#include <string>
namespace cartographer {
namespace location {

struct map_update {
    double min_range;
    double max_range;
    double missing_data_ray_length;
};
struct options {
  cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D  FastCSM_Option;
  cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions RealtimeCSM_Option;
  cartographer::sensor::proto::AdaptiveVoxelFilterOptions AdaFilterOption;
  cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D  CeresOption;
  cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D RangeDataInserter;
  cartographer::cloud::proto::MapBuilderServerOptions MapBuilderServe;
  std::string map_file;
  map_update map_update_;
  double resolution;
  double full_match_score_threshold;
  double realtime_match_score_threshold;
  bool use_imu;

};
options LoadOptions( const std::string& configuration_directory,
                     const std::string& configuration_basename);
}//namespace cartographer

}//namespace cartographer

#endif
