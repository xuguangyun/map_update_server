#include "cartographer/location/laser_location.h"

namespace cartographer {
namespace location {


 Laserlocation::Laserlocation( const std::string& configuration_directory, const std::string& configuration_basename)
{
     location_option_=location::LoadOptions(configuration_directory,configuration_basename);
     if(LoadMap(location_option_.map_file))
     {
         fast_correlative_scan_matcher_= absl::make_unique<FastCorrelativeScanMatcher2D>(*probability_grid_, location_option_.FastCSM_Option);
         real_time_correlative_scan_matcher_ =absl::make_unique<RealTimeCorrelativeScanMatcher2D>(location_option_.RealtimeCSM_Option);
         ceres_scan_matcher_ = absl::make_unique<CeresScanMatcher2D>(location_option_.CeresOption);
         adapt_filter_=absl::make_unique<sensor::AdaptiveVoxelFilter>(location_option_.AdaFilterOption);
         range_data_inserter_=absl::make_unique<ProbabilityGridRangeDataInserter2D>(location_option_.RangeDataInserter);
     }
 }

bool Laserlocation::LoadMap(std::string& mapfile_)
{
     SDL_Surface * img;
     if(!(img = IMG_Load(mapfile_.c_str())))
     {
          return false;
     }

      probability_grid_=absl::make_unique<ProbabilityGrid>(
         MapLimits(location_option_.resolution, Eigen::Vector2d(location_option_.resolution*img->w*0.5, location_option_.resolution*img->h*0.5), CellLimits(img->h, img->w)),
         &conversion_tables);
      unsigned char* pixels;
      unsigned char* p;
      int rowstride, n_channels, avg_channels;
      unsigned int i,j;
      double occ;
      rowstride = img->pitch;
      n_channels = img->format->BytesPerPixel;
      avg_channels = n_channels - 1;
      int max_count=0;
      // Copy pixel data into the map structure
      pixels = (unsigned char*)(img->pixels);
      for(j = 0; j < img->h; j++)
       {
         for (i = 0; i < img->w; i++)
         {
           // Compute mean of RGB for this pixel
           p = pixels + j*rowstride + i*n_channels;
           // If negate is true, we consider blacker pixels free, and whiter
           // pixels occupied.  Otherwise, it's vice versa.
           //white=255;black=0
           occ = (255 - *p) / 255.0;
          //  occ = (*p) / 255.0;
           // Apply thresholds to RGB means to determine occupancy values for
           // map.  Note that we invert the graphics-ordering of the pixels to
           // produce a map with cell (0,0) in the lower-left corner.

           if(*p>255)
           {
               max_count++;
           }
          //  Eigen::Array2i index_(img->w-i,img->h-j);
              Eigen::Array2i index_(j,img->w-i);//!!!!!!!!!!!!!!!!!!
           //   Eigen::Array2i index_(i,img->w-j);
            if(*p!=128)
            {
              probability_grid_->SetProbability(index_,occ);
            }
           }
         }
        probability_grid_->FinishUpdate();
        return true;
}
// process laser data ;using CSM to location,
void Laserlocation::ProcessLaserdata(sensor::TimedPointCloudData& time_point_cloud)
{
    const common::Time& time = time_point_cloud.time;
    if(!location_option_.use_imu)
    {
        InitializeExtrapolator(time);
    }
    if (extrapolator_ == nullptr) { //>>changed by SGC
        if(location_option_.use_imu)
        {
            InitializeExtrapolator(time);  //>> make sure extrapolator initialized
            LOG(INFO) << "Extrapolator re-initialized.";
        }
        // Until we've initiatimelized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        else
        {
            LOG(INFO) << "Extrapolator not yet initialized.";
        }
      }
    //>>std::cout<<"time:"<<(time)<<std::endl;
    transform::Rigid3d range_data_poses=extrapolator_->ExtrapolatePose(time).cast<double>();//prediction
    transform::Rigid2d pose_prediction = transform::Project2D(range_data_poses);
    //>>std::cout<<"expect:"<<transform::ToProto(pose_prediction).DebugString()<<std::endl;

    sensor::PointCloud point_cloud_;
            point_cloud_.reserve(time_point_cloud.ranges.size());
            for (int i=0;i<time_point_cloud.ranges.size();i++) {
              point_cloud_.push_back(cartographer::sensor::ToRangefinderPoint(
                  time_point_cloud.ranges[i]));
            }
     sensor::PointCloud point_cloud_filter=adapt_filter_->Filter((point_cloud_));
     time_point_cloud_filter.time=time_point_cloud.time;
     time_point_cloud_filter.origin=time_point_cloud.origin;
     time_point_cloud_filter.ranges.reserve(point_cloud_.size());
     for(int i=0;i<point_cloud_.size();i++)
     {
         time_point_cloud_filter.ranges.push_back({point_cloud_[i].position,0});
     }
     if(!location_ok)
     {
         //>>std::cout<<"first location ..."<<std::endl;
          fast_correlative_scan_matcher_->MatchFullSubmap(point_cloud_filter, 0.3f, &score_fast_csm, &fast_csm_pose);
          const transform::Rigid3d pose_estimate_fast =
               transform::Embed3D(fast_csm_pose);
           extrapolator_->AddPose(time, pose_estimate_fast);
           realtimeLocationValid = extrapolator_->timedPoseValid;
           std::cout<<"score"<<score_fast_csm<<std::endl;
          std::cout<<"full_pos:"<<transform::ToProto(fast_csm_pose).DebugString()<<std::endl;
     }
     else
     {
         const SearchParameters search_parameters(location_option_.RealtimeCSM_Option.linear_search_window(),
                                                  location_option_.RealtimeCSM_Option.angular_search_window(),point_cloud_filter, location_option_.resolution);
         fast_correlative_scan_matcher_->MatchWithSearchParameters(search_parameters, pose_prediction, point_cloud_filter, 0.3f, &score_realtime_csm, &realtime_csm_pose);
         //score_realtime_csm = real_time_correlative_scan_matcher_->Match(pose_prediction, point_cloud_filter, *probability_grid_, &realtime_csm_pose);
          //>>std::cout<<"score"<<score_realtime_csm<<std::endl;
     }
     if(location_ok)
     {
       //>>std::cout<<"real_pose:"<<transform::ToProto(realtime_csm_pose).DebugString()<<std::endl;
       transform::Rigid2d pose_observation ;
       ceres::Solver::Summary summary;
      ceres_scan_matcher_->Match(realtime_csm_pose.translation(), realtime_csm_pose,
                                point_cloud_filter,
                                *probability_grid_, &pose_observation,
                                &summary);
      const transform::Rigid3d pose_estimate =
           transform::Embed3D(pose_observation);

       if(score_realtime_csm>location_option_.realtime_match_score_threshold)
       {
             current_pose=transform::Rigid2d(pose_observation.translation(),pose_observation.rotation());
             extrapolator_->AddPose(time, pose_estimate);
       }
     std::cout<<"ceres_now:"<<transform::ToProto(realtime_csm_pose).DebugString()<<std::endl;
     }
     if(score_fast_csm>location_option_.full_match_score_threshold&&location_ok==false)
      {
         location_ok=true;

         current_pose=transform::Rigid2d(fast_csm_pose.translation(),fast_csm_pose.rotation());
     }


}
void  Laserlocation::ProcessMapUpdate(transform::Rigid3d& pose_now,sensor::TimedPointCloudData& time_point_cloud)
{
    if(time_point_cloud.ranges.size()>0)
    {
    transform::Rigid2d pose_now_2d = transform::Project2D(pose_now);
    sensor::PointCloud point_cloud_;
            point_cloud_.reserve(time_point_cloud.ranges.size());
            for (int i=0;i<time_point_cloud.ranges.size();i++) {
              point_cloud_.push_back(cartographer::sensor::ToRangefinderPoint(
                  time_point_cloud.ranges[i]));
            }
      //STEP1: real_time_csm update location
   double  score = real_time_correlative_scan_matcher_->Match(pose_now_2d, point_cloud_,
                                                  *probability_grid_, &realtime_csm_pose);
     //STEP2: ceres for more precise location
     transform::Rigid2d pose_observation ;
     ceres::Solver::Summary summary;
    ceres_scan_matcher_->Match(realtime_csm_pose.translation(), realtime_csm_pose,
                              point_cloud_,
                              *probability_grid_, &pose_observation,
                              &summary);
     LOG(INFO) << "pose_data:"<<transform::ToProto(pose_observation).DebugString()<<std::endl;
    //STEP3: update the map
    if(score>0.35&&score<0.65)
    {
     sensor::RangeData range_data=tranformToRangeData(pose_observation, time_point_cloud);
     range_data_inserter_->Insert(range_data, probability_grid_.get());
      LOG(INFO) << "update map!:";
    }
    }

}
 sensor::TimedPointCloudData Laserlocation::GetTimePointCloudDataFilter()
 {
    return time_point_cloud_filter;
 }
 sensor::RangeData Laserlocation::tranformToRangeData(transform::Rigid2d& pose_now, sensor::TimedPointCloudData& time_point_cloud)
 {
     sensor::RangeData range_data_;
     for(int i=0;i<time_point_cloud.ranges.size();i++)
     {
         sensor::RangefinderPoint hit_in_local = sensor::ToRangefinderPoint(time_point_cloud.ranges[i]);
         double range=time_point_cloud.ranges[i].position.norm();
         if (range >= location_option_.map_update_.min_range) {
               if (range <= location_option_.map_update_.max_range) {
                 range_data_.returns.push_back(hit_in_local);
               } else {
                   float rate=(float)(location_option_.map_update_.missing_data_ray_length/range);
                   hit_in_local.position=rate*hit_in_local.position;
                range_data_.misses.push_back({ hit_in_local.position});
               }
             }
     }
     sensor::RangeData range_data_in_local =
           TransformRangeData(range_data_,
                              transform::Embed3D(pose_now.cast<float>()));
     range_data_in_local.origin=Eigen::Vector3f(pose_now.translation().x(), pose_now.translation().y(), 0);
     return range_data_in_local;
 }
void Laserlocation::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      10);
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}
void Laserlocation::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(location_option_.use_imu) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}
 void Laserlocation::AddOdometry(const sensor::OdometryData& odometry_data)
 {
     if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
      }
      extrapolator_->AddOdometryData(odometry_data);
 }



}//namespace location

}//namespace cartographer
