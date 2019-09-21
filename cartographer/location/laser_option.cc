#include "cartographer/location/laser_option.h"
#include "cartographer/common/configuration_file_resolver.h"
namespace cartographer {
namespace location {

cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
 CreateFastCorrelativeScanMatcherTestOptions2D(
    cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_angular_search_window(lua_parameter_dictionary->GetDouble("fast_angular_search_window"));
  options.set_linear_search_window(lua_parameter_dictionary->GetDouble("fast_linear_search_window"));
  options.set_branch_and_bound_depth(lua_parameter_dictionary->GetInt("fast_branch_and_bound_depth"));
  return options;
}
cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherTestOptions2D(cartographer::common::LuaParameterDictionary* const
                                                  lua_parameter_dictionary) {
    cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions options;
    options.set_linear_search_window(lua_parameter_dictionary->GetDouble("realtime_linear_search_window"));
    options.set_angular_search_window(lua_parameter_dictionary->GetDouble("realtime_angular_search_window"));
    options.set_translation_delta_cost_weight(lua_parameter_dictionary->GetDouble("realtime_translation_delta_cost_weight"));
    options.set_rotation_delta_cost_weight(lua_parameter_dictionary->GetDouble("realtime_rotation_delta_cost_weight"));

  return options;
}
 cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
 CreateCeresScanMatch2d(cartographer::common::LuaParameterDictionary* const
                        lua_parameter_dictionary)
 {
     cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D options;
     options.set_occupied_space_weight(lua_parameter_dictionary->GetDouble("ceres_occupied_space_weight"));
     options.set_translation_weight(lua_parameter_dictionary->GetDouble("ceres_translation_weight"));
     options.set_rotation_weight(lua_parameter_dictionary->GetDouble("ceres_rotation_weight"));
     options.mutable_ceres_solver_options()->set_use_nonmonotonic_steps(
                 lua_parameter_dictionary->GetBool("ceres_use_nonmonotonic_steps"));
     options.mutable_ceres_solver_options()->set_max_num_iterations(
                 lua_parameter_dictionary->GetNonNegativeInt("ceres_max_num_iterations"));
    options.mutable_ceres_solver_options()->set_num_threads(lua_parameter_dictionary->GetNonNegativeInt("ceres_num_threads"));
     return options;
 }
 cartographer::sensor::proto::AdaptiveVoxelFilterOptions
 CreateAdaptiveVoxelFilterOptions(cartographer::common::LuaParameterDictionary* const
                                  lua_parameter_dictionary){
     cartographer::sensor::proto::AdaptiveVoxelFilterOptions options;
     options.set_max_length(lua_parameter_dictionary->GetDouble("adafilter_max_length"));
     options.set_min_num_points(lua_parameter_dictionary->GetDouble("adafilter_min_num_points"));
     options.set_max_range(lua_parameter_dictionary->GetDouble("adafilter_max_range"));

     return options;

 }
 cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D
 CreateRnageDataInserterOptions(cartographer::common::LuaParameterDictionary* const
                                lua_parameter_dictionary){
 cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D options;
 options.set_hit_probability(lua_parameter_dictionary->GetDouble("hit_probability"));
 options.set_miss_probability(lua_parameter_dictionary->GetDouble("miss_probability"));
 options.set_insert_free_space(lua_parameter_dictionary->GetBool("insert_free_space"));
 return options;
 }
options LoadOptions(const std::string &configuration_directory, const std::string &configuration_basename)
{
    options opt;
    auto file_resolver =
         absl::make_unique<cartographer::common::ConfigurationFileResolver>(
             std::vector<std::string>{configuration_directory});
     const std::string code =
         file_resolver->GetFileContentOrDie(configuration_basename);
     cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
         code, std::move(file_resolver));
     opt.FastCSM_Option=CreateFastCorrelativeScanMatcherTestOptions2D(&lua_parameter_dictionary);
     opt.RealtimeCSM_Option=CreateRealTimeCorrelativeScanMatcherTestOptions2D(&lua_parameter_dictionary);
     opt.CeresOption=CreateCeresScanMatch2d(&lua_parameter_dictionary);
     opt.AdaFilterOption=CreateAdaptiveVoxelFilterOptions(&lua_parameter_dictionary);
     opt.RangeDataInserter=CreateRnageDataInserterOptions(&lua_parameter_dictionary);
     opt.MapBuilderServe=cartographer::cloud::CreateMapBuilderServerOptions(&lua_parameter_dictionary);
     opt.map_file=lua_parameter_dictionary.GetString("map_file");
     opt.resolution=lua_parameter_dictionary.GetDouble("resolution");
     opt.full_match_score_threshold=lua_parameter_dictionary.GetDouble("full_match_score_threshold");
     opt.realtime_match_score_threshold=lua_parameter_dictionary.GetDouble("realtime_match_score_threshold");
     opt.use_imu=lua_parameter_dictionary.GetBool("use_imu");
     opt.map_update_.min_range=lua_parameter_dictionary.GetDouble("map_update_min_range");
     opt.map_update_.max_range=lua_parameter_dictionary.GetDouble("map_update_max_range");
     opt.map_update_.missing_data_ray_length=lua_parameter_dictionary.GetDouble("map_update_missing_data_ray_length");
     return opt;
}

}//namespace location

}//namespace cartographer
