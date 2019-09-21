options = {
	
	fast_linear_search_window = 3,
	fast_angular_search_window = 1,
	fast_branch_and_bound_depth =8, 
	
	realtime_linear_search_window = 0.5, 
	realtime_angular_search_window = 0.26, 
	realtime_translation_delta_cost_weight = 0, 
	realtime_rotation_delta_cost_weight = 0, 
	
	ceres_occupied_space_weight = 1,
	ceres_translation_weight = 0.1,
	ceres_rotation_weight = 1.5, 
        ceres_use_nonmonotonic_steps=true,
	ceres_max_num_iterations =50,
        ceres_num_threads=1, 
	
	
	adafilter_max_length = 0.5, 
	adafilter_min_num_points = 100, 
	adafilter_max_range = 30, 

        map_update_min_range=0.5,
        map_update_max_range=30,
        map_update_missing_data_ray_length=5,
	
        hit_probability=0.85,
        miss_probability=0.25,
        insert_free_space=true,

	server_address="127.0.0.1:55555",
        num_grpc_threads=4,
	num_event_threads=4,
	uplink_server_address="",

	map_file = "/root/workspace/carto_server/configuration_files/map.png",
	resolution = 0.05,
	full_match_score_threshold = 0.21,
	realtime_match_score_threshold = 0.31,
        use_imu=false,
}
return options

