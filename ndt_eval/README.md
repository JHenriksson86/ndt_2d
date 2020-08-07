# Fuzzy-point set registration evaluation

## Overview
This packaged is made to evaluate the fuzzy registration python package. It takes pcd files and converts them into PointCloud2 messages, which are sent to fuzzy_psr_python for registration. It can also utilize odometry as an intial guess, for this to work the odometry needs transformed to the sensor frame. To run the evaluation procedure there are two launch files available, with and without visualization. At the moment only the fine registration procedure is implemented.

## Parameters
In the launch files there are several parameters that can be changed
* pcd_path: The path to the pcd files
* pcd_name: The name of the pcd files, the files should be numbered in order like pcd_name1.pcd
* pcd_start_number: Start number of the pcd files
* pcd_stop_number: Stop number of the pcd files
* pcd_step_size: The step size between pcd files, since the registration procedure can handle bigger jumps between pointclouds.
* odometry_file: File containing the odometry this file is also necessary for to get the stamp the file formatting should be: Stamp Postion.x Position.y Position.z Orientation.x Orientation.y Orientation.z Orientation.w Endline
* output_file: Output file for the registration result this will be formatted in the same way as the odometry file.
* initial_guess: True or False if the registration should use the initial guess.

* param_coarse_registration: Enable/Disable coarse registration, not implemented at the moment.
* param_fine_registration: Enable/Disable fine registration, If fine registration is disabled the registration will only output the initial guess.
* param_trim_ratio: Makes the registration procedure ignore the worst matches.
* param_noisy_fixed_cloud: Coarse registration parameter, not implemented at the moment.
* param_noisy_movable_cloud
* param_epsilon: Coarse registration parameter, not implemented at the moment.
* param_rotation_range: Coarse registration parameter, not implemented at the moment.
* param_translation_range: Coarse registration parameter, not implemented at the moment.
* param_number_of_clusters: Coarse registration parameter, not implemented at the moment.
* param_sigma_r: Coarse registration parameter, not implemented at the moment.
* param_sigma_t: Coarse registration parameter, not implemented at the moment.
* param_coarse_downsampling_fixed_cloud: Coarse registration parameter, not implemented at the moment.
* param_coarse_downsampling_movable_cloud: Coarse registration parameter, not implemented at the moment.
* param_fine_downsampling: Enable/Disable downsampling for the fine registration. Downsampling can be good to disable when pointclouds smaller than 1000 points are used.
* param_fine_downsampling_fixed_cloud: Average grid downsampling is used, the parameter is how big the grid size in meters should be.
* param_fine_downsampling_movable_cloud: Average grid downsampling is used, the parameter is how big the grid size in meters should be.