# robot_localization_ranges

This packages builds on `robot_localization` to introduce range measurements in EKF.

Based on two classes that inherit from `robot_localization`:

 - `Ekf` (`public robot_localization::Ekf`): the EKF itself
 - `RosFilterRanges` (`public robot_localization::RosFilter<Ekf>`): the ROS interface
