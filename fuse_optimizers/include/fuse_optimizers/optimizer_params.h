/***************************************************************************
 * Copyright (C) 2019 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef FUSE_OPTIMIZERS_OPTIMIZER_PARAMS_H
#define FUSE_OPTIMIZERS_OPTIMIZER_PARAMS_H

#include <fuse_core/uuid.h>
#include <fuse_variables/stamped.h>
#include <locus_cpp/ros_util.h>
#include <ros/node_handle.h>

#include <string>


namespace fuse_optimizers
{

/**
 * @brief Defines the set of parameters required by the fuse_optimizers::FixedLagSmoother class
 */
struct OptimizerParams
{
public:

  /**
   * @brief The duration of the smoothing window in seconds
   */
  double lag_duration { 5.0 };

  /**
   * @brief The target frequency for optimization cycles. If an optimization takes longer than expected, an
   *        optimization cycle may be skipped.
   */
  double optimization_frequency { 10.0 };

  /**
   * @brief The maximum time to wait for motion models to be generated for a received transactions.
   *
   * Transactions are processes sequentially, so no new transactions will be added to the graph while waiting for
   * motion models to be generated. Once the timeout expires, that transaction will be deleted from the queue.
   */
  double transaction_timeout { 1.0 };

  /**
   * @brief The topic name of the advertised reset service
   */
  std::string reset_service { "reset" };

  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh)
  {
    locus_cpp::checkRequired(nh, {"topic"});  // NOLINT

    // Read settings from the parameter server
    device_id = fuse_variables::loadDeviceId(nh);
    queue_size = locus_cpp::positiveParam(nh, "queue_size", queue_size);
    nh.getParam("topic", topic);
  }
};

}  // namespace fuse_optimizers

#endif  // FUSE_OPTIMIZERS_OPTIMIZER_PARAMS_H
