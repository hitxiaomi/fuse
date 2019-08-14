/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_CORE_SERIALIZATION_H
#define FUSE_CORE_SERIALIZATION_H

#include <fuse_core/uuid.h>

#include <ros/time.h>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/xml_iarchive.hpp>
//#include <boost/archive/xml_oarchive.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/uuid/uuid_serialize.hpp>

// #include <cereal/cereal.hpp>
// #include <cereal/archives/binary.hpp>
// #include <cereal/archives/json.hpp>
// #include <cereal/archives/portable_binary.hpp>
// #include <cereal/archives/xml.hpp>

#include <Eigen/Core>

namespace boost
{
namespace serialization
{

/**
 * @brief Serialize a ros::Time variable using Boost Serialization
 */
template<class Archive>
void serialize(Archive& archive, ros::Time& stamp, const unsigned int version)
{
  archive & stamp.sec;
  archive & stamp.nsec;
}

/**
 * @brief Serialize an Eigen Matrix using Boost Serialization
 */
template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void save(
  Archive & ar,
  const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
  const unsigned int version)
{
  int rows = g.rows();
  int cols = g.cols();

  ar & rows;
  ar & cols;
  ar & boost::serialization::make_array(g.data(), rows * cols);
}

template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void load(
  Archive & ar,
  Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
  const unsigned int version)
{
  int rows, cols;
  ar & rows;
  ar & cols;
  g.resize(rows, cols);
  ar & boost::serialization::make_array(g.data(), rows * cols);
}

template<class Archive,
         class S,
         int Rows_,
         int Cols_,
         int Ops_,
         int MaxRows_,
         int MaxCols_>
inline void serialize(
  Archive & ar,
  Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
  const unsigned int version)
{
  split_free(ar, g, version);
}

}  // namespace serialization
}  // namespace boost

#endif  // FUSE_CORE_SERIALIZATION_H
