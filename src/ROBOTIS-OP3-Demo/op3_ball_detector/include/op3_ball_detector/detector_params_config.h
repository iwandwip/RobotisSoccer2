/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef _DETECTOR_PARAMS_CONFIG_H_
#define _DETECTOR_PARAMS_CONFIG_H_

#include <rclcpp/rclcpp.hpp>

namespace robotis_op
{


class DetectorParamsConfig
{
 public:
  int gaussian_blur_size;         // size of gaussian blur kernel mask [pixel]
  double gaussian_blur_sigma;     // sigma of gaussian blur kernel mask [pixel]
  double canny_edge_th;           // threshold of the edge detector.
  double hough_accum_resolution;  // resolution of the Hough accumulator, in terms of inverse ratio of image resolution
  double min_circle_dist;         // Minimum distance between circles
  double hough_accum_th;          // accumulator threshold to decide circle detection
  int min_radius;                 // minimum circle radius allowed
  int max_radius;                 // maximum circle radius allowed
  int filter_h_min;               // filter threshold
  int filter_h_max;               // filter threshold
  int filter_s_min;               // filter threshold
  int filter_s_max;               // filter threshold
  int filter_v_min;               // filter threshold
  int filter_v_max;               // filter threshold
  bool use_second_filter;
  int filter2_h_min;              // filter threshold
  int filter2_h_max;              // filter threshold
  int filter2_s_min;              // filter threshold
  int filter2_s_max;              // filter threshold
  int filter2_v_min;              // filter threshold
  int filter2_v_max;              // filter threshold
  int ellipse_size;
  bool debug_image;                     // to debug log

  DetectorParamsConfig()
      : canny_edge_th(CANNY_EDGE_TH_DEFAULT),
        hough_accum_resolution(HOUGH_ACCUM_RESOLUTION_DEFAULT),
        min_circle_dist(MIN_CIRCLE_DIST_DEFAULT),
        hough_accum_th(HOUGH_ACCUM_TH_DEFAULT),
        min_radius(MIN_RADIUS_DEFAULT),
        max_radius(MAX_RADIUS_DEFAULT),
        filter_h_min(FILTER_H_MIN_DEFAULT),
        filter_h_max(FILTER_H_MAX_DEFAULT),
        filter_s_min(FILTER_S_MIN_DEFAULT),
        filter_s_max(FILTER_S_MAX_DEFAULT),
        filter_v_min(FILTER_V_MIN_DEFAULT),
        filter_v_max(FILTER_V_MAX_DEFAULT),
        use_second_filter(false),
        filter2_h_min(FILTER_H_MIN_DEFAULT),
        filter2_h_max(FILTER_H_MAX_DEFAULT),
        filter2_s_min(FILTER_S_MIN_DEFAULT),
        filter2_s_max(FILTER_S_MAX_DEFAULT),
        filter2_v_min(FILTER_V_MIN_DEFAULT),
        filter2_v_max(FILTER_V_MAX_DEFAULT),
        ellipse_size(ELLIPSE_SIZE),
        debug_image(false)
  {
  }

  ~DetectorParamsConfig()
  {
  }
};

}
#endif  // _DETECTOR_PARAMS_CONFIG_H_
