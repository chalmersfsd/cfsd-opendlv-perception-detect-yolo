/*
 * Copyright (C) 2019   Felix Hörnschemeyer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BIRDVIEW_PERCEPTION
#define BIRDVIEW_PERCEPTION

// Take stereo-vision depth value if depthConfidence
// is above depthConfidenceThreshold. Units: %
const float depthConfidenceThreshold = 55;
const float depthDistanceThreshold = 4;

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <yolo_v2_class.hpp>

#include <iostream>

struct bboxConf_t : bbox_t {
  float depthConfidence;
  bboxConf_t(bbox_t& temp):bbox_t(temp), depthConfidence(0.0f){}
};

struct cameraPara {
  double focLength_mm = 0.0;
  double sensHeight_mm = 0.0;
  double cx = 0.0;
  double focLength_pix = 0.0;
  double sensHeight_pix = 0.0;
  double framewidth_mm = 0.0;
};

// Set up camera parameters
cameraPara setupCameraPara(uint32_t height, uint32_t camera);

// Calculate conde distance by hight of cone in frame
opendlv::logic::perception::ObjectPosition getDistance(cameraPara camPara, bboxConf_t &detection, bool verbose = false);
#endif
