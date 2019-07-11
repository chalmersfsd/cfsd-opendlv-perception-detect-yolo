/*
 * Copyright (C) 2019 Ola Benderius
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

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <yolo_v2_class.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"


int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cfg-file")) || (0 == commandlineArguments.count("weight-file")) ) {
    std::cerr << argv[0] << " detects and classifies objects in a camera feed by using Yolo." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " [--verbose]" << std::endl;
    std::cerr << "         --cfg-file: Yolo configuration file" << std::endl;
    std::cerr << "         --weight-file: Yolo weight file" << std::endl;
    std::cerr << "         --name.argb: name of the shared memory for the ARGB formatted image (default: video0.argb)" << std::endl;
    std::cerr << "         --verbose: prints diagnostics data to screen" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cfg-file=yolo.cfg --weight-file=yolo.weight [--name.argb=video0.argb] [--verbose]" << std::endl;
  } else {
    std::string const NAME_ARGB{(commandlineArguments["name.argb"].size() != 0) ? commandlineArguments["name.argb"] : "video0.argb"};
   // bool const VERBOSE{commandlineArguments.count("verbose") != 0};

    int32_t const width;
    int32_t const height;


    Detector detector(commandlineArguments["cfg-file"], commandlineArguments["weight-file"]);

    int32_t const netWidth = get_net_width();
    int32_t const netHeight = get_net_height();

    float const widthRatio = static_cast<float>(width / netWidth);
    float const heightRatio = static_cast<float>(height / netheight);


    // Get image in
    // Resize, remove alpha channel, save in BGR (?)
    // Start threads
    //   Detect
    //   Calculate distance (scale up detections, check in depth image)
    //   Output data

    image_t *frame = nullptr;// = detector.mat_to_image_resize(detection_data.cap_frame);

    std::vector<bbox_t> result = detector.detect(*frame, 0.2f, true);
    result = detector.tracking_id(result, true, 5, 40);
 
    // If ZED: result_vec = get_3d_coordinates(result_vec, detection_data.zed_cloud);

    retCode = 0;
  }
  return retCode;
}
