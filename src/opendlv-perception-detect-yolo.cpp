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

static char getPixelExtendArgb(char *img, uint32_t w, uint32_t h, int32_t x, 
    int32_t y, uint32_t c)
{
    if (x < 0 || x >= static_cast<int32_t>(w) || y < 0 
        || y >= static_cast<int32_t>(h)) {
      return 0.0f;
    }
    return img[y * w * 4 + x * 4 + c];
}

static float bilinearInterpolationArgb(char *img, uint32_t w, uint32_t h, 
    float x, float y, uint32_t c)
{
    uint32_t ix = static_cast<uint32_t>(floorf(x));
    uint32_t iy = static_cast<uint32_t>(floorf(y));

    float dx = x - ix;
    float dy = y - iy;

    return (1 - dy) * (1 - dx) * getPixelExtendArgb(img, w, h, ix, iy, c) +
      dy * (1 - dx) * getPixelExtendArgb(img, w, h, ix, iy + 1, c) +
      (1 - dy) * dx * getPixelExtendArgb(img, w, h, ix + 1, iy, c) +
      dy * dx * getPixelExtendArgb(img, w, h, ix + 1, iy + 1, c);
}

static void resizeArgbToYoloImg(char *imgSrc, float *imgDst, uint32_t wSrc,
    uint32_t hSrc, uint32_t wDst, uint32_t hDst, float wRatio, float hRatio)
{
  for (uint32_t c = 0; c < 3; ++c) {
    for (uint32_t j = 0; j < hDst; ++j) {
      for (uint32_t i = 0; i < wDst; ++i) {
        float v = bilinearInterpolationArgb(imgSrc, wSrc, hSrc, i * wRatio, 
            j * hRatio, c);
        imgDst[c * wDst * hDst + j * wDst + i] = v / 255.0f;
      }
    }
  }
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("cfg-file")) ||
      (0 == commandlineArguments.count("weight-file")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")) ) {
    std::cerr << argv[0] << " detects and classifies objects in a camera feed "
      << "by using Yolo." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " [--verbose]" << std::endl;
    std::cerr << "         --cid: OD4 session conference id" << std::endl;
    std::cerr << "         --cfg-file: Yolo configuration file" << std::endl;
    std::cerr << "         --weight-file: Yolo weight file" << std::endl;
    std::cerr << "         --name: name of the shared memory for the ARGB "
      << "formatted image (default: video0)" << std::endl;
    std::cerr << "         --name-depth: name of the shared memory for the "
      << "depth XYZ formatted image " << std::endl;
    std::cerr << "         --width: the width of the images " << std::endl;
    std::cerr << "         --width: the height of the images " << std::endl;
    std::cerr << "         --id: sender id of output messages" << std::endl;
    std::cerr << "         --verbose: prints diagnostics data to screen" 
      << std::endl;
    std::cerr << "Example: " << argv[0] << " --cfg-file=yolo.cfg "
      << "--weight-file=yolo.weight --width=1270 --height=720 [--name=video0] "
      << "[--name-depth=video0-depth] [--id=0] [--verbose]" << std::endl;
  } else {
    std::string const nameArgb{(commandlineArguments["name"].size() != 0) ? 
      commandlineArguments["name"] : "video0"};
    std::string const nameDepth{
      (commandlineArguments["name-depth"].size() != 0) ? 
      commandlineArguments["name-depth"] : ""};
    uint32_t const width{static_cast<uint32_t>(
        std::stoi(commandlineArguments["width"]))};
    uint32_t const height{static_cast<uint32_t>(
        std::stoi(commandlineArguments["height"]))};
    uint32_t const id{(commandlineArguments["id"].size() != 0) ?
      static_cast<uint32_t>(std::stoi(commandlineArguments["height"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    Detector detector(commandlineArguments["cfg-file"],
        commandlineArguments["weight-file"]);

    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{nameArgb}};
    if (shmArgb && shmArgb->valid()) {
      std::clog << argv[0] << ": Attached to shared ARGB memory '" 
        << shmArgb->name() << " (" << shmArgb->size() 
        << " bytes)." << std::endl;
    
      bool hasDepth{false};
      std::unique_ptr<cluon::SharedMemory> shmDepth;
      if (!nameDepth.empty()) {
        shmDepth.reset(new cluon::SharedMemory{nameDepth});
        if (shmDepth && shmDepth->valid()) {
          std::clog << argv[0] << ": Attached to shared depth memory '" 
            << shmDepth->name() << " (" << shmDepth->size() 
            << " bytes)." << std::endl;
          hasDepth = true;
        }
      }

      image_t yoloImg;
      yoloImg.w = detector.get_net_width();
      yoloImg.h = detector.get_net_height();
      yoloImg.c = 3;
      yoloImg.data = new float[yoloImg.w * yoloImg.h * 3];
    
      float const widthRatio = static_cast<float>(width / yoloImg.w);
      float const heightRatio = static_cast<float>(height / yoloImg.h);

      cluon::OD4Session od4{static_cast<uint16_t>(
          std::stoi(commandlineArguments["cid"]))};

      while (od4.isRunning()) {
        shmArgb->wait();
        shmArgb->lock();
        
        {
          resizeArgbToYoloImg(shmArgb->data(), yoloImg.data, width, height,
              yoloImg.w, yoloImg.h, widthRatio, heightRatio);
        }
        shmArgb->unlock();

        std::vector<bbox_t> detections = detector.detect(yoloImg, 0.2f, true);

        for (auto &detection : detections) {
          detection.x = static_cast<uint32_t>(widthRatio * detection.x);
          detection.w = static_cast<uint32_t>(widthRatio * detection.w);
          detection.y = static_cast<uint32_t>(heightRatio * detection.y);
          detection.h = static_cast<uint32_t>(heightRatio * detection.h);
        }
        
        detections = detector.tracking_id(detections, true, 5, 40);

        if (hasDepth) {
          shmDepth->wait();
          shmDepth->lock();
          {
            char *depthData = shmDepth->data();
            for (auto &detection : detections) {
              uint32_t i = static_cast<uint32_t>(
                  detection.x + detection.w * 0.5f);
              uint32_t j = static_cast<uint32_t>(
                  detection.y + detection.h * 0.5f);
              
              memcpy(&detection.x_3d, depthData + (j * width * 16 + i * 16), 4);
              memcpy(&detection.y_3d,
                  depthData + (j * width * 16 + i * 16 + 4), 4);
              memcpy(&detection.z_3d,
                  depthData + (j * width * 16 + i * 16 + 8), 4);
            }
          }
          shmDepth->unlock();
        }
    
        //   Output data
        if (verbose) {
          std::cout << "Found objects: " << detections.size() << std::endl;
          for (auto &detection : detections) {
            std::cout << "  ... i=" << detection.x << ", j=" 
              << detection.y << ", w=" << detection.w << ", h=" << detection.h 
              << ", prob=" << detection.prob << ", id=" << detection.obj_id 
              << ", tack id=" << detection.track_id << ", frame=" 
              << detection.frames_counter << ", x=" << detection.x_3d 
              << ", y=" << detection.y_3d << ", z=" << detection.z_3d 
              << std::endl;
          }
        }
      }

      delete[] yoloImg.data;
    }

    retCode = 0;
  }
  return retCode;
}
