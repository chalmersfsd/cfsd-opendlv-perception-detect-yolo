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

#include <X11/Xlib.h>

#include <yolo_v2_class.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "birdview-perception.hpp"

static uint8_t getPixelExtendArgb(char *img, uint32_t w, uint32_t h, int32_t x,
    int32_t y, uint32_t c)
{
    if (x < 0 || x >= static_cast<int32_t>(w) || y < 0
        || y >= static_cast<int32_t>(h)) {
      return 0.0f;
    }
    return static_cast<uint8_t>(img[y * w * 4 + x * 4 + c]);
}

static float bilinearInterpolationArgb(char *img, uint32_t w, uint32_t h,
    float x, float y, uint32_t c)
{
  uint32_t ix = static_cast<uint32_t>(floorf(x));
  uint32_t iy = static_cast<uint32_t>(floorf(y));

  float dx = x - ix;
  float dy = y - iy;

  return (1 - dy) * (1 - dx) * getPixelExtendArgb(img, w, h, ix, iy, c)
    + dy * (1 - dx) * getPixelExtendArgb(img, w, h, ix, iy + 1, c)
    + (1 - dy) * dx * getPixelExtendArgb(img, w, h, ix + 1, iy, c)
    + dy * dx * getPixelExtendArgb(img, w, h, ix + 1, iy + 1, c);
}

static void resizeArgbToYoloImg(char *imgSrc, float *imgDst, uint32_t wSrc,
    uint32_t hSrc, uint32_t wDst, uint32_t hDst, float wRatio, float hRatio,
    bool interpolate)
{
  for (uint32_t c = 0; c < 3; ++c) {
    for (uint32_t j = 0; j < hDst; ++j) {
      for (uint32_t i = 0; i < wDst; ++i) {
        float v;
        if (interpolate) {
          v = bilinearInterpolationArgb(imgSrc, wSrc, hSrc, i * wRatio,
              j * hRatio, 2 - c);
        } else {
          v = getPixelExtendArgb(imgSrc, wSrc, hSrc,
            static_cast<uint32_t>(i * wRatio),
            static_cast<uint32_t>(j * hRatio), 2 - c);
        }
        imgDst[c * wDst * hDst + j * wDst + i] = v / 255.0f;
      }
    }
  }
}

static void drawBoxArgb(char *img, uint32_t width, uint32_t i0, uint32_t j0,
    uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t i1 = i0 + w;
  uint32_t j1 = j0 + h;
  for (uint32_t i = i0; i <= i1; ++i) {
    img[(i + j0 * width) * 4 + 2] = r;
    img[(i + j1 * width) * 4 + 2] = r;

    img[(i + j0 * width) * 4 + 1] = g;
    img[(i + j1 * width) * 4 + 1] = g;

    img[(i + j0 * width) * 4] = b;
    img[(i + j1 * width) * 4] = b;
  }
  for (uint32_t j = j0; j <= j1; ++j) {
    img[(i0 + j * width) * 4 + 2] = r;
    img[(i1 + j * width) * 4 + 2] = r;

    img[(i0 + j * width) * 4 + 1] = g;
    img[(i1 + j * width) * 4 + 1] = g;

    img[(i0 + j * width) * 4] = b;
    img[(i1 + j * width) * 4] = b;
  }
}

void getDepthData(float *depthConfData, float *depthData, bboxConf_t &detection, uint32_t width, bool verbose = 0)
{
  uint32_t best_idx = 0;
  float best_value = 0.0f;

  for (uint16_t line = detection.y; line <= detection.y + detection.h/2; line++)
  {
    for (uint32_t idx = line * width + detection.x - detection.w/2; idx <= line * width + detection.x + detection.w/2; idx++)
    {
      if (depthConfData[idx] > best_value)
      {
        best_value = depthConfData[idx];
        best_idx = idx;
      }
    }
  }
  detection.depthConfidence = best_value;
  detection.x_3d = depthData[(best_idx * 4)];
  detection.y_3d = depthData[(best_idx * 4) + 1];
  detection.z_3d = depthData[(best_idx * 4) + 2];
  if (verbose)
  {
    std::cout << "Taking depth data from position [" << best_idx / width << ", " << best_idx % width
    << "] with boundaries on height [" << detection.y << ", " << detection.y + detection.h/2
    << "] and width [" << detection.x - detection.w/2 << ", " << detection.x + detection.w/2
    << "]" << std::endl;
  }
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("cfg-file")) ||
      (0 == commandlineArguments.count("weight-file")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")) ||
      (0 == commandlineArguments.count("camera"))  )
  {
    std::cerr << argv[0] << " detects and classifies objects in a camera feed "
      << "by using Yolo." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " [--verbose]" << std::endl;
    std::cerr << "     --cid: OD4 session conference id" << std::endl;
    std::cerr << "     --cfg-file: Yolo configuration file" << std::endl;
    std::cerr << "     --weight-file: Yolo weight file" << std::endl;
    std::cerr << "     --name: name of the shared memory for the ARGB and XYZ"
      << "formatted image (default: video0)" << std::endl;
    std::cerr << "     --width: the width of the images " << std::endl;
    std::cerr << "     --height: the height of the images " << std::endl;
    std::cerr << "     --camera: on car: '0', in office: '1' " << std::endl;
    std::cerr << "     --id: sender id of output messages" << std::endl;
    std::cerr << "     --verbose: prints diagnostics data to screen"
      << std::endl;
    std::cerr << "Example: " << argv[0] << " --cfg-file=yolo.cfg "
      << "--weight-file=yolo.weight --width=1280 --height=720 --camera=0 [--name=video0] "
      << "[--name-depth=video0-depth] [--id=0] [--verbose]" << std::endl;
  } else
  {
    std::string const name{(commandlineArguments["name"].size() != 0) ?
      commandlineArguments["name"] : "video0"};
    std::string const nameArgb{name + ".argb"};
    std::string const nameXyz{name + ".xyz"};
    std::string const nameDepthConf{name + ".dconf"};

    uint32_t const width{static_cast<uint32_t>(
        std::stoi(commandlineArguments["width"]))};
    uint32_t const height{static_cast<uint32_t>(
        std::stoi(commandlineArguments["height"]))};
    uint32_t const camera{static_cast<uint32_t>(
        std::stoi(commandlineArguments["camera"]))};
    uint32_t const id{(commandlineArguments["id"].size() != 0) ?
      static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    float const halfWidth{static_cast<float>(width) / 2.0f};

    //Set up camera parameters
    cameraPara camPara = setupCameraPara(height,camera);

    Display* display{nullptr};
    Visual* visual{nullptr};
    Window window{0};
    XImage* ximage{nullptr};

    Detector detector(commandlineArguments["cfg-file"],
        commandlineArguments["weight-file"]);

    std::cout << "Connecting to shared memory " << nameArgb << std::endl;
    std::unique_ptr<cluon::SharedMemory> shmArgb{
      new cluon::SharedMemory{nameArgb}};
    if (shmArgb && shmArgb->valid()) {
      std::clog << argv[0] << ": Attached to shared ARGB memory '"
        << shmArgb->name() << " (" << shmArgb->size()
        << " bytes)." << std::endl;
    }

    bool hasXyzData{false};

    std::cout << "Connecting to shared memory " << nameXyz << std::endl;
    std::unique_ptr<cluon::SharedMemory> shmXyz{
      new cluon::SharedMemory{nameXyz}};
    if (shmXyz && shmXyz->valid()) {
      hasXyzData = true;
      std::clog << argv[0] << ": Attached to shared depth memory '"
        << shmXyz->name() << " (" << shmXyz->size()
        << " bytes)." << std::endl;
    }

    std::cout << "Connecting to shared memory " << nameDepthConf << std::endl;
    std::unique_ptr<cluon::SharedMemory> shmDepthConf{
      new cluon::SharedMemory{nameDepthConf}};
    if (shmDepthConf && shmDepthConf->valid()) {
      std::clog << argv[0] << ": Attached to shared depth memory '"
        << shmDepthConf->name() << " (" << shmDepthConf->size()
        << " bytes)." << std::endl;
    }

    image_t yoloImg;
    yoloImg.w = detector.get_net_width();
    yoloImg.h = detector.get_net_height();
    yoloImg.c = 3;
    yoloImg.data = new float[yoloImg.w * yoloImg.h * yoloImg.c];

    char *verboseImg{nullptr};
    if (verbose) {
      verboseImg = new char[width * height * 4];
    }

    if (verbose) {
      display = XOpenDisplay(NULL);
      visual = DefaultVisual(display, 0);
      window = XCreateSimpleWindow(display, RootWindow(display, 0), 0, 0,
          width, height, 1, 0, 0);
      shmArgb->lock();
      {
        ximage = XCreateImage(display, visual, 24, ZPixmap, 0, verboseImg,
            width, height, 32, 0);
      }
      shmArgb->unlock();

      XMapWindow(display, window);
    }

    float const widthRatio = static_cast<float>(width) / yoloImg.w;
    float const heightRatio = static_cast<float>(height) / yoloImg.h;

    cluon::OD4Session od4{static_cast<uint16_t>(
        std::stoi(commandlineArguments["cid"]))};

    uint64_t frameCount{0};
    while (od4.isRunning())
    {
      cluon::data::TimeStamp t0 = cluon::time::now();
      shmArgb->wait();
      shmArgb->lock();

      if (verbose) {
        memcpy(verboseImg, shmArgb->data(), shmArgb->size());
      }
      resizeArgbToYoloImg(shmArgb->data(), yoloImg.data, width, height,
          yoloImg.w, yoloImg.h, widthRatio, heightRatio, false);

      shmArgb->unlock();

      std::vector<bbox_t> temp = detector.detect(yoloImg, 0.5f, true);

      for (auto &detection : temp) {
        detection.x = static_cast<uint32_t>(widthRatio * detection.x);
        detection.w = static_cast<uint32_t>(widthRatio * detection.w);
        detection.y = static_cast<uint32_t>(heightRatio * detection.y);
        detection.h = static_cast<uint32_t>(heightRatio * detection.h);
      }

      temp = detector.tracking_id(temp, true, 5, 40);
      std::vector<bboxConf_t> detections;
      for (auto &detection : temp) { detections.push_back(detection); }

      if (hasXyzData) {
        shmXyz->wait();
        shmXyz->lock();
        shmDepthConf->lock();
        {
          char *depthData = shmXyz->data();
          char* depthConfData = shmDepthConf->data();
          for (auto &detection : detections) {
            getDepthData((float*)depthConfData, (float*)depthData, detection, width, verbose);
          }
        }
        shmDepthConf->unlock();
        shmXyz->unlock();
      }


      if (verbose) {
        float fps = 1000000.0f /
          (cluon::time::toMicroseconds(cluon::time::now())
            - cluon::time::toMicroseconds(t0));
        std::cout << "\n====================================================\n";
        std::cout << "Frames per second: " << fps << ", found objects "
          << detections.size() << std::endl;
      }

      if (detections.size() > 0)
      {
        cluon::data::TimeStamp ts{cluon::time::now()};

        opendlv::logic::perception::ObjectFrameStart startMsg;
        startMsg.objectFrameId(frameCount);
        od4.send(startMsg, ts, id);

        uint32_t n = 0;
        for (auto &detection : detections)
        {
          uint32_t const objectId = n++ * 1000 + detection.track_id;
          opendlv::logic::perception::ObjectPosition conePos;
          opendlv::logic::perception::ObjectType coneType;

          coneType.type(static_cast<uint32_t>(detection.obj_id));
          coneType.objectId(objectId);
          od4.send(coneType, ts, id);

          conePos = getDistance(camPara, detection, verbose);
          conePos.objectId(objectId);
          od4.send(conePos, ts, id);

          opendlv::logic::perception::ObjectDirection coneDirection;
          coneDirection.objectId(objectId);
          coneDirection.azimuthAngle(halfWidth - (detection.x +
            static_cast<float>(detection.w) / 2.0f));
          coneDirection.zenithAngle(static_cast<float>(height) - detection.y);
          od4.send(coneDirection, ts, id);

          opendlv::logic::perception::ObjectAngularBlob coneAngularBlob;
          coneAngularBlob.objectId(objectId);
          coneAngularBlob.width(detection.w);
          coneAngularBlob.height(detection.h);
          od4.send(coneAngularBlob, ts, id);

          if (verbose)
          {
            std::string coneName[4] = {"Yellow", "Blue  ", "Red   ", "BigRed"};
            std::cout << "  ...object-id=" << objectId << " i=" << detection.x
              << ", j=" << detection.y << ", w=" << detection.w << ", h="
              << detection.h << ", prob=" << detection.prob << ", Color="
              << coneName[detection.obj_id] << ", tack id=" << detection.track_id
              << ", frame=" << frameCount << ", x="
              << detection.z_3d << ", y=" << -detection.x_3d << ", z="
              << detection.y_3d << " Cone(x,y) = " << conePos.x()<<" , "<< conePos.y() << std::endl;

            std::array<std::array<uint8_t, 3>, 8> colors{{
              {{255, 255, 0}},
              {{0, 0, 255}},
              {{255, 0, 0}},
              {{0, 255, 0}},
              {{255, 0, 255}},
              {{0, 255, 255}},
              {{255, 255, 255}},
              {{0, 0, 0}}
            }};

            uint32_t const k = detection.obj_id % colors.size();
            drawBoxArgb(verboseImg, width, detection.x, detection.y,
                detection.w, detection.h, colors[k][0], colors[k][1],
                colors[k][2]);

          }
        }
        opendlv::logic::perception::ObjectFrameEnd endMsg;
        endMsg.objectFrameId(frameCount);
        od4.send(endMsg, cluon::time::now(), id);
        frameCount++;
      }
      if(verbose)
      {
        XPutImage(display, window, DefaultGC(display, 0), ximage, 0, 0, 0, 0,
                  width, height);
      }
    }

    delete[] yoloImg.data;
    delete[] verboseImg;

    retCode = 0;
  }
  return retCode;
}
