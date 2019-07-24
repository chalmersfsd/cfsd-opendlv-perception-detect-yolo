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

#include "birdview-perception.hpp"
#include <iostream>

//Define intrinsic camera parameters inside matrix (left/right)
static double mtxLeftVGA_Office[3][3] = {
  {349.833, 0, 316.973},
  {0, 349.833, 183.859},
  {0, 0, 1}
};

static double mtxLeftVGA_Car[3][3] = {
  {349.891, 0, 334.352},
  {0, 349.891, 187.937},
  {0, 0, 1}
};

static double mtxLeftHD_Office[3][3] = {
  {699.666, 0, 603.946},
  {0, 699.666, 353.718},
  {0, 0, 1}
};

static double mtxLeftHD_Car[3][3] = {
  {699.783, 0, 637.704},
  {0, 699.783, 360.875},
  {0, 0, 1}
};

static double mtxLeftFHD_Office[3][3] = {
  {1399.33, 0, 890.891},
  {0, 1399.33, 530.436},
  {0, 0, 1}
};

static double mtxLeftFHD_Car[3][3] = {
  {1399.57, 0, 958.407},
  {0, 1399.57, 544.749},
  {0, 0, 1}
};

static double mtxLeft2K_Office[3][3] = {
  {1399.33, 0, 1034.89},
  {0, 1399.33, 611.436},
  {0, 0, 1}
};

static double mtxLeft2K_Car[3][3] = {
  {1399.57, 0, 1102.41},
  {0, 1399.57, 625.749},
  {0, 0, 1}
};

void printMatrix(double (*mtx)[3][3])
{
  std::cout << "=============This is matrix ============" << '\n';
  std::cout << (*mtx)[0][0] << " " << (*mtx)[0][1] << " " << (*mtx)[0][2] << std::endl;
  std::cout << (*mtx)[1][0] << " " << (*mtx)[1][1] << " " << (*mtx)[1][2] << std::endl;
  std::cout << (*mtx)[2][0] << " " << (*mtx)[2][1] << " " << (*mtx)[2][2] << std::endl;
}

cameraPara setupCameraPara(uint32_t height, uint32_t camera){

    cameraPara camPara;
    double (*mtx)[3][3] = nullptr;
    double pixelSize_mm = 0.0;

    switch(camera)  {
      //Camera used in car:
      case 0:
        switch(height) {
          case 1242:
            pixelSize_mm = 0.002;
            mtx = &mtxLeft2K_Car;
            break;
          case 1080:
            pixelSize_mm = 0.002;
            mtx = &mtxLeftFHD_Car;
            break;
          case 720:
            pixelSize_mm = 0.004;
            mtx = &mtxLeftHD_Car;
            break;
          case 376:
            pixelSize_mm = 0.008;
            mtx = &mtxLeftVGA_Car;
            break;
          default:
            std::cout << "Wrong camera height" << std::endl;
            break;
        }
        break;
      //Camera used in office:
      case 1:
        switch(height) {
          case 1242:
            pixelSize_mm = 0.002;
            mtx = &mtxLeft2K_Office;
            break;
          case 1080:
            pixelSize_mm = 0.002;
            mtx = &mtxLeftFHD_Office;
            break;
          case 720:
            pixelSize_mm = 0.004;
            mtx = &mtxLeftHD_Office;
            break;
          case 376:
            pixelSize_mm = 0.008;
            mtx = &mtxLeftVGA_Office;
            break;
          default:
            std::cout << "Wrong camera height" << std::endl;
            break;
        }
        break;
      default:
        std::cout << "Wrong camera type" << std::endl;
        break;
    }
  camPara.focLength_pix = (*mtx)[0][0];
  camPara.cx = (*mtx)[0][2];
  camPara.sensHeight_pix = height;
  camPara.focLength_mm = camPara.focLength_pix * pixelSize_mm;
  camPara.sensHeight_mm = camPara.sensHeight_pix * pixelSize_mm;

  return camPara;
}

opendlv::logic::perception::ObjectPosition getDistance(cameraPara camPara, bboxConf_t &detection, bool verbose)
{

  opendlv::logic::perception::ObjectPosition conePos;
  conePos.x(0.0);
  conePos.y(0.0);
  if ( (detection.depthConfidence > depthConfidenceThreshold || detection.depthConfidence > detection.prob)
        && !std::isnan(detection.z_3d))
  {
    conePos.x(detection.z_3d);
    if (verbose) std::cout << "Taking distance from stereo with confidence " << detection.depthConfidence << std::endl;
  }
  else
  {
    if (verbose) std::cout << "Taking distance from cone size with confidence" << (int)(detection.prob*100) << std::endl;
    double realObjHeight_m = 0.0;
    double objHeightSensor_mm = 0.0;

    switch(detection.obj_id) {
      case 0:
      case 1:
      case 2:
        //Normal cone orange, yellow, blue
        realObjHeight_m = 0.325;
        break;
      case 3:
        //Big orange cone
        realObjHeight_m = 0.505;
        break;
      default:
        std::cout<<"Wrong object id: " << detection.obj_id << std::endl;
        break;
    }
    objHeightSensor_mm = camPara.sensHeight_mm * detection.h / camPara.sensHeight_pix;
    conePos.x( realObjHeight_m * camPara.focLength_mm / objHeightSensor_mm );

    if((conePos.x() < depthDistanceThreshold) && !std::isnan(detection.z_3d))
    {
      conePos.x(detection.z_3d);
      if (verbose) std::cout << "Overwriting distance from cone size because of threshold distance" << (int)(detection.prob*100) << std::endl;
    }
  }

  conePos.y( -(conePos.x()*detection.x - conePos.x()*camPara.cx) / camPara.focLength_pix );

  return conePos;
}
