#ifndef GUPPY_VISION_CAMERA_UTILS_HPP
#define GUPPY_VISION_CAMERA_UTILS_HPP

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <iostream>
#include <sstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

enum StreamMode {
  STREAM_MODE_TELEDYNE_GIGE_VISION,  // windows
  STREAM_MODE_PGRLWF,                // legacy windows
  STREAM_MODE_SOCKET,                // macos and linux
};

namespace camera_manager {

class CameraManager {
 public:
  CameraManager();
  ~CameraManager();

  SystemPtr system;
  CameraList camList;

  static int setStreamMode(CameraPtr pCam, StreamMode mode);

  static void takePicture(CameraPtr pCam);
};

}  // namespace camera_manager

#endif  // GUPPY_VISION_CAMERA_UTILS_HPP