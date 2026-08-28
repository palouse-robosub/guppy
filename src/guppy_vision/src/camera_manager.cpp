#include "camera_manager.hpp"

using namespace camera_manager;

CameraManager::CameraManager() {
  system = System::GetInstance();
  camList = system->GetCameras();
}

CameraManager::~CameraManager() {
  cout << "Attempting graceful exit." << endl;

  camList.Clear();
  system->ReleaseInstance();

  cout << "Successfully exited gracefully." << endl;
}

void CameraManager::takePicture(CameraPtr pCam) {
  INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
  pCam->Init();
  INodeMap& nodeMap = pCam->GetNodeMap();

  cout << "Acquiring single image from " << nodeMapTLDevice.GetDeviceName()
       << endl;

  pCam->BeginAcquisition();

  gcstring deviceSerialNumber("");
  CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
  deviceSerialNumber = ptrStringSerial->GetValue();

  ImageProcessor processor;
  processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

  ImagePtr pResultImage = pCam->GetNextImage(1000);

  const size_t width = pResultImage->GetWidth();

  const size_t height = pResultImage->GetHeight();

  cout << "Grabbed image: width = " << width << ", height = " << height << endl;

  // ImagePtr convertedImage = processor.Convert(pResultImage,
  // PixelFormat_BayerRG8); convertedImage->Save("saved.png");

  std::cout << pResultImage->GetData() << std::endl;

  pResultImage->Release();
  pCam->EndAcquisition();
}

int CameraManager::setStreamMode(CameraPtr pCam, StreamMode mode) {
  int result = 0;

  // Retrieve Stream nodemap
  const INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();

  // The node "StreamMode" is only available for GEV cameras.
  // Skip setting stream mode if the node is inaccessible.
  const CEnumerationPtr ptrStreamMode = sNodeMap.GetNode("StreamMode");
  if (!IsReadable(ptrStreamMode) || !IsWritable(ptrStreamMode)) {
    return 0;
  }

  gcstring streamMode;
  switch (mode) {
    case STREAM_MODE_PGRLWF:
      streamMode = "LWF";
      break;
    case STREAM_MODE_SOCKET:
      streamMode = "Socket";
      break;
    case STREAM_MODE_TELEDYNE_GIGE_VISION:
    default:
      streamMode = "TeledyneGigeVision";
  }

  // Retrieve the desired entry node from the enumeration node
  const CEnumEntryPtr ptrStreamModeCustom =
      ptrStreamMode->GetEntryByName(streamMode);
  if (!IsReadable(ptrStreamModeCustom)) {
    // Failed to get custom node
    cout << "Stream mode " + streamMode + " not available.  Aborting..."
         << endl;
    return -1;
  }
  // Retrieve the integer value from the entry node
  const int64_t streamModeCustom = ptrStreamModeCustom->GetValue();

  // Set integer as new value for enumeration node
  ptrStreamMode->SetIntValue(streamModeCustom);

  // Print out the current stream mode
  cout << endl
       << "Stream Mode set to " +
              ptrStreamMode->GetCurrentEntry()->GetSymbolic()
       << "..." << endl;

  return 0;
}

int main(int argc, char* argv[]) {
  CameraManager manager;

  const unsigned int numCameras = manager.camList.GetSize();

  cout << "Discovered " << numCameras << " cameras." << endl;

  CameraManager::takePicture(manager.camList[0]);

  cout << "Exiting." << endl;

  return 0;
}