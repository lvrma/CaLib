#include "calib.hpp"

int main() {
  cv::Mat cam, dst;
  cv::Size res;

  CaLib::GetInstance().CalibrateFromLiveWebcam(22.0f, cam, dst, res, 5);
  CaLib::GetInstance().CalibrateFromVideoFile("../rsc/s8.mp4", 22.0f, cam, dst,
                                              res, 30, 10);
  CaLib::GetInstance().WriteCalibration("../rsc/test.yml", cam, dst, res);
  CaLib::GetInstance().LoadCalibration("../rsc/test.yml", cam, dst, res);

  return 0;
}