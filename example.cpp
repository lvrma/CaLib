#include "calib.hpp"

int main() {

	cv::Mat cam, dst;
	cv::Size res;

	//CaLib::GetInstance().CalibrateFromLiveWebcam(22.0f, cam, dst, res, 5);
	//CaLib::GetInstance().CalibrateFromVideoFile(22.0f, cam, dst, res, "../rsc/s8.mp4", 30, 10);
	//CaLib::GetInstance().WriteCalibration("../rsc/s8", cam, dst, res);
	//CaLib::GetInstance().LoadCalibration("../rsc/s8.yml", cam, dst, res);

	return 0;
}