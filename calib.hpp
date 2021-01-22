#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>


// TODO: Log tags, config VS for uniform code pattern, pretty TUI, document,
// clarify that path can also be added to filename, hardcoded or relative,
// make sure var names are uniform, write README, calib overloads for fx, cx,
// make sure in and out names and const are fine, make everything 80 col
// internal methods for building mat from fx and cx (it is doubled),
// make extracting cx fx from mat uniform (it is doubled)
// add cap.open checks, cap.release, and quit/cancel button,
// name check module is repeated, make method
// replace tabs with spaces for uniformity on git
class CaLib {
public:
	static CaLib& GetInstance() {
		static CaLib instance;
		return instance;
	}

	CaLib(CaLib const&) = delete;
	void operator=(CaLib const&) = delete;

	void CalibrateFromVideoFile(const float chess_square_size,
							cv::Mat& cam_matrix_out,
							cv::Mat& dist_coeff_out,
							cv::Size &camera_resolution_out,
							const std::string &filename,
							const int samples = 30,
							const int wait_per_frame_in_ms = 5) {

		cv::VideoCapture cap(filename);
		this->Calibrate(cap, chess_square_size, cam_matrix_out, dist_coeff_out,
			camera_resolution_out,
						samples,
			wait_per_frame_in_ms);
	}

	void CalibrateFromLiveWebcam(const float chess_square_size, 
								 cv::Mat &cam_matrix_out,
								 cv::Mat &dist_coeff_out,
		cv::Size& camera_resolution_out,
								 const int samples = 30,
								 const int webcam_id = 0) {

		cv::VideoCapture cap(webcam_id);
		this->Calibrate(cap, chess_square_size, cam_matrix_out, dist_coeff_out,
			camera_resolution_out,
						samples);
	}

	void LoadCalibration(const std::string& camera_name,
		cv::Point2d& focal_length_in_px_out,
		cv::Point2d &principal_point_out,
		cv::Mat& dist_coeff_out,
		cv::Size& camera_res_out = dummy_size) {

		double w, h, fx, fy, cx, cy;
		cv::Mat dist_coeffs;

		std::string tmp_name, extension;
		if (camera_name.length() < 4) extension = "";
		else extension = camera_name.substr(camera_name.length() - 4, 
			camera_name.length() - 2);
		if (extension != ".yml") tmp_name = camera_name + ".yml";
		else tmp_name = camera_name;

		cv::FileStorage file_stream(tmp_name, cv::FileStorage::READ);

		file_stream["width"] >> w;
		file_stream["height"] >> h;
		file_stream["fx"] >> fx;
		file_stream["fy"] >> fy;
		file_stream["cx"] >> cx;
		file_stream["cy"] >> cy;
		file_stream["distortion_coefficients"] >> dist_coeffs;

		file_stream.release();

		bool non_zero = false;
		for (int i = 0; i < dist_coeffs.rows; i++)
			for (int j = 0; j < dist_coeffs.cols; j++)
				if (dist_coeffs.at<double>(i, j) != 0)
					non_zero = true;

		if (!non_zero) dist_coeffs = cv::Mat();
		
		dist_coeff_out = dist_coeffs;
		camera_res_out = cv::Size(w, h);
		focal_length_in_px_out = cv::Point2d(fx, fy);
		principal_point_out = cv::Point2d(cx, cy);

		std::cout << "Calibration " << tmp_name << " loaded." << std::endl;
		std::cout << "Focal length: " << focal_length_in_px_out << std::endl;
		std::cout << "Principal Point: " << principal_point_out << std::endl;
		std::cout << "Distortion Coefficients: " << dist_coeff_out << std::endl;
		std::cout << "Camera Resolution: " << camera_res_out << std::endl;
	}

	void LoadCalibration(const std::string &filename,
						 cv::Mat& cam_matrix_out,
						 cv::Mat& dist_coeff_out,
						 cv::Size& camera_res_out = dummy_size) {
		
		cv::Point2d f, c;
		this->LoadCalibration(filename, f, c, dist_coeff_out, camera_res_out);

		cam_matrix_out = (cv::Mat_<double>(3, 3) << f.x, 0.0, c.x, 
													0.0, f.y, c.y, 
													0.0, 0.0, 1.0);
	}

	void GetFocalLength(const cv::Mat& cam_matrix,
		cv::Point2d& focal_length_in_px_out) {
		focal_length_in_px_out = cv::Point2d(cam_matrix.at<double>(0, 0),
			cam_matrix.at<double>(1, 1));
	}

	void GetPrincipalPoint(const cv::Mat& cam_matrix, 
		cv::Point2d& principal_point_out) {
		principal_point_out = cv::Point2d(cam_matrix.at<double>(0, 2),
			cam_matrix.at<double>(1, 2));
	}

	void BuildCameraMatrix(const cv::Point2d& focal_length_in_px,
		const cv::Point2d& principal_point, cv::Mat &cam_matrix_out) {
		cam_matrix_out = (cv::Mat_<double>(3, 3) << focal_length_in_px.x, 0.0, principal_point.x,
			0.0, focal_length_in_px.y, principal_point.y,
			0.0, 0.0, 1.0);
	}

	void WriteCalibration(const std::string& camera_name,
						  const cv::Point2d& focal_length_in_px,
						  const cv::Point2d& principal_point,
						  const cv::Mat& dist_coeff,
						  const cv::Size& camera_res) {

		std::string tmp_name, extension;
		if (camera_name.length() < 4) extension = "";
		else extension = camera_name.substr(camera_name.length() - 4,
			camera_name.length() - 2);
		if (extension != ".yml") tmp_name = camera_name + ".yml";
		else tmp_name = camera_name;

		cv::FileStorage file_stream(tmp_name,
									cv::FileStorage::WRITE);

		file_stream << "width" << camera_res.width;
		file_stream << "height" << camera_res.height;
		file_stream << "fx" << focal_length_in_px.x;
		file_stream << "fy" << focal_length_in_px.y;
		file_stream << "cx" << principal_point.x;
		file_stream << "cy" << principal_point.y;
		file_stream << "distortion_coefficients" << dist_coeff;

		file_stream.release();

		std::cout << "Calibration written to file." << std::endl;
	}

	void WriteCalibration(const std::string& camera_name,
							const cv::Mat& cam_matrix,
							const cv::Mat& dist_coeff,
							const cv::Size& camera_res) {

		cv::Point2d f, c;
		f = cv::Point2d(cam_matrix.at<double>(0, 0), 
			cam_matrix.at<double>(1, 1));
		c = cv::Point2d(cam_matrix.at<double>(0, 2), 
			cam_matrix.at<double>(1, 2));

		this->WriteCalibration(camera_name, f, c, dist_coeff, camera_res);
	}

private:
	const cv::Size CHESS_BOARD = cv::Size(6, 9);
	const int CALIB_FLAGS = cv::CALIB_CB_ADAPTIVE_THRESH | 
							cv::CALIB_CB_NORMALIZE_IMAGE;
	static cv::Size dummy_size;

	CaLib() = default;
	~CaLib() = default;

	void Calibrate(cv::VideoCapture& cap,
		const float chess_square_size,
		cv::Mat& cam_matrix_out,
		cv::Mat& dist_coeff_out,
		cv::Size& camera_resolution_out,
		const int samples = 30,
		const int wait_per_frame_in_ms = 5) {

		std::vector<std::vector<cv::Point3f>> src_pts(1);
		std::vector<std::vector<cv::Point2f>> dst_pts;
		std::vector<cv::Point2f> tmp_pts;

		cv::Mat frame = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
		std::vector<cv::Mat> tvecs, rvecs;
		dist_coeff_out = cv::Mat::zeros(8, 1, CV_64F);

		std::cout << "To capture a frame, make sure the camera window is ";
		std::cout << "selected, wait for the chessboard overlay, and ";
		std::cout << "press SPACE to capture." << std::endl;
		std::cout << "If the frame was successfully captured, a line will be ";
		std::cout << "printed in this console." << std::endl;

		std::cout << "For best results, try to vary translation (especially ";
		std::cout << "depth) and rotation as much as possible." << std::endl;
		std::cout << "Strating video feed to begin calibration..." << std::endl;

		if (!cap.read(frame))
			std::cout << ("ERROR: No video feed.") << std::endl;

		int count = 0;
		int delay = 10;
		while (count < samples && cap.read(frame)) {
			bool found = false;
			if (delay == 0) {
				found = cv::findChessboardCorners(frame, CHESS_BOARD, tmp_pts,
					CALIB_FLAGS);
				delay = 10;
			}

			if (found) {
				cv::drawChessboardCorners(frame, CHESS_BOARD, tmp_pts, found);
				delay = 0;
			}
			else delay--;

			cv::imshow("CaLib Camera Calibration", frame);
			char key = cv::waitKey(wait_per_frame_in_ms);

			if (key == ' ') {
				if (found) dst_pts.push_back(tmp_pts);
				count++;
				std::cout << std::to_string(samples - count);
				std::cout << " frames to go." << std::endl;
			}

			tmp_pts.clear();
		}

		std::cout << "Computing Calibration, please wait." << std::endl;
		std::cout << "Depending on the amount of samples, ";
		std::cout << "this might take a few minutes." << std::endl;

		for (int i = 0; i < this->CHESS_BOARD.height; i++)
			for (int j = 0; j < this->CHESS_BOARD.width; j++)
				src_pts[0].push_back(cv::Point3f(j * chess_square_size,
					i * chess_square_size, 0));

		src_pts.resize(dst_pts.size(), src_pts[0]);
		cv::calibrateCamera(src_pts, dst_pts, this->CHESS_BOARD,
			cam_matrix_out, dist_coeff_out, rvecs, tvecs);

		camera_resolution_out = frame.size();
	}
};