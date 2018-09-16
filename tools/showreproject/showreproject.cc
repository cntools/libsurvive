
#include <chrono>
#include <iostream>
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <map>
#include <math.h>
#include <memory>
#include <set>
#include <vector>

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

uint32_t timestamp;

cv::Mat_<cv::Vec3b> img;
cv::Mat_<cv::Vec3b> err[2];

cv::Vec3f flow2rgb(float x, float y, float scale = 1) {
	cv::Mat_<cv::Vec3f> hsv(1, 1);
	hsv(0, 0) = {(float)(atan2(y, x) * 180. / M_PI), 1.0, std::min(1.0f, sqrt(x * x + y * y) * scale)};
	cv::Mat_<cv::Vec3f> bgr(1, 1);
	cv::cvtColor(hsv, bgr, CV_HSV2RGB_FULL);
	return bgr(0, 0) * 255;
}

static void draw_viz(cv::Mat &img, double scale = 1) {
	double size = 50;
	cv::Point2f origin(size, size);
	for (double r = size; r > size / 100.; r -= size / 100.) {
		for (double theta = 0; theta < 2 * M_PI; theta += .01) {
			auto x = cos(theta) * r;
			auto y = sin(theta) * r;
			auto clr = flow2rgb(x, y, scale / size);
			cv::line(img, cv::Point2f(x, y) + origin, origin, clr);
		}
	}
}

double error = 0;
int error_count = 0;

static void redraw(SurviveContext *ctx) {
	int SIZE = 1000;
	int shift = ctx->user_ptr ? 1 : 0;

	auto show_pov = 130. / 180;
	auto fov = 120. / 180. * SIZE / show_pov;

	auto map = [&](const double *a) {
		auto x = a[0] * SIZE / (M_PI) / show_pov + SIZE / 2;
		return cv::Point(x, SIZE - (a[1] * SIZE / (M_PI) + SIZE / 2));
	};

	auto region = img.data ? img(cv::Rect(SIZE * shift, 0, SIZE, SIZE)) : cv::Mat_<cv::Vec3b>();

	if (region.data) {

		region.setTo(cv::Vec3b(0, 0, 0));
		cv::rectangle(region, cv::Point(SIZE / 2 - fov / 2, SIZE / 2 - fov / 2),
					  cv::Point(SIZE / 2 + fov / 2, SIZE / 2 + fov / 2), cv::Vec3b(255, 255, 255));
	}

	// eregion.copyTo(region);

	for (int i = 0; i < ctx->objs_ct; i++) {
		auto so = ctx->objs[i];
		auto scene = &so->activations;
		for (size_t lh = 0; lh < 2; lh++) {
			auto eregion = err[lh](cv::Rect(SIZE * shift, 0, SIZE, SIZE));

			for (size_t sensor = 0; sensor < so->sensor_ct; sensor++) {

				auto ncolor = cv::Vec3b(lh == 0 ? 255 : 0,
										255, // sensor / (double)so->sensor_ct * 255,
										i / (double)ctx->objs_ct * 255);
				auto rcolor = cv::Vec3b(lh == 0 ? 255 : 0,
										128, // sensor / (double)so->sensor_ct * 255,
										i / (double)ctx->objs_ct * 255);

				if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance, timestamp,
														 sensor, lh)) {
					const double *a = scene->angles[sensor][lh];
					// FLT a[2];
					// survive_apply_bsd_calibration(so->ctx, lh, _a, a);

					auto l = scene->lengths[sensor][lh];
					double r = std::max(3., (l[0] + l[1]) / 1000.);

					if (region.data)
						cv::circle(region, map(a), r, ncolor);

					FLT point3d[3];
					FLT out[2];
					ApplyPoseToPoint(point3d, &so->OutPoseIMU, so->sensor_locations + 3 * sensor);
					survive_reproject(ctx, lh, point3d, out);

					double ex = out[0] - a[0];
					double ey = out[1] - a[1];
					double err_add = sqrt(ex * ex + ey * ey);
					error += err_add;
					error_count++;

					auto &e = eregion(map(a).y, map(a).x);
					e = flow2rgb(ex, ey, 100);

					cv::putText(img, std::to_string(error / error_count), cv::Point2f(10, 20), CV_FONT_HERSHEY_PLAIN, 1,
								cv::Scalar(255, 255, 255));

					if (region.data) {
						if (err_add < .01) {
							cv::line(region, map(a) + cv::Point(ex * 10000, -ey * 10000), map(a),
									 cv::Scalar(255, 255, 255));
						}
						cv::rectangle(region, map(out) - cv::Point(r, r), map(out) + cv::Point(r, r), rcolor);
					}
				}
			}
		};
	}
	if (img.data) {
		cv::imshow("Reprojection", img);
	}
}

void light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length,
				   uint32_t lighthouse) {
	timestamp = timecode;
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);
}

SurvivePose lastPose = {};

void raw_pose_process(SurviveObject *so, uint32_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);
	auto d = dist3d(lastPose.Pos, pose->Pos);
	// std::cerr << d << std::endl;
	if (d < .01) {
		redraw(so->ctx);
	}
	lastPose = *pose;
}

void lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *pose, SurvivePose *obj_pose) {
	survive_default_lighthouse_pose_process(ctx, lighthouse, pose, obj_pose);
}

SurviveContext *create(int argc, char **argv) {
	auto ctx = survive_init(argc, argv);
	if (ctx == nullptr)
		return nullptr;

	survive_install_pose_fn(ctx, raw_pose_process);
	survive_install_lighthouse_pose_fn(ctx, lighthouse_process);
	survive_install_light_fn(ctx, light_process);

	return ctx;
}

void drawbsds(SurviveContext *ctx) {
	int SIZE = 1000;

	std::vector<SurviveCalFlag> show_flags = {
		SVCal_All, SVCal_Phase, SVCal_Gib, SVCal_Curve, SVCal_Tilt,
	};

		for (int lh = 0; lh < 2; lh++) {
			cv::Mat_<cv::Vec3b> img = cv::Mat_<cv::Vec3b>(SIZE, SIZE);
			img.setTo(cv::Vec3b(0, 0, 0));
			for (int x = 0; x < SIZE; x++) {
				for (int y = 0; y < SIZE; y++) {
					FLT in[2] = {x * M_PI / SIZE - M_PI / 2., y * M_PI / SIZE - M_PI / 2.};
					if (fabs(in[0]) > 60. / 180 * M_PI || fabs(in[1]) > 60. / 180 * M_PI)
						continue;

					FLT out[2];
					survive_apply_bsd_calibration(ctx, lh, in, out);
					double ex = out[0] - in[0];
					double ey = out[1] - in[1];
					ex -= ctx->bsd[lh].fcal[0].phase;
					ey -= ctx->bsd[lh].fcal[1].phase;

					// Make it opposite of angles
					ex *= -1;
					ey *= -1;

					img(y, x) = flow2rgb(ex, ey, 100);
				}
			}
			draw_viz(img);
			cv::imwrite("BSD" + std::to_string(lh) + ".png", img);
			cv::imshow("BSD" + std::to_string(lh), img);
		}
}

int main(int argc, char **argv) {
	// for (int i = 0; i < 1 << 5; i++) {
	// size_t cidx = 15 | (i << 5);

	//        auto conf = survive_calibration_config_create_from_idx(cidx);
	//      if (survive_calibration_config_index(&conf) != cidx)
	//        continue;

	auto ctx1 = create(argc, argv);
	if (ctx1 == nullptr)
		return -1;
	size_t cidx = survive_configi(ctx1, "default-cal-conf", SC_GET, 0);
	size_t idx = survive_configi(ctx1, "default-cal-conf2", SC_GET, 0);

	SurviveContext *ctx2 = 0;
	int numCtx = 1;
	int ctx2_flag = 1;
	if (idx != 0) {
		numCtx++;
		ctx2 = create(argc, argv);
		ctx2->user_ptr = (void *)&ctx2_flag;
	}

	size_t showui = survive_configi(ctx1, "show-ui", SC_GET, 0);
	if (showui)
		drawbsds(ctx1);

	int waitUpdate = 100;
	int SIZE = 1000;
	if (showui) {
		img = cv::Mat_<cv::Vec3b>(SIZE, numCtx * SIZE);
		img.setTo(cv::Vec3b(0, 0, 0));
	}
	for (int lh = 0; lh < 2; lh++) {
		err[lh] = cv::Mat_<cv::Vec3b>(SIZE, numCtx * SIZE);
		err[lh].setTo(cv::Vec3b(0, 0, 0));
	}

	if (img.data) {
		cv::imshow("Reprojection", img);
		cv::waitKey(0);
	}

	auto start = std::chrono::high_resolution_clock::now();
	while (survive_poll(ctx1) == 0 && (ctx2 == 0 || survive_poll(ctx2) == 0)) {
		auto now = std::chrono::high_resolution_clock::now();
		if ((now - start) > std::chrono::milliseconds(33)) {
			cv::waitKey(1);
			start = now;
		}
	}

	for (int i = 0; i < 2; i++) {
		draw_viz(err[i]);
		cv::putText(err[i], std::to_string(error / error_count), cv::Point2f(100, 20), CV_FONT_HERSHEY_PLAIN, 1,
					cv::Scalar(255, 255, 255));
		cv::putText(err[i], std::to_string(cidx), cv::Point2f(100, 40), CV_FONT_HERSHEY_PLAIN, 1,
					cv::Scalar(255, 255, 255));

		auto name = "LH" + std::to_string(i);
		cv::imwrite(name + "_" + std::to_string(cidx) + ".png", err[i]);
		cv::imshow(name, err[i]);
	}

	survive_close(ctx1);

	std::cerr << "Error: " << error / error_count << std::endl;

	int c = '\0';
	while (((c = cv::waitKey(0)) & 0xff) != 'q') {
		std::cerr << (uint8_t)c << std::endl;
	}
	return 0;
}
