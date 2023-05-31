#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cvdef.h"
// #include "opencv2/imgproc/imgproc_c.h"
// #include "opencv2/highgui/highgui_c.h"
#include "LowPassFilter/LowPassFilter.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <stdio.h>
using namespace cv;

class MeanFilter {
    private:
        float mean = 0;
    public:
        void correct(float value) {
            mean = (mean + value) / 2;
        }
        float predict() {
            return mean;
        }
};

float kalman_filter(KalmanFilter &KF, float range);
float mean_filter(MeanFilter &MF, float range);
float lowpass_filter(LowPassFilter &LPF, float range);
float show_measure(std::vector<float> ranges);
void draw_points(std::vector<cv::Point2d> points);