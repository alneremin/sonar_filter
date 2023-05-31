#include "filters.h"
#include "plot.h"

float kalman_filter(KalmanFilter &KF, float range)
{
    Mat prediction = KF.predict();
    float prediction1 = prediction.at<float>(0);
    Mat_<float> measurement(1, 1);
    measurement.at<float>(0, 0) = range;

    KF.correct(measurement);

    return prediction1;
}

float mean_filter(MeanFilter &MF, float range)
{
    float prediction = MF.predict();

    MF.correct(range);

    return prediction;
}

float lowpass_filter(LowPassFilter &LPF, float range) {
    float prediction = LPF.getOutput();
    LPF.update(range);
    return prediction;
}

float show_measure(std::vector<float> ranges) {
    std::vector<cv::Point2d> v;
    cv::Point2d init_p;
    v.push_back(init_p);

    for(int i = 0; i < ranges.size(); ++i)
    {
        cv::Point2d p;
        p.x = ranges[i];
        p.y = 0;
        v.push_back(p);
    }
    draw_points(v);
}

void draw_points(std::vector<cv::Point2d> points) {
    
    cv::Mat xData, yData, plot_result;
    xData.create(1, points.size(), CV_64F);
    yData.create(1, points.size(), CV_64F);

    for(int i = 0; i < points.size(); ++i)
    {
        xData.at<double>(i) = -points[i].x;
        yData.at<double>(i) = points[i].y;
    }

    auto plot = cv::plot::Plot2d::create(yData, xData);
    plot->setPlotSize(500, 500);
    plot->setMaxX(1);
    plot->setMinX(-2.1);
    plot->setMaxY(1);
    plot->setMinY(-1);
    plot->setNeedPlotLine(false);
    // plot->setPlotBackgroundColor(cv::Scalar(50, 50, 50)); 
    // plot->setPlotLineColor(cv::Scalar(50, 50, 255));
    plot->render(plot_result);          

    imshow("Graph", plot_result);
    cv::waitKey(1);
}