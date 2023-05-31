
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include "filters.h"

void collback(const sensor_msgs::Range &range);
ros::Publisher kalmanPub;
ros::Publisher meanPub;
ros::Publisher lowpassPub;

std_msgs::Float32 kalmanMsg;
std_msgs::Float32 meanMsg;
std_msgs::Float32 lowpassMsg;

KalmanFilter KF(1, 1, 0);
MeanFilter MF;
LowPassFilter LPF(1.0, 0.01);

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "sonar_filter");
    
    KF.transitionMatrix = (Mat_<float>(1, 1) << 1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/sonar", 1, collback);
    kalmanPub = node.advertise<std_msgs::Float32>("/filters/kalman", 1);
    meanPub = node.advertise<std_msgs::Float32>("/filters/mean", 1);
    lowpassPub = node.advertise<std_msgs::Float32>("/filters/lowpass", 1);
    ros::spin();
    return 0;
}

void collback(const sensor_msgs::Range &range) {
    // std::cout << range.range << std::endl;
    kalmanMsg.data = kalman_filter(KF, range.range);
    kalmanPub.publish(kalmanMsg);

    meanMsg.data = mean_filter(MF, range.range);
    meanPub.publish(meanMsg);

    lowpassMsg.data = lowpass_filter(LPF, range.range);
    lowpassPub.publish(lowpassMsg);

    show_measure({range.range, kalmanMsg.data, meanMsg.data, lowpassMsg.data});
}