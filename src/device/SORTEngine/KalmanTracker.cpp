/**
 * ============================================================================
 *
 * Copyright (C) Huawei Technologies Co., Ltd. 2019. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include "KalmanTracker.h"

void KalmanTracker::CVKalmanInit(cv::Rect_<float> stateMat)
{
    /*
    The SORT algorithm uses a linear constant velocity model,which assumes 7
    states, including:
            x coordinate of bounding box center
            y coordinate of bounding box center
            area of bounding box
            aspect ratio of w to h
            velocity of x
            velocity of y
            variation rate of area

    The aspect ratio is considered to be unchanged, so there is no additive item
    for aspect ratio in the transitionMatrix


    Kalman filter equation step by step:
    (1)  X(k|k-1)=AX(k-1|k-1)+BU(k)
    X(k|k-1) is the predicted state(statePre),X(k-1|k-1) is the k-1 statePost,A
    is transitionMatrix, B is controlMatrix, U(k) is control state, in SORT U(k) is 0.

    (2)  P(k|k-1)=AP(k-1|k-1)A'+Q
     P(k|k-1) is the predicted errorCovPre, P(k-1|k-1) is the k-1 errorCovPost,
    Q is processNoiseCov

    (3)  Kg(k)=P(k|k-1)H'/(HP(k|k-1))H'+R
    Kg(k) is the kalman gain, the ratio of estimate variance in total variance,
    H is the measurementMatrix,R is the measurementNoiseCov

    (4)  X(k|k)=X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))
     X(k|k) is the k statePost, Z(k) is the measurement of K, in SORT Z(k) is
    the detection result of k

    (5)  P(k|k)=(1-Kg(k)H)P(k|k-1)
    P(k|k) is the errorCovPost

    */
    const int STATE_DIM = 7;
    const int MEASURE_DIM = 4;
    cvkalmanfilter = cv::KalmanFilter(STATE_DIM, MEASURE_DIM, 0);  // zero control
    measurement = cv::Mat::zeros(MEASURE_DIM, 1, CV_32F);  // 4 measurements, Z(k), according to detection results

    // A, will not be updated
    cvkalmanfilter.transitionMatrix = (cv::Mat_<float>(STATE_DIM, STATE_DIM) << 
                                        1, 0, 0, 0, 1, 0, 0,
                                        0, 1, 0, 0, 0, 1, 0,
                                        0, 0, 1, 0, 0, 0, 1,
                                        0, 0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 0, 0, 0, 1, 0,
                                        0, 0, 0, 0, 0, 0, 1);

    cv::setIdentity(cvkalmanfilter.measurementMatrix);                           // H, will not be updated
    cv::setIdentity(cvkalmanfilter.processNoiseCov, cv::Scalar::all(1e-2));      // Q, will not be updated
    cv::setIdentity(cvkalmanfilter.measurementNoiseCov, cv::Scalar::all(1e-1));  // R, will bot be updated
    cv::setIdentity(cvkalmanfilter.errorCovPost, cv::Scalar::all(1));            // P(k-1|k-1), will be updated

    // initialize state vector with bounding box in
    // [center_x,center_y,area,ratio]
    // style, the velocity is 0
    // X(k-1|k-1)
    cvkalmanfilter.statePost.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
    cvkalmanfilter.statePost.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
    cvkalmanfilter.statePost.at<float>(2, 0) = stateMat.area();
    cvkalmanfilter.statePost.at<float>(3, 0) = stateMat.width / stateMat.height;
}

// Predict the bounding box.
cv::Rect_<float> KalmanTracker::Predict()
{
    // predict
    // return X(k|k-1)=AX(k-1|k-1), and update
    // P(k|k-1)=AP(k-1|k-1)A'+Q
    cv::Mat predictState = cvkalmanfilter.predict();

    float w = sqrt(predictState.at<float>(2, 0) * predictState.at<float>(3, 0));
    if (w < DBL_EPSILON) {
        return cv::Rect_<float>(0, 0, 0, 0);
    }
    float h = predictState.at<float>(2, 0) / w;
    float x = (predictState.at<float>(0, 0) - w / 2);
    float y = (predictState.at<float>(1, 0) - h / 2);

    if (x < 0 && predictState.at<float>(0, 0) > 0) {
        x = 0;
    }
    if (y < 0 && predictState.at<float>(1, 0) > 0) {
        y = 0;
    }

    return cv::Rect_<float>(x, y, w, h);
}

// Update the state using observed bounding box
void KalmanTracker::Update(cv::Rect_<float> stateMat)
{
    // measurement, update Z(k)
    measurement.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
    measurement.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
    measurement.at<float>(2, 0) = stateMat.area();
    measurement.at<float>(3, 0) = stateMat.width / stateMat.height;

    // update, do the following steps:
    // Kg(k)=P(k|k-1)H'/(HP(k|k-1))H'+R
    // X(k|k)=X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))
    // P(k|k)=(1-Kg(k)H)P(k|k-1)
    cvkalmanfilter.correct(measurement);
}
