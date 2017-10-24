#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP


#include <opencv2/opencv.hpp>
#include <stdio.h>


int initMat(std::string file, cv::Mat &internes, cv::Mat &rotation, cv::Mat &translation, cv::Mat &distCoeffs);
int matrixCorr(cv::Mat &pointsLut, cv::Mat &pointsCorr, cv::Mat lutSrc, cv::Mat lutDst);
int saveMat(cv::Mat point4D);
int lut2corr(cv::Mat lutSrc, cv::Mat internesSrc, cv::Mat distCoeffsSrc, cv::Mat &pointsUndSrc,
             cv::Mat lutDst, cv::Mat internesDst, cv::Mat distCoeffsDst, cv::Mat &pointsUndDst);


void composePoseMatrix(cv::Mat &poseMatrix, cv::Mat rotation, cv::Mat translation);
void undistortMatrix(cv::Mat &pointsOutput, cv::Mat pointsInput, cv::Mat internes, cv::Mat distCoeffs);
void triangulate(cv::Mat lutCam, cv::Mat lutProj);


#endif // TRIANGULATION_HPP

