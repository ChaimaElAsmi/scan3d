#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP


#include <opencv2/opencv.hpp>
#include <stdio.h>

//triangulation (Proj -> Cam = 0) , (Cam -> Proj = 1)
//#define TR_CAM 0

// strings pour les noms de fichier
#define IDX_TR_MASK_CAM   0
#define IDX_TR_MASK_PROJ  1
#define IDX_TR_DATA_CAM   2
#define IDX_TR_DATA_PROJ  3
#define IDX_TR_PARC       4
#define IDX_TR_PARP       5

class triangulation {

    //projector
    cv::Mat internes_proj;
    cv::Mat rotation_proj;
    cv::Mat translation_proj;
    cv::Mat distCoeffs_proj;
    cv::Mat poseMatrix_proj;

    //camera
    cv::Mat internes_cam;
    cv::Mat rotation_cam;
    cv::Mat translation_cam;
    cv::Mat distCoeffs_cam;
    cv::Mat poseMatrix_cam;

    // filenames
    const char *fn_tr_mask_cam;
    const char *fn_tr_mask_proj;
    const char *fn_tr_data_cam;
    const char *fn_tr_data_proj;
    const char *fn_tr_parc;
    const char *fn_tr_parp;

    public:
    triangulation();
    ~triangulation();

    void triangulate(cv::Mat lutCam, cv::Mat lutProj);
    void setPathT(int idx, std::__cxx11::string path, const char *filename);


    private:
    int initMat(std::string file, cv::Mat &internes, cv::Mat &rotation, cv::Mat &translation, cv::Mat &distCoeffs);
    int matrixCorr(cv::Mat &pointsLut, cv::Mat &pointsCorr, cv::Mat lutSrc, cv::Mat lutDst,int camproj,cv::Mat &pointIdx);
    int saveMat(const char *name,cv::Mat point4D);
    int saveSTL(const char *name,cv::Mat point4D,cv::Mat pointIdx);

    int lut2corr(cv::Mat lutSrc, cv::Mat internesSrc, cv::Mat distCoeffsSrc, cv::Mat &pointsUndSrc,
                 cv::Mat lutDst, cv::Mat internesDst, cv::Mat distCoeffsDst, cv::Mat &pointsUndDst,int camproj,cv::Mat &pointIdx);


    void composePoseMatrix(cv::Mat &poseMatrix, cv::Mat rotation, cv::Mat translation);
    void undistortMatrix(cv::Mat &pointsOutput, cv::Mat pointsInput, cv::Mat internes, cv::Mat distCoeffs);



};


#endif // TRIANGULATION_HPP

