#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>

#include <triangulation.hpp>
#include <paths.hpp>

using namespace cv;
using namespace std;


int initMat(string file, Mat &internes, Mat &rotation, Mat &translation, Mat &distCoeffs) {

    internes = Mat::zeros(3, 3, CV_64F);
    rotation = Mat::zeros(3, 3, CV_64F);
    translation = Mat::zeros(3, 1, CV_64F);
    distCoeffs = Mat::zeros(5, 1, CV_64F);

    FileStorage fsI(file, FileStorage::READ);
    if(!fsI.isOpened()) {
        cout << "Erreur! Le fichier Init n'est pas ouvert! \n" << endl;
        return -1;
    }

    fsI["Camera_Matrix"] >> internes;
    fsI["Rotation"] >> rotation;
    fsI["Translation"] >> translation;
    fsI["Distortion_Coefficients"] >> distCoeffs;

    fsI.release();
    return 0;
}

void composePoseMatrix(Mat &poseMatrix, Mat rotation, Mat translation) {

    poseMatrix = Mat::zeros(3, 4, CV_64F);

    Mat idt = Mat::eye(3, 4, CV_64F);
    Mat mat_trans = Mat::eye(4, 4, CV_64F);
    Mat mat_rot = Mat::eye(4, 4, CV_64F);

    //Calcul de la matrice de translation (4x4)
    int rowsT = mat_trans.rows;
    int colsT = mat_trans.cols;
    for(int i=0; i<rowsT; i++) {
        for(int j=0; j<colsT-1; j++) {
            if(i == rowsT-1)
                mat_trans.at<double>(j,i) = translation.at<double>(j);
        }
    }

    //Calcul de la matrice de rotation (4x4)
    for(int i=0; i<rowsT; i++) {
        for(int j=0; j<colsT-1; j++) {
            if(i != colsT-1)
                mat_rot.at<double>(j,i) = rotation.at<double>(j,i);
        }
    }

    poseMatrix = idt * mat_trans * mat_rot;
}


double randAToB(double a, double b) {
    return a + (rand() / (RAND_MAX / (b-a)));
}

double randAOrB(double a, double b) {
    return (rand()%2) * (b-a) + a;
}

int matrixCorr(Mat &pointsLut, Mat &pointsCorr, Mat lutSrc, Mat lutDst) {

    int lutSrc_r = lutSrc.rows;
    int lutSrc_c = lutSrc.cols;
    int lutDst_r = lutDst.rows;
    int lutDst_c = lutDst.cols;
    cout << "LutSrc(r,c)  " << lutSrc_r << "  " << lutSrc_c << endl;
    cout << "LutDst(r,c)  " << lutDst_r << "  " << lutDst_c << endl;


    unsigned short *p = (unsigned short*) lutSrc.data;
    unsigned short *q;
    Mat mask = Mat::zeros(lutSrc_r, lutSrc_c, CV_8UC1);
    int n=0;
    for(int r=0; r<lutSrc_r; r++) {
        for(int c=0; c<lutSrc_c; c++) {
            q = p+(r*lutSrc_c+c)*3;
            //printf("%d  %d  %d \n", q[0], q[1],q[2]);
            float f = (float)q[0]/65535; //pourcentage d'erreur
//            if(f > 0.5)
            //printf("f = %f \n", f);
            if((q[0]==0) || (f>0.2)) continue;
            //if((r%10)!=0 || (c%10)!=0) continue; //pour avoir un seul point
            mask.at<uchar>(r, c) = 255;
            n++;
        }
    }
    cout << endl << n << " pixels sélectionnés!" << endl;

    imwrite(FN_TR_MASK, mask);

    pointsLut = Mat::zeros(2, n, CV_64F);
    pointsCorr = Mat::zeros(2, n, CV_64F);

    n=0;
    for(int r=0; r<lutSrc_r; r++) {
        for(int c=0; c<lutSrc_c; c++) {
            if(mask.at<uchar>(r, c) == 0) continue;

            //valeur aléatoire [-1,1]
            //srand(time(NULL));
            double varRand = randAOrB(1,-1);

            q = p+(r*lutSrc_c+c)*3;
            //printf("%d  %d  %d \n", q[0], q[1],q[3]);
            pointsLut.at<double>(0,n) = c;
            pointsLut.at<double>(1,n) = r;
            //TEST
            pointsCorr.at<double>(0,n) = ( ((double)q[2]*lutDst_c)/65535 ) + varRand;
            pointsCorr.at<double>(1,n) = ( ((double)q[1]*lutDst_r)/65535 ) + varRand;

            if(n==10000)
                printf("(%f, %f) -> (%f, %f) \n",pointsLut.at<double>(0,n),pointsLut.at<double>(1,n),
                                             pointsCorr.at<double>(0,n),pointsCorr.at<double>(1,n));
            n++;
        }
    }

    cout << endl;
    cout << "OutputArray(r,c)  " << pointsCorr.rows << "  " << pointsCorr.cols << endl;

    return n;
}

void undistortMatrix(Mat &pointsOutput, Mat pointsInput, Mat internes, Mat distCoeffs) {

    Mat pointsInputTr = pointsInput.t();
    Mat points1D = Mat(pointsInputTr.rows, 1, CV_64FC2);

    double *p = (double*) pointsInputTr.data;
    double *q = (double*) points1D.data;

    for(int i=0; i<pointsInputTr.rows*2; i++)
        q[i] = p[i];

    pointsOutput = Mat(pointsInputTr.rows, 1, CV_64FC2);
    undistortPoints(points1D, pointsOutput, internes, distCoeffs);
    double *r = (double*) pointsOutput.data;

    for(int i=0; i<pointsInputTr.rows;i+=(pointsInputTr.rows/2))
    cout  << q[0+i*2] << ", " << q[1+i*2] << "  ->  " << r[0+i*2] << " , " << r[1+i*2] << endl;
}

int lut2corr(Mat lutSrc, Mat internesSrc, Mat distCoeffsSrc, Mat &pointsUndSrc,
         Mat lutDst, Mat internesDst, Mat distCoeffsDst, Mat &pointsUndDst) {

    Mat pointsSrc;
    Mat pointsDst;
    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Taille des Luts :" << endl;
    cout << "--------------------------" << endl;

    int size = matrixCorr(pointsSrc, pointsDst, lutSrc, lutDst);

    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Undistort points : " << size << endl;
    cout << "--------------------------" << endl;
    //Undistort points Source
    undistortMatrix(pointsUndSrc, pointsSrc, internesSrc, distCoeffsSrc);

    //Undistort points Destination
    undistortMatrix(pointsUndDst, pointsDst, internesDst, distCoeffsDst);

    return size;
}

int saveMat(Mat point4D) {

    string filenameS = FN_TR_DATA;
    FileStorage fsS(filenameS, FileStorage::WRITE);
    if(!fsS.isOpened()) {
        cout << "Erreur! Le fichier Save n'est pas ouvert! \n" << endl;
        return -1;
    }

    fsS << "Homogeneous_Coordinates" << point4D;

    fsS.release();

    return 0;
}


void triangulate(Mat lutCam, Mat lutProj) {
    cout << "----- Triangulation -----" << endl;

    //Projecteur
    string fileProj = FN_TR_PPROJ;
    Mat int_proj;
    Mat rot_proj;
    Mat trans_proj;
    Mat distCoeffs_proj;

    //Récupérer les matrices internes, rotation et translation
    initMat(fileProj, int_proj, rot_proj, trans_proj, distCoeffs_proj);

    cout << "--------------------------" << endl;
    cout << "Matrices du projecteur :" << endl;
    cout << "--------------------------" << endl;
    cout << "   Internes = " << endl << int_proj << endl;
    cout << endl;
    cout << "   Rotation = " << endl << rot_proj << endl;
    cout << endl;
    cout << "   Translation = " << endl << trans_proj << endl << endl;
    cout << endl;
    cout << "   DistCoeffs = " << endl << distCoeffs_proj << endl << endl;

    Mat poseMatrixProj;

    //composer la matrice de projection
    composePoseMatrix(poseMatrixProj, rot_proj, trans_proj);

    cout << "   Matrice [R|t] = " << endl << poseMatrixProj << endl;


    //Cameras
    string fileCam = FN_TR_PCAM;
    Mat int_cam;
    Mat rot_cam;
    Mat trans_cam;
    Mat distCoeffs_cam;

    //Récupérer les matrices internes, rotation et translation
    initMat(fileCam, int_cam, rot_cam, trans_cam, distCoeffs_cam);

    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Matrices de la caméra :" << endl;
    cout << "--------------------------" << endl;
    cout << "   Internes = " << endl << int_cam << endl;
    cout << endl;
    cout << "   Rotation = " << endl << rot_cam << endl;
    cout << endl;
    cout << "   Translation = " << endl << trans_cam << endl << endl;
    cout << endl;
    cout << "   DistCoeffs = " << endl << distCoeffs_cam << endl << endl;

    Mat poseMatrixCam;

    //composer la matrice de projection
    composePoseMatrix(poseMatrixCam, rot_cam, trans_cam);

    cout << "   Matrice [R|t] = " << endl << poseMatrixCam << endl;


    //Init
//    string nameProj  = "lutproj_200.png";
//    string nameCam  = "lutcam_200.png";

    Mat pointsUndLut;
    Mat pointsUndCorr;
    Mat point4D;
    int size;

    //Triangulation
    if(TR_CAM) {
        /* Cam -> Proj */
        size = lut2corr(lutCam,  int_cam,  distCoeffs_cam,  pointsUndLut,
                   lutProj, int_proj, distCoeffs_proj, pointsUndCorr);

        point4D = Mat::zeros(4, size, CV_64F);
        triangulatePoints(poseMatrixCam, poseMatrixProj, pointsUndLut, pointsUndCorr, point4D);
    }
    else {
        /* Proj -> Cam */
        size = lut2corr(lutProj, int_proj, distCoeffs_proj, pointsUndLut,
                   lutCam,  int_cam,  distCoeffs_cam,  pointsUndCorr);

        point4D = Mat::zeros(4, size, CV_64F);
        triangulatePoints(poseMatrixProj, poseMatrixCam, pointsUndLut, pointsUndCorr, point4D);
    }

    //Display Coordinates
    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Triangulation points :" << endl;
    cout << "--------------------------" << endl;


    cout << point4D.rows << "    " << point4D.cols << endl;

    double w, x, y, z;
    for(int c=0; c<point4D.cols; c+=(point4D.cols/2)) {

        w = point4D.at<double>(3,c);
        x = point4D.at<double>(0,c)/w;
        y = point4D.at<double>(1,c)/w;
        z = point4D.at<double>(2,c)/w;

        cout << x << ", " << y << ", " << z << endl;
    }

    //Save
    saveMat(point4D);

    cout << endl;
    cout << "----- Triangulation Done -----" << endl;
}
