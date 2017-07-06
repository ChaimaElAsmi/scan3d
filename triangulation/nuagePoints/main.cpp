#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>



using namespace cv;
using namespace std;

int initMat(String file, Mat &internes, Mat &rotation, Mat &translation) {

    internes = Mat::zeros(3, 3, CV_64F);
    rotation = Mat::zeros(3, 3, CV_64F);
    translation = Mat::zeros(3, 1, CV_64F);

    string filename = "/home/chaima/Documents/scanGit/scan3d/calibration/" + file;
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) {
        cout << "Erreur! Le fichier n'est pas ouvert! \n" << endl;
        return -1;
    }

    fs["Camera_Matrix"] >> internes;
    fs["Rotation"] >> rotation;
    fs["Translation"] >> translation;

    fs.release();
    return 0;
}

void composeProjectionMatrix(Mat &projectionMatrix, Mat internes, Mat rotation, Mat translation) {

    projectionMatrix = Mat::zeros(3, 4, CV_64F);

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

    projectionMatrix = internes * idt * mat_rot * mat_trans;
}

void matrixCorr(Mat &pointsLut, Mat &pointsCorr, Mat lut) {

    int lutr = lut.rows;
    int lutc = lut.cols;
    cout << "rowsLut " << lutr << endl << "colsLut " << lutc << endl;

    unsigned short *p = (unsigned short*) lut.data;
    unsigned short *q;
    Mat mask = Mat::zeros(lutr, lutc, CV_8UC1);
    int n=0;
    for(int r=0; r<lutr; r+=10) {
        for(int c=0; c<lutc; c+=10) {
            q = p+r*lutc+c;
            //printf("%d  %d  %d \n", q[0], q[1],q[3]);
            float f = (float)q[0]/65535; //pourcentage d'erreur
            if((q[0]==0) || (f>0.2)) continue;
            mask.at<uchar>(r, c) = 255;
            n++;
        }
    }
    cout << "j'ai sélectionné " << n << " pixels" << endl;

    imwrite("/home/chaima/Documents/scanGit/scan3d/triangulation/nuagePoints/mask.png", mask);

    pointsLut = Mat::zeros(2, n, CV_64F);
    pointsCorr = Mat::zeros(2, n, CV_64F);

    n=0;
    for(int r=0; r<lutr; r++) {
        for(int c=0; c<lutc; c++) {
            if(mask.at<uchar>(r, c) == 0) continue;

            q = p+r*lutc+c;
            printf("%d  %d  %d \n", q[0], q[1],q[3]);
            pointsLut.at<double>(0,n) = c;
            pointsLut.at<double>(1,n) = r;
            pointsCorr.at<double>(0,n) = ((double)q[2]*640)/65535;
            pointsCorr.at<double>(1,n) = ((double)q[1]*480)/65535;
            printf("(%f, %f) -> (%f, %f) \n",pointsLut.at<double>(0,n),pointsLut.at<double>(1,n),
                                             pointsCorr.at<double>(0,n),pointsCorr.at<double>(1,n));
            n++;
        }
    }

    cout << "pointsCorr = " << pointsCorr.rows << "    " << pointsCorr.cols << endl;
}


int main(int argc, char *argv[])
{
    //Projecteur
    String fileProj = "out_projector_data.xml";
    Mat int_proj;
    Mat rot_proj;
    Mat trans_proj;

    //Récupérer les matrices internes, rotation et translation
    initMat(fileProj, int_proj, rot_proj, trans_proj);

    cout << "Matrice du projecteur = " << endl << int_proj << endl;
    cout << endl;
    cout << "Rotation = " << endl << rot_proj << endl;
    cout << endl;
    cout << "Translation = " << endl << trans_proj << endl << endl;

    Mat projectionMatrixProj;

    //composer la matrice de projection
    composeProjectionMatrix(projectionMatrixProj, int_proj, rot_proj, trans_proj);

    cout << "proj = " << endl << projectionMatrixProj << endl;


    //Caméra
    String fileCam = "out_camera_data.xml";
    Mat int_cam;
    Mat rot_cam;
    Mat trans_cam;

    //Récupérer les matrices internes, rotation et translation
    initMat(fileCam, int_cam, rot_cam, trans_cam);

    cout << endl << endl;
    cout << "Matrice de la caméra = " << endl << int_cam << endl;
    cout << endl;
    cout << "Rotation = " << endl << rot_cam << endl;
    cout << endl;
    cout << "Translation = " << endl << trans_cam << endl << endl;

    Mat projectionMatrixCam;

    //composer la matrice de projection
    composeProjectionMatrix(projectionMatrixCam, int_cam, rot_cam, trans_cam);

    cout << "proj = " << endl << projectionMatrixCam << endl;

    //Lecture de la LUT de la caméra et du projecteur
    String filename = "/home/chaima/Documents/scanGit/scan3d/calibration/LUT/new/";
    String nameCam  = "lutcam_08.png";
    String nameProj  = "lutproj_08.png";
    Mat lutcam  = imread(filename + nameCam, CV_LOAD_IMAGE_UNCHANGED);
    Mat lutproj = imread(filename + nameProj, CV_LOAD_IMAGE_UNCHANGED);

//    namedWindow("Display LUT", 1);
//    imshow("Display LUT", lutcam);
//    waitKey(0);
//    imshow("Display LUT", lutproj);
//    waitKey(0);


    Mat pointsLut;
    Mat pointsCorr;
    matrixCorr(pointsLut, pointsCorr, lutproj);


    //Triangulation
//    Mat point4D = Mat::zeros(4, (int)(size/pas), CV_64F);
//    triangulatePoints(projectionMatrixProj, projectionMatrixCam, pointsLut, pointsCorr, point4D);

//    double w = point4D.at<double>(3,0);
//    double x = point4D.at<double>(0,0)/w;
//    double y = point4D.at<double>(1,0)/w;
//    double z = point4D.at<double>(2,0)/w;

//    cout << x << ", " << y << ", " << z << endl;


    cout << "test " << endl;

}

