#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>



using namespace cv;
using namespace std;

int initMat(String file, Mat internes, Mat rotation, Mat translation) {
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

void composeProjectionMatrix(Mat projectionMatrix, Mat internes, Mat rotation, Mat translation) {

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


int main(int argc, char *argv[])
{
    //Projecteur
    String fileProj = "out_projector_data.xml";
    Mat int_proj = Mat::zeros(3, 3, CV_64F);
    Mat rot_proj = Mat::zeros(3, 3, CV_64F);
    Mat trans_proj = Mat::zeros(3, 1, CV_64F);

    //Récupérer les matrices internes, rotation et translation
    initMat(fileProj, int_proj, rot_proj, trans_proj);

    cout << "Matrice du projecteur = " << endl << int_proj << endl;
    cout << endl;
    cout << "Rotation = " << endl << rot_proj << endl;
    cout << endl;
    cout << "Translation = " << endl << trans_proj << endl << endl;

    Mat projectionMatrixProj = Mat::zeros(3, 4, CV_64F);

    //composer la matrice de projection
    composeProjectionMatrix(projectionMatrixProj, int_proj, rot_proj, trans_proj);

    cout << "proj = " << endl << projectionMatrixProj << endl;


    //Caméra
    String fileCam = "out_camera_data.xml";
    Mat int_cam = Mat::zeros(3, 3, CV_64F);
    Mat rot_cam = Mat::zeros(3, 3, CV_64F);
    Mat trans_cam = Mat::zeros(3, 1, CV_64F);

    //Récupérer les matrices internes, rotation et translation
    initMat(fileCam, int_cam, rot_cam, trans_cam);

    cout << endl << endl;
    cout << "Matrice de la caméra = " << endl << int_cam << endl;
    cout << endl;
    cout << "Rotation = " << endl << rot_cam << endl;
    cout << endl;
    cout << "Translation = " << endl << trans_cam << endl << endl;

    Mat projectionMatrixCam = Mat::zeros(3, 4, CV_64F);

    //composer la matrice de projection
    composeProjectionMatrix(projectionMatrixCam, int_cam, rot_cam, trans_cam);

    cout << "proj = " << endl << projectionMatrixCam << endl;

    //Lecture de la LUT du projecteur
    String filename = "/home/chaima/Documents/scanGit/scan3d/calibration/LUT/new/lutproj_08.png";
    Mat lut = imread(filename, CV_LOAD_IMAGE_COLOR);

//    namedWindow("Display LUT", 1);
//    imshow("Display LUT", lut);
//    waitKey(0);

    int lutr = lut.rows;
    int lutc = lut.cols;
    cout << "rows " << lutr << endl << "cols " << lutc << endl;

    Point3_<uchar>* p;
    int pas = 1;
    Mat pointsLut = Mat::zeros(2, (int)(lutr*lutc)/2, CV_64F);
    Mat pointsLut2 = Mat::zeros(2, (int)(pointsLut.cols/pas), CV_64F);
    Mat pointsCorr = Mat::zeros(2, (int)(pointsLut.cols/pas), CV_64F);

    pointsLut = lut.reshape(3,2);

    for(int i=0; i<pointsLut.rows; i++) {
        for(int j=0; j<(int)(pointsLut.cols/pas); j++) {
            //pointsLut2.at<uchar>(i, j) = pointsLut.at<uchar>(i,j*pas);
            p = pointsLut.ptr<Point3_<uchar> >(i,j);
            pointsCorr.at<Vec3s>(i,j)=Vec3s( (p->x*1770)/65535, (p->y*lutr)/65535, (p->z*lutc)/65535);

            //printf("red  %u  green  %u   blue   %u \n", p->z, p->y, p->x);
        }
    }

    //cout << pointsCorr << endl;

    //Triangulation
    Mat point4D = Mat::zeros(4, (int)(pointsLut.cols/pas), CV_64F);
    triangulatePoints(projectionMatrixProj, projectionMatrixCam, pointsLut2, pointsCorr, point4D);

//    double w = point4D.at<double>(3,0);
//    double x = point4D.at<double>(0,0)/w;
//    double y = point4D.at<double>(1,0)/w;
//    double z = point4D.at<double>(2,0)/w;

//    cout << x << ", " << y << ", " << z << endl;


    cout << "test " << endl;

}

