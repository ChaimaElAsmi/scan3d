//#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>

#include <triangulation.hpp>

using namespace cv;
using namespace std;



triangulation::triangulation() {
    cout << endl;
    cout << "----- Triangulation -----" << endl;

    internes_proj = Mat::zeros(3, 3, CV_64F);
    rotation_proj = Mat::zeros(3, 3, CV_64F);
    translation_proj = Mat::zeros(3, 1, CV_64F);
    distCoeffs_proj = Mat::zeros(5, 1, CV_64F);
    poseMatrix_proj = Mat::zeros(3, 4, CV_64F);

    internes_cam = Mat::zeros(3, 3, CV_64F);
    rotation_cam = Mat::zeros(3, 3, CV_64F);
    translation_cam = Mat::zeros(3, 1, CV_64F);
    distCoeffs_cam = Mat::zeros(5, 1, CV_64F);
    poseMatrix_cam = Mat::zeros(3, 4, CV_64F);

    // strings pour les filename
    fn_tr_mask_cam=NULL;
    fn_tr_mask_proj=NULL;
    fn_tr_data_cam=NULL;
    fn_tr_data_proj=NULL;
    fn_tr_parc=NULL;
    fn_tr_parp=NULL;

}



triangulation::~triangulation() {
    cout << endl;
    cout << "----- Triangulation Done -----" << endl;
}


int triangulation::initMat(string file, Mat &internes, Mat &rotation, Mat &translation, Mat &distCoeffs) {

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

void triangulation::composePoseMatrix(Mat &poseMatrix, Mat rotation, Mat translation) {

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

// camproj: 0=cam, 1=proj. Sert pour sauvegarder le mask
int triangulation::matrixCorr(Mat &pointsLut, Mat &pointsCorr, Mat lutSrc, Mat lutDst,int camproj,Mat &pointIdx) {

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

    if( camproj==0 ) imwrite(fn_tr_mask_cam, mask);
    else imwrite(fn_tr_mask_proj, mask);

    pointsLut = Mat::zeros(2, n, CV_64F);
    pointsCorr = Mat::zeros(2, n, CV_64F);
    pointIdx = Mat::zeros(lutSrc_r, lutSrc_c, CV_32SC1);


    n=0;
    for(int r=0; r<lutSrc_r; r++) {
        for(int c=0; c<lutSrc_c; c++) {
            if(mask.at<uchar>(r, c) == 0) {
                pointIdx.at<int>(r, c)=-1;
                continue;
            }
            pointIdx.at<int>(r, c)=n;

            //valeur aléatoire [-1,1]
            //srand(time(NULL));
            double varRand = randAOrB(1,-1);

            q = p+(r*lutSrc_c+c)*3;
            //printf("%d  %d  %d \n", q[0], q[1],q[3]);
            pointsLut.at<double>(0,n) = c;
            pointsLut.at<double>(1,n) = r;
            //TEST
            pointsCorr.at<double>(0,n) = ( ((double)q[2]*lutDst_c)/65535 ); //+ varRand;
            pointsCorr.at<double>(1,n) = ( ((double)q[1]*lutDst_r)/65535 ); //+ varRand;

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

void triangulation::undistortMatrix(Mat &pointsOutput, Mat pointsInput, Mat internes, Mat distCoeffs) {

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

int triangulation::lut2corr(Mat lutSrc, Mat internesSrc, Mat distCoeffsSrc, Mat &pointsUndSrc,
         Mat lutDst, Mat internesDst, Mat distCoeffsDst, Mat &pointsUndDst,int camproj,Mat &pointIdx) {

    Mat pointsSrc;
    Mat pointsDst;
    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Taille des Luts :" << endl;
    cout << "--------------------------" << endl;

    int size = matrixCorr(pointsSrc, pointsDst, lutSrc, lutDst,camproj,pointIdx);

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

int triangulation::saveMat(const char *name,Mat point4D) {

    FileStorage fsS(name, FileStorage::WRITE);
    if(!fsS.isOpened()) {
        cout << "Erreur! Le fichier Save n'est pas ouvert! \n" << endl;
        return -1;
    }

    fsS << "Homogeneous_Coordinates" << point4D;

    fsS.release();

    return 0;
}

int triangulation::saveSTL(const char *name,Mat point4D,Mat pointIdx) {
    FILE *F=fopen(name,"w");
    if( F==NULL ) { printf("Impossible d'ouvrir %s\n",name);return(-1); }

	int nr=pointIdx.rows;
	int nc=pointIdx.cols;

	fprintf(F,"solid v3d\n");

	double xA,yA,zA,w;
	double xB,yB,zB;
	double xC,yC,zC;
	double xD,yD,zD;
	int idxA,idxB,idxC,idxD;
	for(int r=0; r<nr-1; r++) {
        for(int c=0; c<nc-1; c++) {
            idxA=pointIdx.at<int>(r, c);
            idxB=pointIdx.at<int>(r, c+1);
            idxC=pointIdx.at<int>(r+1, c);
            idxD=pointIdx.at<int>(r+1, c+1);

			if( idxA>=0 ) {
				w = point4D.at<double>(3,idxA);
				xA = point4D.at<double>(0,idxA)/w;
				yA = point4D.at<double>(1,idxA)/w;
				zA = point4D.at<double>(2,idxA)/w;
			}
			if( idxB>=0 ) {
				w = point4D.at<double>(3,idxB);
				xB = point4D.at<double>(0,idxB)/w;
				yB = point4D.at<double>(1,idxB)/w;
				zB = point4D.at<double>(2,idxB)/w;
			}
			if( idxC>=0 ) {
				w = point4D.at<double>(3,idxC);
				xC = point4D.at<double>(0,idxC)/w;
				yC = point4D.at<double>(1,idxC)/w;
				zC = point4D.at<double>(2,idxC)/w;
			}
			if( idxD>=0 ) {
				w = point4D.at<double>(3,idxD);
				xD = point4D.at<double>(0,idxD)/w;
				yD = point4D.at<double>(1,idxD)/w;
				zD = point4D.at<double>(2,idxD)/w;
			}

			if( idxA>=0 && idxB>=0 && idxC>=0 ) {
 				fprintf(F,"facet normal 0 0 0\n");
				fprintf(F,"outer loop\n");
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xA,yA,zA);
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xC,yC,zC);
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xB,yB,zB);
				fprintf(F,"endloop\n");
				fprintf(F,"endfacet\n");
			}
			if( idxB>=0 && idxC>=0 && idxD>=0 ) {
 				fprintf(F,"facet normal 0 0 0\n");
				fprintf(F,"outer loop\n");
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xB,yB,zB);
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xC,yC,zC);
				fprintf(F,"vertex %12.6f %12.6f %12.6f\n",xD,yD,zD);
				fprintf(F,"endloop\n");
				fprintf(F,"endfacet\n");
			}
/*
  vertex -56.1573 -22.6431 21.6972
  vertex -55.6515 -22.6336 21.7902
  vertex -56.2969 -23.1655 21.9234
*/
		}
	}


	fprintf(F,"endsolid v3d\n");

    fclose(F);

    return(0);
}



void triangulation::triangulate(Mat lutCam, Mat lutProj) {

    /* ---Projecteur--- */

    //Récupérer les matrices internes, rotation et translation
    initMat(fn_tr_parp, internes_proj, rotation_proj, translation_proj, distCoeffs_proj);

    cout << "--------------------------" << endl;
    cout << "Matrices du projecteur :" << endl;
    cout << "--------------------------" << endl;
    cout << "   Internes = " << endl << internes_proj << endl;
    cout << endl;
    cout << "   Rotation = " << endl << rotation_proj << endl;
    cout << endl;
    cout << "   Translation = " << endl << translation_proj << endl << endl;
    cout << endl;
    cout << "   DistCoeffs = " << endl << distCoeffs_proj << endl << endl;

    //composer la matrice de projection
    composePoseMatrix(poseMatrix_proj, rotation_proj, translation_proj);

    cout << "   Matrice [R|t] = " << endl << poseMatrix_proj << endl;


    /* ---Camera--- */

    //Récupérer les matrices internes, rotation et translation
    initMat(fn_tr_parc, internes_cam, rotation_cam, translation_cam, distCoeffs_cam);

    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Matrices de la caméra :" << endl;
    cout << "--------------------------" << endl;
    cout << "   Internes = " << endl << internes_cam << endl;
    cout << endl;
    cout << "   Rotation = " << endl << rotation_cam << endl;
    cout << endl;
    cout << "   Translation = " << endl << translation_cam << endl << endl;
    cout << endl;
    cout << "   DistCoeffs = " << endl << distCoeffs_cam << endl << endl;

    //composer la matrice de projection
    composePoseMatrix(poseMatrix_cam, rotation_cam, translation_cam);

    cout << "   Matrice [R|t] = " << endl << poseMatrix_cam << endl;

    Mat pointsUndLut;
    Mat pointsUndCorr;
    Mat point4Dcam;
    Mat point4Dproj;
    int size;

    Mat pointIdxCam;
    Mat pointIdxProj;

    //Triangulation
    // CAM en premier
        /* Cam -> Proj */
        size = lut2corr(lutCam,  internes_cam,  distCoeffs_cam,  pointsUndLut,
                   lutProj, internes_proj, distCoeffs_proj, pointsUndCorr,0,pointIdxCam);

        point4Dcam = Mat::zeros(4, size, CV_64F);
        triangulatePoints(poseMatrix_cam, poseMatrix_proj, pointsUndLut, pointsUndCorr, point4Dcam);

    // PROJ en second
        /* Proj -> Cam */
        size = lut2corr(lutProj, internes_proj, distCoeffs_proj, pointsUndLut,
                   lutCam,  internes_cam,  distCoeffs_cam,  pointsUndCorr,1,pointIdxProj);

        point4Dproj = Mat::zeros(4, size, CV_64F);
        triangulatePoints(poseMatrix_proj, poseMatrix_cam, pointsUndLut, pointsUndCorr, point4Dproj);

    //Display Coordinates
    cout << endl << endl;
    cout << "--------------------------" << endl;
    cout << "Triangulation points :" << endl;
    cout << "--------------------------" << endl;


    cout << "CAM: "<<point4Dcam.rows << "    " << point4Dcam.cols << endl;
    cout << "PROJ:"<<point4Dproj.rows << "    " << point4Dproj.cols << endl;

    /*
    double w, x, y, z;
    for(int c=0; c<point4D.cols; c+=(point4D.cols/2)) {

        w = point4D.at<double>(3,c);
        x = point4D.at<double>(0,c)/w;
        y = point4D.at<double>(1,c)/w;
        z = point4D.at<double>(2,c)/w;

        cout << x << ", " << y << ", " << z << endl;
    }
    */

    //Save
    saveMat(fn_tr_data_cam,point4Dcam);
    saveMat(fn_tr_data_proj,point4Dproj);

    saveSTL("../triangule/outcam.stl",point4Dcam,pointIdxCam);
    saveSTL("../triangule/outproj.stl",point4Dproj,pointIdxProj);
}


void triangulation::setPathT(int idx,string path,const char *filename) {

    string newfilename = path + (string) filename;

    switch( idx ) {
        case IDX_TR_MASK_CAM : fn_tr_mask_cam=strdup(newfilename.c_str());break;
        case IDX_TR_MASK_PROJ : fn_tr_mask_proj=strdup(newfilename.c_str());break;
        case IDX_TR_DATA_CAM : fn_tr_data_cam=strdup(newfilename.c_str());break;
        case IDX_TR_DATA_PROJ : fn_tr_data_proj=strdup(newfilename.c_str());break;
        case IDX_TR_PARC : fn_tr_parc=strdup(newfilename.c_str());break;
        case IDX_TR_PARP : fn_tr_parp=strdup(newfilename.c_str());break;
        default: printf("setPathT: code %d inconnu!!!!!!!!!\n",idx); break;
    }
}

