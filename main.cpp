#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>

#include <leopard.hpp>


using namespace cv;
using namespace std;


double horloge() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return( (double) tv.tv_sec + tv.tv_usec / 1000000.0);
}


void testLeopardSeb() {
    printf("----- test leopard -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    leopard *L=new leopard();
    /// lire des images
    int nb=20;
    Mat *imagesCam;
    imagesCam=L->readImages((char *)"data/cam1/cam%03d.jpg",0,nb-1);
    L->computeMask(1,imagesCam,nb,1.45,5.0,1,0,0);
    //L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    imagesProj=L->readImages((char *)"data/proj1/leopard_2560_1080_32B_%03d.jpg",0,nb-1);
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,0,0);
    //L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    L->prepareMatch();
    //L->forceBrute();
    for(int i=0;i<40;i++) {
        L->doLsh();
        L->doHeuristique();
    }

    cv::Mat lutCam;
    cv::Mat lutProj;
    L->makeLUT(lutCam,1);
    L->makeLUT(lutProj,0);

    imwrite("lutcam.png",lutCam);
    imwrite("lutproj.png",lutProj);


    printf("test\n");
    delete L;
    printf("----- done -----\n");
}





void testLeopardChaima() {
    printf("----- test leopard -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    leopard *L=new leopard();
    /// lire des images
    int nb=30;
    int from=25;
    Mat *imagesCam;
    imagesCam=L->readImages((char *)"data/cam2/cam_%03d.jpg",from,from+nb-1);
    //seuil = 1.45
    L->computeMask(1,imagesCam,nb,1,5.0,1,0,0);
    //L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    //proj2/leopard_1280_720_%03d
    //proj1/leopard_2560_1080_32B_
    imagesProj=L->readImages((char *)"data/proj2/leopard_1280_720_%03d.jpg",0,nb-1);
    L->computeMask(0,imagesProj,nb,1,5.0,1,0,0);
    //L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    L->prepareMatch();
    //L->forceBrute();
    for(int i=0;i<30;i++) L->doLsh();

    cv::Mat lutCam;
    cv::Mat lutProj;
    L->makeLUT(lutCam,1);
    L->makeLUT(lutProj,0);

    imwrite("lsh/lut2/lutcam.png",lutCam);
    imwrite("lsh/lut2/lutproj.png",lutProj);


    printf("test\n");
    delete L;
    printf("----- done -----\n");
}




int main(int argc, char *argv[]) {

    int nbImages = 100;
    Mat img[nbImages];


    // options
    for(int i=1;i<argc;i++) {
		if( strcmp("-h",argv[i])==0 ) {
			printf("Usage: %s -h\n",argv[0]);
			exit(0);
		}
    }



    testLeopardSeb();
    //testLeopardChaima();
    exit(0);

    VideoCapture cap(1);
    if(!cap.isOpened())
        return -1;

    for(int i = 0; i < 30; i++)
        cap >> img[0]; //Démarrer la caméra

    cout << "Camera ready" << endl;

    srand(time(NULL));
    int random = rand() % 500000;
    cout << "Camera wait (random) : " << random << endl;
    usleep(random);

    double timeS = horloge();
    //Capture
    for(int i = 0; i < nbImages; i++)
        cap >> img[i];

    double timeE = horloge();
    cout << "Time (30 images) : " << ((timeE - timeS) / nbImages) * 30 << endl;

    namedWindow("Display Image", 1);

    //Save and Show
    for(int i = 0; i < nbImages; i++) {
        imwrite( format("/home/chaima/Documents/Workspace/CaptureVid2/Images/testGit/image_%04d.jpg",
        i), img[i] );

        imshow("Display Image", img[i]);

        waitKey(30);
    }

    return 0;
}
