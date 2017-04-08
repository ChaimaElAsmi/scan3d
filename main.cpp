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


void testLeopard() {
    printf("----- test leopard -----\n");
    leopard *L=new leopard();
    /// lire des images
    Mat *images;
    images=L->readImages((char *)"data/cam1/cam%03d.jpg",0,29);


    Mat iMin,iMax;
    L->computeMinMax(images,30,iMin,iMax);

    imwrite("minCam.png",iMin);
    imwrite("maxCam.png",iMax);

    delete[] images;




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



    testLeopard();
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
