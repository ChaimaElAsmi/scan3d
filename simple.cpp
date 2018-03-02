//#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>



#include <leopard.hpp>
#include <triangulation.hpp>
#include <paths.hpp>

/*
 *
 * ./simple -simple 60 /home/roys/projet/gopro/dataset-2016-05-12-6fisheyes/cam4/n180deg/cam%03d.png /home/roys/git-chaima/scan3d-seb-older/data/proj1/leopard_2560_1080_32B_%03d.jpg
 *
 *
 */


using namespace cv;
using namespace std;



double horloge() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return( (double) tv.tv_sec + tv.tv_usec / 1000000.0);
}


int doSimple(int nb,char *camName,char *refName) {
    printf("----- match leopard simple -----\n");

    leopard *L=new leopard();

    // setup les output
    string pathscan = "";
    L->setPathL(IDX_SCAN_MASKC,pathscan,"maskcam.png");
    L->setPathL(IDX_SCAN_MEANC,pathscan,"meancam.png");
    L->setPathL(IDX_SCAN_MASKP,pathscan,"maskproj.png");
    L->setPathL(IDX_SCAN_MEANP,pathscan,"meanproj.png");

    /// lire des images
    Mat *imagesCam;
    imagesCam=L->readImagesGray((char *)camName,0,nb-1, -1.0);
    if( imagesCam==NULL ) {
        printf("*** impossible de lire les images %s de 0 a %d\n",camName,nb-1);
        return(-1);
    }
    
    //L->computeMask(1,imagesCam,nb,0.1,5.0,1,970,1070,240,400); // toute l'image
    L->computeMask(1,imagesCam,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    //L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    imagesProj=L->readImagesGray(refName,0,nb-1, -1.0);
    if( imagesProj==NULL ) {
        printf("*** impossible de lire les images %s de 0 a %d\n",refName,nb-1);
        return(-1);
    }
    // normalement -1 pour ne pas mettre de limite
    //L->computeMask(0,imagesProj,nb,1.45,5.0,1,600,680,160,660); // toute l'image
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    //L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    L->prepareMatch();
    //L->forceBruteProj(0,0);
    // algo normal plus rapide
    for(int i=0;i<10;i++) {
        printf("--- %d ---\n",i);
        L->doLsh(0,0);
    }

    cv::Mat lutCam;
    cv::Mat lutProj;
    cv::Mat mixCam;
    cv::Mat mixProj;
    L->makeLUT(lutCam,mixCam,1);
    L->makeLUT(lutProj,mixProj,0);

    imwrite("lutcam.png",lutCam);
    imwrite("lutproj.png",lutProj);

    delete L;
    printf("----- done -----\n");
    return 0;
}

int main(int argc, char *argv[]) {

#ifdef USE_THREADS
    printf("USING THREADS!!!!\n");
#endif

    // options
    for(int i=1;i<argc;i++) {
        printf("arg[%d]=%s\n",i,argv[i]);
        if( strcmp("-h",argv[i])==0 ) {
            printf("Usage: %s [-h] -simple 60 cam%%03d.png ref%%03d.png\n",argv[0]);
            exit(0);
        }else if( strcmp("-simple",argv[i])==0 && i+3<argc ) {
            // simple match entre deux ensembles d'images
            // -simple 60 cam%03d.png ref%03d.png camMask.png
            doSimple(atoi(argv[i+1]),argv[i+2],argv[i+3]);
            i+=3;continue;
        }
    }


    return 0;
}
