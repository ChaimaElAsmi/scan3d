//#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

#include <leopard.hpp>
#include <triangulation.hpp>
#include <paths.hpp>



using namespace cv;
using namespace std;



double horloge() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return( (double) tv.tv_sec + tv.tv_usec / 1000000.0);
}


void scanLeopard(string nameCam,  string nameProj, string namelutC, string namelutP,
                 string namemixC, string namemixP, Mat &lutCam, Mat &lutProj,
                 int nb, int quad, int sp, int synchro) {

    printf("----- Scan leopard -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    leopard *L=new leopard();

    // setup les filename
    L->setPathL(IDX_SCAN_MASKC,path,FN_SCAN_MASKC);
    L->setPathL(IDX_SCAN_MEANC,path,FN_SCAN_MEANC);
    L->setPathL(IDX_SCAN_MASKP,path,FN_SCAN_MASKP);
    L->setPathL(IDX_SCAN_MEANP,path,FN_SCAN_MEANP);

    int from=1;

    //Camera: Images / Code simpl
    Mat *imagesCam;
    imagesCam = L->readImagesGray((char *) nameCam.c_str(), from, from+nb-1, -1.0);
    //computeMask(cam, img, nb, seuil, bias, step, xmin, xmax, ymin, ymax)
    L->computeMask(1,imagesCam,nb,0.9,5.0,1,-1,-1,-1,-1); //815,815+20,815,815+20
    L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);

    //Projecteur: Images / Code simple
    Mat *imagesProj;
    imagesProj=L->readImagesGray((char *) nameProj.c_str(), 0, nb-1, -1.0);
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);


    //Cherche la premiere image de la séquence camera
    int posR = 0;
    L->prepareMatch();
    posR = L->findFirstImage();

    srand(time(NULL));

    Mat *imagesCamDecal = new Mat[nb];
    for(int i=0; i<nb; i++)
        imagesCamDecal[i] = imagesCam[(i+posR)%nb];

    //trouvre la précédente ou la suivante
    int compteur;
    compteur = L->findPrevNext(imagesCamDecal, imagesProj, quad);

    //Choix du mix
    L->mix(imagesCamDecal, imagesProj, compteur, quad, sp, synchro);

    Mat mixCam;
    Mat mixProj;
    L->makeLUT(lutCam,mixCam,1);
    L->makeLUT(lutProj,mixProj,0);
    imwrite(namelutC, lutCam);
    imwrite(namelutP, lutProj);
    imwrite(namemixC, mixCam);
    imwrite(namemixP, mixProj);

    delete[] imagesCam;
    delete[] imagesCamDecal;
    delete[] imagesProj;
    delete L;
    printf("----- leopard done -----\n");
}


int main(int argc, char *argv[]) {

    //Créer des directory pour stocker les output
    string dir = "mkdir -p "
                 +path+CAP+" "
                 +path+LUT+" "
                 +path+MASK+" "
                 +path+TRG;
    system(dir.c_str());


    Mat lutCam;
    Mat lutProj;

    string nameCam  = path+FN_CAP_CAM;
    string nameProj = path+FN_CAP_PROJ;

    int doScan=0;
    int doTriangule=1;
    int quad=1; //quadratic code = 1 , linear code = 0
    int nb=60; //nombres d'images
    int synchro=0;
    int doSp=0;

    // options
    for(int i=1;i<argc;i++) {

        printf("--%s\n",argv[i]);
        if( strcmp("-h",argv[i])==0 ) {
            printf("Usage: %s [-h] [-scan|-triangule]\n",argv[0]);
            exit(0);
        }else if( strcmp("-scan",argv[i])==0 ) {
            doScan=1;continue;
        }else if( strcmp("-triangule",argv[i])==0 ) {
            doTriangule=1;continue;
        }else if( strcmp("-sp",argv[i])==0 ) {
            doSp=1;continue;
        }
    }


    /* ----------------------- Scan 3D ----------------------- */
    if( doScan ) {
        scanLeopard(nameCam, nameProj, (path+FN_SCAN_LUTC), (path+FN_SCAN_LUTP),
                    (path+FN_SCAN_MIXC), (path+FN_SCAN_MIXP), lutCam, lutProj,
                    nb, quad, doSp, synchro);
    }
    else {
        printf("----- Pas de scan -----\n");

        lutCam  = imread((path+FN_SCAN_LUTC) , CV_LOAD_IMAGE_UNCHANGED);
        lutProj =  imread((path+FN_SCAN_LUTP), CV_LOAD_IMAGE_UNCHANGED);
    }

    /* ----------------------- Triangulation ----------------------- */
    if( doTriangule ) {
        triangulation *T = new triangulation();

        string pathvide="";
        //Paths
        T->setPathT(IDX_TR_MASK,path,FN_TR_MASK);
        T->setPathT(IDX_TR_DATA,path,FN_TR_DATA);
        T->setPathT(IDX_TR_PARC,path,FN_TR_PARC);
        T->setPathT(IDX_TR_PARP,path,FN_TR_PARP);

        T->triangulate(lutCam, lutProj);
        delete T;
    }

    return 0;
}
