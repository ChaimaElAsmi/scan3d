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
                 string namemixC, string namemixP, Mat *imgCam, Mat &lutCam, Mat &lutProj,
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

    int from;
    if(synchro)
        from = 0;
    else
        from = 50;

    //Camera: Images / Code simple
    Mat *imagesCam;
    if(imgCam->rows != 0) {
        imagesCam = L->convertToGray(imgCam, from, from+nb-1);
    }
    else {
        imagesCam = L->readImagesGray((char *) nameCam.c_str(), from, from+nb-1, -1.0);
    }
    //computeMask(cam, img, nb, seuil, bias, step, xmin, xmax, ymin, ymax)
    L->computeMask(1,imagesCam,nb,0.6,5.0,1,-1,-1,-1,-1); //815,815+20,815,815+20
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


    int doCapture=0;
    int doScan=1;
    int doTriangule=0;
    int doSp=0;
    int synchro=0;
    int quad=1; //quadratic code = 1 , linear code = 0
    int nb=60; //nombres d'images
    int nbImages;



    // options
    for(int i=1;i<argc;i++) {
        printf("->%s\n",argv[i]);
        if( strcmp("-h",argv[i])==0 ) {
            printf("Usage: %s [-h] [-capture|-scan|-triangule]\n",argv[0]);
            exit(0);
        }else if( strcmp("-capture",argv[i])==0 ) {
            doCapture=1;continue;
        }else if( strcmp("-scan",argv[i])==0 ) {
            doScan=1;continue;
        }else if( strcmp("-triangule",argv[i])==0 ) {
            doTriangule=1;continue;
        }else if( strcmp("-sp",argv[i])==0 ) {
            doSp=1;continue;
        }else if( strcmp("-synchro",argv[i])==0 ) {
            synchro=1;continue;
        }
    }


    if(synchro)
        nbImages=nb;
    else
        nbImages=300;

    Mat img[nbImages];

    /* ----------------------- Capture ----------------------- */
    if( doCapture ) {

        printf("----- Capture -----\n");

        VideoCapture cap(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
        cap.set(CV_CAP_PROP_FPS,30);

        for(int i = 0; i < 30; i++)
            cap >> img[0]; //Démarrer la caméra

        if(!cap.isOpened()) {
            cout << "Camera error" << endl;
            return -1;
        }
        else {
            cout << "Camera ready" << endl;
        }

        double timeS = horloge();

        int offset=0;
        if(synchro){
            //On suppose que les images sont répétées 5 fois
            double somme[5];
            Mat last,cur,tmp;
            cap >> last;
            cvtColor(last,last,CV_RGB2GRAY);
            Scalar s;
            for(int i=0;i<5;i++)
                somme[i]=0;
            for(int i=0; i<100;i++){
                cap >> cur;
                cvtColor(cur,cur,CV_RGB2GRAY);
                absdiff(last,cur,tmp);
//                if(i==10) {
//                    imwrite("cur.png",cur);
//                    imwrite("last.png",last);
//                    imwrite("tmp.png",tmp);
//                }
                s = sum(tmp);
                //printf("s %d %d %f \n",i%5,i,s.val[0]/1920.0/1080.0);
                somme[i%5]+=s.val[0]/1920.0/1080.0;
                last=cur.clone();
            }

            for(int i=0;i<5;i++){
                if(somme[i]<somme[offset])
                    offset=i;
                printf("%d %f \n",i,somme[i]);
            }
            printf("offset = %d \n",offset);

        //capture
            Mat vide;
            int i,j;
            for(i=0,j=0; j < nbImages; i++) {
                if(i%5==offset) {
                    cap >> img[j];
                    j++;
                }
                else
                    cap >> vide;
            }
        } else{

            for(int i = 0; i < nbImages; i++) {
                cap >> img[i];
                printf("img %d \n",i);
            }
        }
        double timeE = horloge();
        double xs, ys;
        xs = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        ys = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        cout << "x = " << xs << "   y = " << ys << endl;

        cout << "Time : " << (timeE - timeS) / nbImages << endl;
        cout << "Time (" << nbImages << " images): " << (timeE - timeS) << endl;
        waitKey(0);

        namedWindow("Display Image", 1);
        for(int i = 0; i < nbImages; i++) {
            imwrite( format(nameCam.c_str(), i), img[i] );
            imshow("Display Image", img[i]);
            waitKey(30);
        }
        destroyWindow("Display Image");
        printf("----- Capture done -----\n");
        printf("\n\n");
    }

    /* ----------------------- Scan 3D ----------------------- */
    if( doScan ) {
        scanLeopard(nameCam, nameProj, (path+FN_SCAN_LUTC), (path+FN_SCAN_LUTP),
                    (path+FN_SCAN_MIXC), (path+FN_SCAN_MIXP), img, lutCam, lutProj,
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
        T->setPathT(IDX_TR_PARC,pathvide,FN_TR_PARC);
        T->setPathT(IDX_TR_PARP,pathvide,FN_TR_PARP);

        T->triangulate(lutCam, lutProj);
        delete T;
    }

    return 0;
}
