#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>


#include <leopard.hpp>
#include <triangulation.hpp>


using namespace cv;
using namespace std;



double horloge() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return( (double) tv.tv_sec + tv.tv_usec / 1000000.0);
}


void testLeopardSeb() {
    printf("----- test leopard seb -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    leopard *L=new leopard();
    /// lire des images
    int nb=40;
    Mat *imagesCam;
    imagesCam=L->readImages((char *)"data/cam1/cam%03d.jpg",0,nb-1, -1.0);
    L->computeMask(1,imagesCam,nb,1.45,5.0,1,0,0);
    //L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    imagesProj=L->readImages((char *)"data/proj1/leopard_2560_1080_32B_%03d.jpg",0,nb-1, -1.0);
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,0,0);
    //L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    // quelques stats
    L->statsCodes(1);
    L->statsCodes(0);

    L->prepareMatch();
    //L->forceBrute();
    for(int i=0;i<20;i++) {
        L->doLsh();
        //L->doHeuristique();
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


//horiz = decal en x ou y // horiz = 1 -> x ; horiz = 0 -> y
Mat* souspixelsImage(Mat* source, int n, float f,int horiz) {
    Mat* dest=new Mat[n];

    int w = source[0].cols;
    int h = source[0].rows;

    int dx=0;
    int dy=0;
    if(horiz) dx=1;
    else      dy=1;

    int d=dx+dy*w;
    for(int i=0; i<n; i++) {
        dest[i] = source[i].clone();
        unsigned char *p=dest[i].data;
        for(int y=0; y<h; y++) {
            for(int x=0; x<w; x++) {
                if(x < w-dx && y < h-dy)  *p = p[0]*(1-f) + p[d]*f;
                p++;
            }
        }
    }
     return dest;
}


void testLeopardChaima(string nameCam, string nameProj, Mat *imgCam, Mat &lutCam, Mat &lutProj) {
    printf("----- test leopard chaima -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    double timeS = horloge();

    leopard *L=new leopard();
    int nb = 60;
    int from = 0;//20;
    //Camera: Images / Code simple
    Mat *imagesCam;
    imagesCam = L->readImages((char *) nameCam.c_str(), from, from+nb-1, -1.0);
    //Auto
//    imagesCam = L->readImages2(imgCam, from, from+nb-1);
    L->computeMask(1,imagesCam,nb,0.45,5.0,1,0,0);
    L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);

    //Projecteur: Images / Code simple
    Mat *imagesProj;
    imagesProj=L->readImages((char *) nameProj.c_str(), 0, nb-1, -1.0);
    L->computeMask(0,imagesProj,nb,1,5.0,1,0,0);
    L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);


    //Souspixels
    Mat *imagesProjSP;
//    imagesProjSP = souspixelsImage(imagesProj, nb, 0.5, 0);
//    Mat *imagesProjSPxy;
//    imagesProjSPxy = souspixelsImage(imagesProjSP, nb, 0.5, 1);
//    imwrite("source.png",imagesProj[0]);
//    imwrite("dest.png",imagesProjSPxy[0]);



    //Cherche la premiere image de la séquence camera
    int posR = 0;
    L->prepareMatch();
    posR = L->doShiftCodes();

    Mat *imagesCamDecal = new Mat[nb];
    for(int i=0; i<nb; i++)
        imagesCamDecal[i] = imagesCam[(i+posR)%nb];


    //Cherche le mix des images du projecteur
    Mat *imagesProjMix=new Mat[nb];
    int sumCostS=0, sumCostP=0;

    //Match avec l'image suivante
    L->prepareMatch();
    for(int i=0; i<nb-1; i++)
        imagesProjMix[i] = imagesProj[i]*0.5 + imagesProj[i+1]*0.5;
    imagesProjMix[nb-1] = imagesProj[nb-1];

    //QUAD
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

    for(int j=0; j<10; j++)
        L->doLsh();

    sumCostS = L->sumCost();

    //Match avec la précédente
    L->prepareMatch();
    for(int i=nb-1; i>0; i--)
        imagesProjMix[i] = imagesProj[i]*0.5 + imagesProj[i-1]*0.5;
    imagesProjMix[0] = imagesProj[0];

    //QUAD
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

    for(int j=0; j<10; j++)
        L->doLsh();

    sumCostP = L->sumCost();

    printf("\n sumSuivante = %d, sumPrécédente = %d \n",sumCostS, sumCostP);

    //Test Souspixels
    float k=0;
//    while(k < 1) {
//        printf("\n\n\n\n--------------------------------------------");
//        printf("\n itération k = %.2f \n", k);
        imagesProjSP = souspixelsImage(imagesProj, nb, k, 0);

    //Choix du mix
    L->prepareMatch();
    if(sumCostS < sumCostP) {
        printf("\n match avec la suivante ! \n");
        for(double fct=0; fct<=1; fct+=0.3) {
            //fct=0.9;
            printf("\n\n---------------------- facteur = %.2f ----------------------\n\n", fct);

            for(int i=0; i<nb-1; i++)
                imagesProjMix[i] = imagesProjSP[i]*(1-fct) + imagesProjSP[i+1]*fct;
            imagesProjMix[nb-1] = imagesProjSP[nb-1];

            //QUAD
            L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
            L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

            //TEST: pas de cumul
            //L->prepareMatch();
            for(int j=0; j<10; j++)
                L->doLsh();

            //L->makeLUT(lutCam,1);
            //L->makeLUT(lutProj,0);

            //imwrite(format("lsh/mix/cumul2/lutcam_%.2f.png", fct),lutCam);
            //imwrite(format("lsh/mix/cumul2/lutproj_%.2f.png", fct),lutProj);
            //break; //Temporaire!
        }
    }
    else {
        printf("\n match avec la précédente ! \n");
        for(double fct=0; fct<=1; fct+=0.3) {
            //fct=0.9;
            printf("\n\n---------------------- facteur = %.2f ----------------------\n\n", fct);

            for(int i=nb-1; i>0; i--)
                imagesProjMix[i] = imagesProjSP[i]*(1-fct) + imagesProjSP[i-1]*fct;
            imagesProjMix[0] = imagesProjSP[0];

            //QUAD
            L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
            L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

            //TEST: pas de cumul
            //L->prepareMatch();
            for(int j=0; j<10; j++)
                L->doLsh();

            //L->makeLUT(lutCam,1);
            //L->makeLUT(lutProj,0);

            //imwrite(format("lsh/mix/cumul2/lutcam_%.2f.png", fct),lutCam);
            //imwrite(format("lsh/mix/cumul2/lutproj_%.2f.png", fct),lutProj);
            //break; //Temporaire!
        }
    }

    //L->forceBrute();

    L->makeLUT(lutCam,1);
    L->makeLUT(lutProj,0);
    imwrite("calibration/LUT/visage/lutcam_vis.png",lutCam);
    imwrite("calibration/LUT/visage/lutproj_vis.png",lutProj);

//    imwrite(format("lsh/SP/objet_60Y/lutcam_%.2f.png", k),lutCam);
//    imwrite(format("lsh/SP/objet_60Y/lutproj_%.2f.png", k),lutProj);

//        k+=0.1;
//    }

    double timeE = horloge();
    printf("\n Time = %f \n", timeE-timeS);

    delete[] imagesProjSP;
    delete[] imagesCam;
    delete[] imagesCamDecal;
    delete[] imagesProj;
    delete[] imagesProjMix;
    delete L;
    printf("----- leopard done -----\n");
}


int main(int argc, char *argv[]) {

    int capture   = 0;
    int scan      = 1;
    int triangule = 0;


    int nbImages = 300;
    Mat img[nbImages];

    //Choix des datas
    int cam = 2;
    string nameCam, nameProj;
    Mat lutCam;
    Mat lutProj;
    if(cam == 1) {
        nameCam  = "data/cam1/cam%03d.jpg";
        nameProj = "data/proj1/leopard_2560_1080_32B_%03d.jpg";
    }
    else {
        nameCam  = "data/test/cam_%04d.jpg";
                //"data/objetplanaire/cam_%04d.jpg";
                //"/home/chaima/Documents/Mathematica/Images/1280x720/Patterns_60/"
                  // "/pat_modif/leopardModif_1280_720_%04d.jpg";
        //"data/objet/images_60/cam_%04d.jpg";
        //"data/test/cam_%04d.jpg";
        nameProj = "/home/chaima/Documents/Mathematica/Images/1280x720/Patterns_60/leopard_1280_720_%04d.jpg";
    }


    /* ----------------------- Capture ----------------------- */
    if(capture) {

        printf("----- Capture -----\n");

        VideoCapture cap(1);

        for(int i = 0; i < 30; i++)
            cap >> img[0]; //Démarrer la caméra

        if(!cap.isOpened()) {
            cout << "Camera error" << endl;
            return -1;
        }
        else {
            cout << "Camera ready" << endl;
        }

        srand(time(NULL));
        int random = rand() % 500000;
        cout << "random : " << random << endl;
        usleep(random);

        double timeS = horloge();
        for(int i = 0; i < nbImages; i++) {
            cap >> img[i];
            //resize(img[i], resized[i], Size(640,480)); //(683,384)
        }
        double timeE = horloge();

        cout << "Time : " << (timeE - timeS) / nbImages << endl;
        cout << "Time (" << nbImages << " images): " << (timeE - timeS) << endl;
        waitKey(0);

        namedWindow("Display Image", 1);
        //Changer la fenetre d'affichage
    //    resizeWindow("Display Image", 800, 600);
    //    moveWindow("Display Image", 1366, 0);

        for(int i = 0; i < nbImages; i++) {
            imwrite( format(nameCam.c_str(), i), img[i] );
            imshow("Display Image", img[i]);
            waitKey(30);
        }

        printf("----- Capture done -----\n");
        printf("\n\n");
    }

    /* ----------------------- Scan 3D ----------------------- */
    if(scan) {
        char *user=getenv("USER");
        printf("Usager %s\n",user);
        printf("\n\n");


        // options
        for(int i=1;i<argc;i++) {
            if( strcmp("-h",argv[i])==0 ) {
                printf("Usage: %s -h\n",argv[0]);
                exit(0);
            }
        }

        if( strcmp(user,"roys")==0 ) {
            testLeopardSeb();
        }else if( strcmp(user,"chaima")==0 ) {
            testLeopardChaima(nameCam, nameProj, img, lutCam, lutProj);
        }
    }
    else {
        printf("----- Pas de capture -----\n");

        lutCam  = imread("calibration/LUT/visage/lutcam_vis.png", CV_LOAD_IMAGE_UNCHANGED);
        lutProj =  imread("calibration/LUT/visage/lutproj_vis.png", CV_LOAD_IMAGE_UNCHANGED);
    }

    /* ----------------------- Triangulation ----------------------- */
    if(triangule) {
        triangulate(lutCam, lutProj);
    }

    return 0;
}
