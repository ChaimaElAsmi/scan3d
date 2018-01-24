#include <QCoreApplication>
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


void testLeopardSeb() {
    printf("----- test leopard seb -----\n");

    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    leopard *L=new leopard();

    // setup les output
    string pathscan = "";
    L->setPathL(IDX_SCAN_MASKC,pathscan,"maskcam.png");
    L->setPathL(IDX_SCAN_MEANC,pathscan,"meancam.png");
    L->setPathL(IDX_SCAN_MASKP,pathscan,"maskproj.png");
    L->setPathL(IDX_SCAN_MEANP,pathscan,"meanproj.png");

    /// lire des images
    int nb=40;
    Mat *imagesCam;
    imagesCam=L->readImages((char *)"data/cam1/cam%03d.jpg",0,nb-1, -1.0);
    imwrite("cam0-nonoise.png",imagesCam[0]);
    // noise  0 : p(erreur)=0.058
    // noise 10 : p(erreur)=0.078
    // noise 20 : p(erreur)=??
    //L->noisify(imagesCam,nb,20.0,0.0);
    imwrite("cam0-noise.png",imagesCam[0]);

    L->computeMask(1,imagesCam,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    //L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    imagesProj=L->readImages((char *)"data/proj1/leopard_2560_1080_32B_%03d.jpg",0,nb-1, -1.0);
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    //L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    // quelques stats
    //L->statsCodes(1);
    //L->statsCodes(0);

    L->prepareMatch();
    //L->forceBrute();
    for(int i=0;i<50;i++) {
        printf("--- %d ---\n",i);
        L->doLsh(0,0);
        //L->doHeuristique();
    }

    cv::Mat lutCam;
    cv::Mat lutProj;
    cv::Mat mixCam;
    cv::Mat mixProj;
    L->makeLUT(lutCam,mixCam,1);
    L->makeLUT(lutProj,mixProj,0);

    imwrite("lutcam.png",lutCam);
    imwrite("lutproj.png",lutProj);


    printf("test\n");
    delete L;
    printf("----- done -----\n");
}


int doSimple(int nb,char *camName,char *refName) {
    printf("----- match leopard simple -----\n");

    leopard *L=new leopard();

    // setup les output
    L->setPath(IDX_SCAN_MASKC,"maskcam.png");
    L->setPath(IDX_SCAN_MEANC,"meancam.png");
    L->setPath(IDX_SCAN_MASKP,"maskproj.png");
    L->setPath(IDX_SCAN_MEANP,"meanproj.png");

    /// lire des images
    Mat *imagesCam;
    imagesCam=L->readImages((char *)camName,0,nb-1, -1.0);
    if( imagesCam==NULL ) {
        printf("*** impossible de lire les images %s de 0 a %d\n",camName,nb-1);
        return(-1);
    }
    
    L->computeMask(1,imagesCam,nb,0.1,5.0,1,970,1070,240,400); // toute l'image
    //L->computeMask(1,imagesCam,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    //L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);
    L->computeCodes(1,LEOPARD_QUADRATIC,imagesCam);
    delete[] imagesCam;

    Mat *imagesProj;
    imagesProj=L->readImages(refName,0,nb-1, -1.0);
    if( imagesProj==NULL ) {
        printf("*** impossible de lire les images %s de 0 a %d\n",refName,nb-1);
        return(-1);
    }
    // normalement -1 pour ne pas mettre de limite
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,600,680,160,660); // toute l'image
    //L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);
    L->computeCodes(0,LEOPARD_QUADRATIC,imagesProj);
    delete[] imagesProj;

    L->prepareMatch();
    L->forceBruteProj(0,0);
    // algo normal plus rapide
    /*
    for(int i=0;i<200;i++) {
        printf("--- %d ---\n",i);
        L->doLsh(0,0);
    }
    */

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

void testLeopardChaima(string nameCam,  string nameProj,
                       string namelutC, string namelutP,
                       string namemixC, string namemixP,
                       Mat *imgCam, Mat &lutCam, Mat &lutProj, int sp, int synchro) {
    printf("----- test leopard chaima -----\n");


    printf("sizeof char %d\n",(int)sizeof(char));
    printf("sizeof short %d\n",(int)sizeof(short));
    printf("sizeof int %d\n",(int)sizeof(int));
    printf("sizeof long %d\n",(int)sizeof(long));
    printf("sizeof long long %d\n",(int)sizeof(long long));

    double timeS = horloge();

    leopard *L=new leopard();

    // setup les filename
    L->setPathL(IDX_SCAN_MASKC,path,FN_SCAN_MASKC);
    L->setPathL(IDX_SCAN_MEANC,path,FN_SCAN_MEANC);
    L->setPathL(IDX_SCAN_MASKP,path,FN_SCAN_MASKP);
    L->setPathL(IDX_SCAN_MEANP,path,FN_SCAN_MEANP);

    int nb = 60;
    int from = 0;//100;
    //Camera: Images / Code simple
    Mat *imagesCam;
    if(imgCam->rows != 0) {
        imagesCam = L->readImages2(imgCam, from, from+nb-1);
    }
    else {
        imagesCam = L->readImages((char *) nameCam.c_str(), from, from+nb-1, -1.0);
    }
    L->computeMask(1,imagesCam,nb,0.65,5.0,1,-1,-1,-1,-1); //815,815+20,815,815+20
    L->computeCodes(1,LEOPARD_SIMPLE,imagesCam);

    //Projecteur: Images / Code simple
    Mat *imagesProj;
    imagesProj=L->readImages((char *) nameProj.c_str(), 0, nb-1, -1.0);
    L->computeMask(0,imagesProj,nb,1.45,5.0,1,-1,-1,-1,-1); // toute l'image
    L->computeCodes(0,LEOPARD_SIMPLE,imagesProj);


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
        L->doLsh(0,0);

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
        L->doLsh(0,0);

    sumCostP = L->sumCost();

    printf("\n sumSuivante = %d, sumPrécédente = %d \n",sumCostS, sumCostP);

    double timeSM = horloge();

    //Choix du mix
    L->prepareMatch();
    if(sumCostS < sumCostP) {
        printf("\n match avec la suivante ! \n");
        for(double fct=0; fct<=1; fct+=0.1) {
            printf("\n\n---------------------- facteur = %.2f ----------------------\n\n", fct);

            for(int i=0; i<nb-1; i++)
                imagesProjMix[i] = imagesProj[i]*(1-fct) + imagesProj[i+1]*fct;
            imagesProjMix[nb-1] = imagesProj[nb-1];

            //QUAD
            L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
            L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

            //TEST: pas de cumul
            //L->prepareMatch();
            for(int j=0; j<20; j++)
                L->doLsh(sp,(int) (fct*255));


            //L->forceBrute(sp,(int) (fct*255));
            if(synchro)
                break;
        }
    }
    else {
        printf("\n match avec la précédente ! \n");
        for(double fct=0; fct<=1; fct+=0.1) {
            printf("\n\n---------------------- facteur = %.2f ----------------------\n\n", fct);

            for(int i=nb-1; i>0; i--)
            imagesProjMix[i] = imagesProj[i]*(1-fct) + imagesProj[i-1]*fct;
            imagesProjMix[0] = imagesProj[0];

            //QUAD
            L->computeCodes(1,LEOPARD_QUADRATIC,imagesCamDecal);
            L->computeCodes(0,LEOPARD_QUADRATIC,imagesProjMix);

            //TEST: pas de cumul
            //L->prepareMatch();
            for(int j=0; j<20; j++)
                L->doLsh(sp,(int) (fct*255));


            //L->forceBrute(sp,(int) (fct*255));
            if(synchro)
                break;
        }
    }

    double timeEM = horloge();
    //L->forceBrute();

    Mat mixCam;
    Mat mixProj;
    L->makeLUT(lutCam,mixCam,1);
    L->makeLUT(lutProj,mixProj,0);
    imwrite(namelutC, lutCam);
    imwrite(namelutP, lutProj);
    imwrite(namemixC, mixCam);
    imwrite(namemixP, mixProj);


    double timeE = horloge();
    printf("\n Time Scan = %f \n", timeE-timeS);
    printf("\n Time match mix = %f \n", timeEM-timeSM);

    delete[] imagesCam;
    delete[] imagesCamDecal;
    delete[] imagesProj;
    delete[] imagesProjMix;
    delete L;
    printf("----- leopard done -----\n");
}


int main(int argc, char *argv[]) {

    int nbImages =300;
    Mat img[nbImages];

    Mat lutCam;
    Mat lutProj;

    string nameCam  = path+FN_CAP_CAM;
    string nameProj = path+FN_CAP_PROJ;

    //Créer des directory pour stocker les images
    string dir = "mkdir "+path+" "+path+
                 "output "+path+"output/scan "+path+
                 "output/scan/lut "+path+
                 "output/scan/mask "+path+
                 "output/triangulation";
    system(dir.c_str());


    // qui est l'usager??
    char *user=getenv("USER");
    printf("Usager %s\n",user);

    int doCapture=1;
    int doScan=1;
    int doTriangule=0;
    int doSp=0;
    int synchro=0;

    if( strcmp(user,"roys")==0 ) {
        // initialisations juste pour sebastien
        doCapture=0;
        doScan=1;
        doTriangule=0;
    }else{
        // initialisations juste pour chaima
        doCapture=0;
        doScan=1;
        doTriangule=0;
        doSp=0;
    }

    // options
    for(int i=1;i<argc;i++) {
        printf("....%s\n",argv[i]);
        if( strcmp("-h",argv[i])==0 ) {
            printf("Usage: %s [-h] [-capture|-scan|-triangule] ou [-simple 60 cam%03d.png ref%03d.png]\n",argv[0]);
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
            nbImages=60;
            synchro=1;continue;
        }else if( strcmp("-simple",argv[i])==0 && i+4<argc ) {
            // simple match entre deux ensembles d'images
            // -simple 60 cam%03d.png ref%03d.png camMask.png
            doSimple(atoi(argv[i+1]),argv[i+2],argv[i+3]);
            exit(0);
        }
    }


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

//        srand(time(NULL));
//        int random = rand() % 500000;
//        cout << "random : " << random << endl;
//        usleep(random);
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

        }



        double timeS = horloge();
        if(synchro){
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
        }else{

            for(int i = 0; i < nbImages; i++) {
                cap >> img[i];
                printf("img %d \n",i);
                //resize(img[i], resized[i], Size(640,480)); //(683,384)
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

        printf("----- Capture done -----\n");
        printf("\n\n");
    }

    /* ----------------------- Scan 3D ----------------------- */
    if( doScan ) {

        if( strcmp(user,"roys")==0 ) {
            testLeopardSeb();
        }else if( strcmp(user,"chaima")==0 ) {
            testLeopardChaima(nameCam, nameProj,
                              (path+FN_SCAN_LUTC), (path+FN_SCAN_LUTP),
                              (path+FN_SCAN_MIXC), (path+FN_SCAN_MIXP),
                              img, lutCam, lutProj, doSp, synchro);
        }
    }
    else {
        printf("----- Pas de scan -----\n");

        lutCam  = imread((path+FN_SCAN_LUTC) , CV_LOAD_IMAGE_UNCHANGED);
        lutProj =  imread((path+FN_SCAN_LUTP), CV_LOAD_IMAGE_UNCHANGED);
    }

    /* ----------------------- Triangulation ----------------------- */
    if( doTriangule ) {
        triangulation *T=new triangulation();

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
