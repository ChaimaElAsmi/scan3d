#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>


using namespace cv;
using namespace std;


double horloge() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return( (double) tv.tv_sec + tv.tv_usec / 1000000.0);
}


int main(int argc, char *argv[]) {

        int nbImages = 400;
        Mat img[nbImages];

       // Mat resized[nbImages];


        //Plusieurs tests
        //for (int j = 0; j < 4; j++) {

            VideoCapture cap(1);

            for(int i = 0; i < 30; i++) {
                cap >> img[0]; //Démarrer la caméra
            }
            cout << "Pres" << endl;

            waitKey(0);

            if(!cap.isOpened())
                return -1;


            //Video vlc monitor screen
            //system("vlc --qt-fullscreen-screennumber=1 -f '/home/chaima/Documents/Mathematica/Video/sequence5_32.mp4' "
              //     "--play-and-exit --no-video-title-show --aspect-ratio 4:3 &");

            //Video vlc default screen
            // system("vlc '/home/chaima/Documents/Mathematica/Video/sequence3.mp4' "
               //     "-f --play-and-exit --no-video-title-show &");

            //Vidéo mplayer
            //system("mplayer '/home/chaima/Documents/Mathematica/Video/sequence3.mp4' "
              //     "-geometry 800x600+1366+0 -noborder -aspect 4:3 &");


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
            /*resizeWindow("Display Image", 800, 600);
            moveWindow("Display Image", 1366, 0);*/

            for(int i = 0; i < nbImages; i++) {

                //imwrite( format("/home/chaima/Documents/Workspace/CaptureVid2/Images/Seq4/tests/test%d"
                              //  "/image_%04d.jpg", (j + 1), i) , img[i] );

                imwrite( format("/home/chaima/Documents/Workspace/CaptureVid2/Images/Seq6/test7/image_%04d.jpg",
                                  i), img[i] );

                imshow("Display Image", img[i]);

                waitKey(30);

            }
        //}

        return 0;
}
