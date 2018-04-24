#ifndef IMGV_PLUGIN_LEOPARD_HPP
#define IMGV_PLUGIN_LEOPARD_HPP


#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>


//
// definir pour utiliser le vote extra qui simule un bit d'erreur
//
//#define EXTRA_ERREUR_BIT


//
// pour reduire les collisions lsh
//
//#define REDUCTION_COLLISIONS

class minfo {
	public:
	int idx; // index du match dans l'autre image  idx=y*w+x,  y=idx/w, x=idx%w
	unsigned short cost; // cout du match
    float subpx; // souspixels en x
    float subpy; // souspixels en y
    unsigned char mix; // [0..255] pour un mix de [0..1]
	//unsigned short active; // 1 = try to find this match, 0 = do not compute this match.
};


// type de code binaire
#define LEOPARD_SIMPLE  0
#define LEOPARD_QUADRATIC  1

// strings pour les noms de fichier
#define IDX_SCAN_MASKC   0
#define IDX_SCAN_MEANC   1
#define IDX_SCAN_MASKP   2
#define IDX_SCAN_MEANP   3

class leopard {

    //
    // set by computeMask
    //

	int n; // the count of images
	int wc,hc; // width and height of camera image
	int wp,hp; // width and height of projector image

    // [wc*hc] access [y*wc+x]
    unsigned char *maskCam;
    // [wp*hp] access [y*wp+x]
    unsigned char *maskProj;
    
    //
    // set by computeCode
    // (can be called many times, no problem)
    //

    int nbb; // number of bits in the code (ex: 120)
	int nb; // number of long in the code =(nbb+63)/64
	// codes for camera [wc*hc*nb] access [(y*wc+x)*nb+b] ou [i*nb+b]
	unsigned long *codeCam;
	// codes for projector [wp*hp*nb] access [(y*wp+x)*nb+b] ou [i*nb+b]
	unsigned long *codeProj;




	// match for camera [wc*hc] access [y*wc+x] -> index into projector image, and cost
	minfo *matchCam;
	// match for projector [wp*hp] access [y*hp+x] -> index into camera image, and cost
	minfo *matchProj;

	// pour LSH
	int *vote;

	cv::Mat minCam,maxCam;
	cv::Mat minProj,maxProj;

    //SousPixels
    int wDecal;
    int dsx,dsy;
    double *ptsCam,*ptsProj;
    double *ptsCamx,*ptsCamy,*ptsCamxy;

    // filenames
    const char *fn_scan_maskc;
    const char *fn_scan_meanc;
    const char *fn_scan_maskp;
    const char *fn_scan_meanp;

    private:
    cv::Mat *readImagesInterne(char *name, int from, int to, double fct,int flags);

    public:
	leopard(); //int w,int h, int nb, int freq, bool blur,string pid);
	~leopard();

    cv::Mat *readImagesGray(char *name, int from, int to, double fct);
    cv::Mat *readImagesBGR(char *name, int from, int to, double fct);
    cv::Mat *convertToGray(cv::Mat *cam, int from, int to);
    void noisify(cv::Mat *cam,int nb,double stddev=10.0,double mean=0.0);
    void computeMask(int cam,cv::Mat *img,int nb,double seuil,double bias,int step,int xmin,int xmax,int ymin,int ymax);
    void computeCodes(int cam,int type,cv::Mat *img);
    void prepareMatch();
    void forceBrute(int sp, unsigned char mix);
    void forceBruteCam(int sp, unsigned char mix);
    void forceBruteProj(int sp, unsigned char mix);
    void sousPixels();
    void unSousPixels(int i);
    void initSP();
    void unInitSP();
    void makeLUT(cv::Mat &lut, cv::Mat &imgmix, int cam, int select);
    int doLsh(int sp, unsigned char mix);
    int doHeuristique();
    int findFirstImage();
    int sumCost();
    void statsCodes(int cam);
    void setPathL(int idx, std::string path, const char *filename);
    int costPrevNext(cv::Mat *imagesCam, cv::Mat *imagesProj, int quad);
    int findPrevNext(cv::Mat *imagesCam, cv::Mat *imagesProj, int quad);
    void mix(cv::Mat *imagesCam, cv::Mat *imagesProj, int compteur, int quad, int sp, int synchro);

    double squaredSum2(double sx, double sy, int w);
    void alignGrad(double sx, double sy, int w,double *dX,double *dY);
    void alignGrad2(double sx, double sy, int w,double *dX,double *dY);

  private:
    void dumpCode(unsigned long *c);
    void dumpCodeNum(unsigned long *c);
	int cost(unsigned long *a,unsigned long *b);
    double horloge();
    int bitCount(unsigned long n);
    void match2image(cv::Mat &lut,minfo *match,unsigned char *mask,int w,int h,int ww,int hh,int select);
    void mix2image(cv::Mat &imgmix,minfo *match,unsigned char *mask,int w,int h,int ww,int hh);
    int lsh(int dir, unsigned long *codeA, minfo *matchA, unsigned char *maskA, int wa, int ha,
                     unsigned long *codeB, minfo *matchB, unsigned char *maskB, int wb, int hb,
                     int aisCam, unsigned char mix);
	int heuristique( unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
					 unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb);
    void shiftCodes(int shift, unsigned long *codes, int w, int h);



	//unsigned char bitCount[256]; // precomputed bit count

#if 0
	int cost(unsigned char *a,unsigned char *b,int sz);
	void match2opencv(minfo *match,int w,int h,int ww,int hh,int nbpoints,int maxcost,int iterations);
	void forceBrute();
	int lsh( int dir,unsigned char *codeA,minfo *matchA,int wa,int ha,
					 unsigned char *codeB,minfo *matchB,int wb,int hb);
	int heuristique( unsigned char *codeA,minfo *matchA,int wa,int ha,
					 unsigned char *codeB,minfo *matchB,int wb,int hb);
	int roundtrip( unsigned char *codeA,minfo *matchA,int wa,int ha,
					 unsigned char *codeB,minfo *matchB,int wb,int hb,int maxdist,int maxcost);

	int sort(unsigned char *codeA,minfo *matchA,int wa,int ha,
                        unsigned char *codeB,minfo *matchB,int wb,int hb);
#endif

   

};


#endif

