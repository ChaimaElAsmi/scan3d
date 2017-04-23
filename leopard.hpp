#ifndef IMGV_PLUGIN_LEOPARD_HPP
#define IMGV_PLUGIN_LEOPARD_HPP


#include <opencv2/opencv.hpp>

#include <stdio.h>

class minfo {
	public:
	int idx; // index du match dans l'autre image  idx=y*w+x,  y=idx/w, x=idx%w
	unsigned short cost; // cout du match
	//unsigned short active; // 1 = try to find this match, 0 = do not compute this match.
};

// type de code binaire
#define LEOPARD_SIMPLE  0
#define LEOPARD_QUADRATIC  1


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

    public:
	leopard(); //int w,int h, int nb, int freq, bool blur,string pid);
	~leopard();

    cv::Mat *readImages(char *name,int from,int to);
    void computeMask(int cam,cv::Mat *img,int nb,double seuil,double bias,int step,int offx,int offy);
    void computeCodes(int cam,int type,cv::Mat *img);
    void prepareMatch();
	void forceBrute();
    void makeLUT(cv::Mat &lut,int cam);
    int doLsh();
    int doHeuristique();

    void statsCodes(int cam);


  private:
    void dumpCode(unsigned long *c);
    void dumpCodeNum(unsigned long *c);
    double horloge();
	int cost(unsigned long *a,unsigned long *b);
    int bitCount(unsigned long n);
	void match2image(cv::Mat &lut,minfo *match,unsigned char *mask,int w,int h,int ww,int hh);

	int lsh( int dir,unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
					 unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb);
	int heuristique( unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
					 unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb);

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

