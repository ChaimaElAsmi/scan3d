#ifndef IMGV_PLUGIN_LEOPARD_HPP
#define IMGV_PLUGIN_LEOPARD_HPP


#include <opencv2/opencv.hpp>

#include <stdio.h>

class minfo {
	public:
	int idx; // index du match dans l'autre image  idx=y*w+x,  y=idx/w, x=idx%w
	unsigned short cost; // cout du match
	unsigned short active; // 1 = try to find this match, 0 = do not compute this match.
};

#define LEOPARD_CAM  0
#define LEOPARD_PROJ 1


class leopard {
	int n; // the count of images
	int nb; // number of bytes in the code (=(n-2+7)/8)
	int wc,hc; // width and height of camera image
	int wp,hp; // width and height of projector image

	// codes for camera [wc*hc*nb] access [(y*wc+x)*nb+b] ou [i*nb+b]
	unsigned char *codeCam;
	// codes for projector [wp*hp*nb] access [(y*wp+x)*nb+b] ou [i*nb+b]
	unsigned char *codeProj;
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
    void computeMinMax(cv::Mat *img,int nb,cv::Mat &min,cv::Mat &max);


  private:
	unsigned char bitCount[256]; // precomputed bit count

#if 0
	int cost(unsigned char *a,unsigned char *b,int sz);
	void match2image(blob *lut,minfo *match,int w,int h,int ww,int hh,int maxcost);
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

