#ifndef IMGV_PLUGIN_LEOPARD_HPP
#define IMGV_PLUGIN_LEOPARD_HPP


/*!

\defgroup pluginLeopard Leopard plugin
@{
\ingroup plugins

****DOC NOT DONE****
Match des patterns leopard camera/projecteur.<br>

Ports
-----

|      # | Name | Direction | Description |
| ------ | ---- | --------- | ----------- |
| 0      | incam   | in        | queue for input camera images |
| 1      | inproj  | in        | queue for input projector images |
| 2      | out  | out       | queue for image output |
| 3      | log  | out       | log output |
| 4      | recycle  | in       | recycling images for results |

Events
------

| #    | Name  | Description |
| ---- | ----  | ----------- |



OSC initializing commands
------------

### `/set/screen  <int width> <int height> <float pixelsPerMM>`

  Define screen size to generate correct match.xml for calibration.
  For apple display, use: 1920 1200 38.76
  For LG superwide, use: 2560 1080 38.03

### `/set/count <int c>`

  Expected number of images. It is understood that the first two images will be used
  to compute the mean. So if you intend to use 10 bytes, then you must set
  the count at 80+2 images.

### `/set/view <string view cam> <string view proj>`

  view given to output of camera match, and for projector match

### `/set/activeDiff <int seuil>

  Defini le difference minimale entre un min et un max observe pour considerer un pixel actif. Valeur par defaut: 96.

### `/supercode <bool v>`

  use supercodes (true) or not. For N patterns, we normaly obtain n-1 bits. With supercode ON,
  we use n (n-1) /2 codes.

### `/init-done`

  IMPORTANT: YOU MUST SEND THIS COMMAND TO GET THE MATCHING STARTED.

Sample Code
--------------

- \ref playLeopard

@}
*/



#include <imgv/imgv.hpp>
#include <imgv/plugin.hpp>

class minfo {
	public:
	int idx; // index du match dans l'autre image  idx=y*w+x,  y=idx/w, x=idx%w
	unsigned short cost; // cout du match
	unsigned short active; // 1 = try to find this match, 0 = do not compute this match.
};

class pluginLeopard : public plugin<blob> {
	int32 n; // the count of images
	int nb; // number of bytes in the code (=(n-2+7)/8)
	int wc,hc; // width and height of camera image
	int wp,hp; // width and height of projector image

	// lecture des images:
	int currI; // image courante. 0..n-1
	int currB; // byte courant... 0..nb-1, ou -1 si pas actif
	int currM; // masque courant... 1,2,4,8,16..128 pour les 8 patterns

	bool initializing;
	int pass; // pour le matching

	int32 iter;	// last pass number
	int32 iterstep; // step pour les pass
	
	// codes for camera [wc*hc*nb] access [(y*wc+x)*nb+b] ou [i*nb+b]
	unsigned char *codeCam;
	// codes for projector [wp*hp*nb] access [(y*wp+x)*nb+b] ou [i*nb+b]
	unsigned char *codeProj;
	// match for camera [wc*hc] access [y*wc+x] -> index into projector image, and cost
	minfo *matchCam;
	// match for projector [wp*hp] access [y*hp+x] -> index into camera image, and cost
	minfo *matchProj;

	int32 activeDiff;

	// pour LSH
	int *vote;

	string viewCam,viewProj;  // output view

	cv::Mat minCam,maxCam;
	cv::Mat minProj,maxProj;

	bool supercode;

	// pour le match.xml
	int screenWidth;
	int screenHeight;
	float pixelsPerMM;

    public:
	pluginLeopard(); //int w,int h, int nb, int freq, bool blur,string pid);
	~pluginLeopard();

	void init();
	void uninit();
	bool loop();
	bool decode(const osc::ReceivedMessage &m);
	void dump(ofstream &file);

  private:
	unsigned char bitCount[256]; // precomputed bit count

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


   

};


#endif

