

#include <leopard.hpp>

#include <sys/time.h>

//
// pattern leopard
//
// match camera et projecteur
//

using namespace cv;


// difference min entre un max et un min pour un pixel "actif"
//#define ACTIVE_DIFFERENCE	96


leopard::leopard() {
    printf("leopard init!\n");
}

leopard::~leopard() { printf("leopard uninit!\n"); }

int leopard::readImages(int which,char *name,int from,int to) {
    printf("-- reading images %s --\n",name);
    int nb,i;
    char buf[300];
    Mat image;
    int w=0,h=0;
    for(i=from,nb=0;i<=to;i++,nb++) {
        sprintf(buf,name,i);
        printf("read %s\n",buf);
        image = imread(buf,CV_LOAD_IMAGE_GRAYSCALE);
        printf("loaded %d x %d\n",image.cols,image.rows);
        if( nb==0 ) {
            w=image.cols;
            h=image.rows;
        }else{
            if( w!=image.cols || h!=image.rows ) {
                printf("Images %d pas de la meme taille!\n",i);
                return -1;
            }
        }
    }
    printf("nb=%d\n",nb);
    return 0;
}


#if 0
void leopard::init() {
	//log << "init tid=" << pthread_self() << warn;

	// random seed is random ba default, and can be set by /seed 65572645765
	struct timeval tv;
	gettimeofday(&tv,NULL);
	uint64 seed = ((uint64)tv.tv_sec*1000000)+tv.tv_usec;

	initializing=true;
	viewCam="";
	viewProj="";
	supercode=false;

	codeCam=NULL;
	codeProj=NULL;
	matchCam=NULL;
	matchProj=NULL;

	screenWidth=1920;
	screenHeight=1080;
	pixelsPerMM=38.76;

	iter=15;
	iterstep=5;

	activeDiff=ACTIVE_DIFFERENCE;

	currI=0;

	// bit count
	for(int i=0;i<256;i++) {
		int k=0;
		for(int m=1;m<=128;m*=2) if( (i&m) ) k++;
		bitCount[i]=k;
	}

	vote=NULL;
}

void leopard::uninit() {
	log << "uninit"<<warn;

	if( codeCam!=NULL ) { free(codeCam);codeCam=NULL; }
	if( codeProj!=NULL ) { free(codeProj);codeProj=NULL; }
	if( matchCam!=NULL ) { free(matchCam);matchCam=NULL; }
	if( matchProj!=NULL ) { free(matchProj);matchProj=NULL; }

	if( vote!=NULL ) { free(vote);vote=NULL; }

}

bool leopard::loop() {
	if( initializing ) {
		// attend des messages sur incam ou inproj
		pop_front_ctrl_nowait(Q_IN_CAM);
		pop_front_ctrl_nowait(Q_IN_PROJ);
	}
	if( initializing ) {
		usleepSafe(100000);
		return true;
	}

	blob *i1;

	// on lit des images!
	if( currI==0 ) {
		log << "waiting for proj 0" << warn;
		i1=pop_front_wait(Q_IN_PROJ);
		i1->copyTo(minProj);
		i1->copyTo(maxProj);
		wp=i1->cols;hp=i1->rows;
		//cv::imwrite("proj000.png",*i1);
		push_back(Q_TRASH,&i1);

		log << "waiting for cam 0" << warn;
		i1=pop_front_wait(Q_IN_CAM);
		i1->copyTo(minCam);
		i1->copyTo(maxCam);
		wc=i1->cols;hc=i1->rows;
		//cv::imwrite("cam000.png",*i1);
		push_back(Q_TRASH,&i1);

		log << "size cam is "<<wc<<" x "<<hc<<warn;
		log << "size proj is "<<wp<<" x "<<hp<<warn;

		currI++;
	}else if( currI==1 ) {
		log << "waiting for proj 1" << warn;
		i1=pop_front_wait(Q_IN_PROJ);
		unsigned char *p=minProj.data;
		unsigned char *q=i1->data;
		// update minProj et maxProj
		for(int i=0;i<wp*hp;i++,p++,q++) if( *q<*p ) *p=*q;
		p=maxProj.data;
		q=i1->data;
		for(int i=0;i<wp*hp;i++,p++,q++) if( *q>*p ) *p=*q;
		//cv::imwrite("proj001.png",*i1);
		push_back(Q_TRASH,&i1);
		//cv::imwrite("minProj0.png",minProj);
		//cv::imwrite("maxProj0.png",maxProj);
		//log << "saved moyProj.png"<<warn;


		log << "waiting for cam 1" << warn;
		i1=pop_front_wait(Q_IN_CAM);
		p=minCam.data;
		q=i1->data;
		for(int i=0;i<wc*hc;i++,p++,q++) if( *q<*p ) *p=*q;
		p=maxCam.data;
		q=i1->data;
		for(int i=0;i<wc*hc;i++,p++,q++) if( *q>*p ) *p=*q;
		//cv::imwrite("cam001.png",*i1);
		push_back(Q_TRASH,&i1);
		//cv::imwrite("minCam0.png",minCam);
		//cv::imwrite("maxCam0.png",maxCam);
		//log << "saved moyCam.png"<<warn;


		// initialize les byte
		currB=0;
		currM=1;

		// intialize all data
		log << "allocating "<<wc*hc*nb<<" bytes for camera code"<<err;
		log << "allocating "<<wp*hp*nb<<" bytes for projector code"<<err;
		codeCam=(unsigned char *)malloc(wc*hc*nb);
		codeProj=(unsigned char *)malloc(wp*hp*nb);

		// les matchs
		matchCam=(minfo *)malloc(wc*hc*sizeof(minfo));
		matchProj=(minfo *)malloc(wp*hp*sizeof(minfo));
		log << "allocating "<<(wc*hc*sizeof(minfo))<< " bytes for camera match"<<err;
		log << "allocating "<<(wp*hp*sizeof(minfo))<< " bytes for projector match"<<err;

		// nb bit is n-2, so n is higher than all codes

		for(int i=0;i<wc*hc*nb;i++) codeCam[i]=0;
		for(int i=0;i<wc*hc;i++) { matchCam[i].idx=0; matchCam[i].cost=n; }

		for(int i=0;i<wp*hp*nb;i++) codeProj[i]=0;
		for(int i=0;i<wp*hp;i++) { matchProj[i].idx=0; matchProj[i].cost=n; }

		pass=0;

		currI++;

	}else if( currI<n ) {

		log << "adding code for image "<<currI<< "(B="<<currB<<",M="<<currM<<")"<<info;

		log << "waiting for proj "<<currI<<info;
		i1=pop_front_wait(Q_IN_PROJ);

		// update min/max
		unsigned char *p=minProj.data;
		unsigned char *q=i1->data;
		// update minProj et maxProj
		for(int i=0;i<wp*hp;i++,p++,q++) if( *q<*p ) *p=*q;
		p=maxProj.data;
		q=i1->data;
		for(int i=0;i<wp*hp;i++,p++,q++) if( *q>*p ) *p=*q;
		//cv::imwrite("minProj.png",minProj);
		//cv::imwrite("maxProj.png",maxProj);


		//log << "saved moyProj.png"<<warn;
		p=codeProj;
		unsigned char *rmin=minProj.data;
		unsigned char *rmax=maxProj.data;
		q=i1->data;
		for(int i=0;i<wp*hp;i++,p+=nb,q++,rmin++,rmax++) {
			// si plus proche de min -> 0, si plus proche de max -> 1
			int ref=((int)*rmax + (int)*rmin + 1)/2;
			if( ((int)*q - ref) >0 ) p[currB] |= currM;
		}
		char buf[100];
#ifdef SAVE_IMAGES
		sprintf(buf,"proj%03d.png",currI);
		cv::imwrite(buf,*i1);
#endif
		push_back(Q_TRASH,&i1);


		log << "waiting for cam "<<currI<<info;
		i1=pop_front_wait(Q_IN_CAM);
		// update min/max
		p=minCam.data;
		q=i1->data;
		// update minCam et maxCam
		for(int i=0;i<wc*hc;i++,p++,q++) if( *q<*p ) *p=*q;
		p=maxCam.data;
		q=i1->data;
		for(int i=0;i<wc*hc;i++,p++,q++) if( *q>*p ) *p=*q;
		//cv::imwrite("minCam.png",minCam);
		//cv::imwrite("maxCam.png",maxCam);

		// get code bits
		p=codeCam;
		rmin=minCam.data;
		rmax=maxCam.data;
		q=i1->data;
		for(int i=0;i<wc*hc;i++,p+=nb,q++,rmin++,rmax++) {
			int ref=((int)*rmax + (int)*rmin + 1)/2;
			if( ((int)*q - ref) >0 ) p[currB] |= currM;
		}
#ifdef SAVE_IMAGES
		sprintf(buf,"cam%03d.png",currI);
		cv::imwrite(buf,*i1);
#endif
		push_back(Q_TRASH,&i1);

		currM *= 2;
		if( currM>128 ) { currM=1;currB++; }

		currI++;

		if( currI==n ) {
			// this is the last image.
			// we have the codes, but now we use min/max to activate or deactivate
			minfo *pm=matchCam;
			rmin=minCam.data;
			rmax=maxCam.data;
			for(int i=0;i<wc*hc;i++,pm++,rmin++,rmax++) {
				if( (int)*rmax-(int)*rmin < activeDiff ) pm->active=0; else pm->active=1;
			}
			pm=matchProj;
			rmin=minProj.data;
			rmax=maxProj.data;
			for(int i=0;i<wp*hp;i++,pm++,rmin++,rmax++) {
				if( (int)*rmax-(int)*rmin < activeDiff ) pm->active=0; else pm->active=1;
			}
			cv::imwrite("minCam.png",minCam);
			cv::imwrite("maxCam.png",maxCam);
			cv::imwrite("minProj.png",minProj);
			cv::imwrite("maxProj.png",maxProj);
		}
	}else{
		log << "matching pass " << pass <<err;

		//forceBrute();

		//profilometre_start("sort");
		//sort(codeCam,matchCam,wc,hc,  codeProj,matchProj,wp,hp);
		//profilometre_stop("sort");
		int delta;
		profilometre_start("lsh");
		for(int p=0;p<3;p++) {
			delta=lsh(pass%2,codeCam,matchCam,wc,hc, codeProj,matchProj,wp,hp);
			// une seconde fois dans l'autre sens
			delta=lsh(1-pass%2,codeCam,matchCam,wc,hc, codeProj,matchProj,wp,hp);
		}
		profilometre_stop("lsh");
		log << "delta cam = "<<delta<<info;

		for(int p=0;p<3;p++) {
			delta=lsh(pass%2,codeProj,matchProj,wp,hp, codeCam,matchCam,wc,hc);
			// une seconde fois dans l'autre sens
			delta=lsh(1-pass%2,codeProj,matchProj,wp,hp, codeCam,matchCam,wc,hc);
		}
		log << "delta proj = "<<delta<<info;

		profilometre_start("heuristique");
		delta=heuristique(codeCam,matchCam,wc,hc,
					  codeProj,matchProj,wp,hp);
		profilometre_stop("heuristique");
		log << "delta heu = "<<delta<<info;

		/*
		// random match! cam->proj
		for(int i=0;i<wc*hc;i++) {
			int j=rand()%(wp*hp); // random match
			int c=cost(codeCam+i*nb,codeProj+j*nb,nb);
			//log << "match " << i << " -> " << j<< " cost "<<c<<info;
			if( c<matchCam[i].cost ) { matchCam[i].idx=j;matchCam[i].cost=c; }
			if( c<matchProj[j].cost ) { matchProj[j].idx=i;matchProj[j].cost=c; }
		}
		// proj -> camera
		for(int i=0;i<wp*hp;i++) {
			int j=rand()%(wc*hc); // random match
			int c=cost(codeCam+j*nb,codeProj+i*nb,nb);
			if( c<matchProj[i].cost ) { matchProj[i].idx=j;matchProj[i].cost=c; }
			if( c<matchCam[j].cost ) { matchCam[j].idx=i;matchCam[j].cost=c; }
		}
		*/
		log << "Done" <<err;
		pass++;

		if( pass>0 && pass%iterstep==0 ) {
			roundtrip(codeCam,matchCam,wc,hc, codeProj,matchProj,wp,hp,20,n-2);
			roundtrip(codeProj,matchProj,wp,hp, codeCam,matchCam,wc,hc,20,n-2);

			blob *lut;

			lut=pop_front_wait(Q_RECYCLE);
			match2image(lut,matchProj,wp,hp,wc,hc,n-2);
			char buf[100];
			sprintf(buf,"lutProj%02d.png",pass);
			imwrite(buf,*lut);
			// send output
			lut->view=viewProj;
			lut->n=0;
			push_back(Q_OUT,&lut);

			lut=pop_front_wait(Q_RECYCLE);
			match2image(lut,matchCam,wc,hc,wp,hp,n-2);
			sprintf(buf,"lutCam%02d.png",pass);
			imwrite(buf,*lut);
			// send output
			lut->view=viewCam;
			lut->n=0;
			push_back(Q_OUT,&lut);

			// output un fichier XML openCV
			match2opencv(matchProj,wp,hp,wc,hc,100,10,1000);
			//match2opencv(matchProj,wp,hp,wc,hc,4,10,100000);
		}


		if( pass==iter ) return false;
		return true;
	}

	return(true);
}

int leopard::cost(unsigned char *a,unsigned char *b,int sz) {
	int i,k=0;
	for(i=0;i<sz;i++) k+=bitCount[a[i]^b[i]];
	return(k);
}

// output une image pour le match (imager w x h) vers une image ww x hh
void leopard::match2image(blob *lut,minfo *match,int w,int h,int ww,int hh,int maxcost) {
	lut->create(h,w,CV_16UC3);
	int x,y,xx,yy,cc;
	int i=0,j=0;
	for(y=0;y<h;y++) for(x=0;x<w;x++) {
		i=y*w+x;
		xx=match[i].idx%ww;
		yy=match[i].idx/ww;
		cc=match[i].cost;
		//lut->at<Vec3s>(y,x)=Vec3s((xx*65535)/ww,(yy*65535)/hh,(cc*65535)/maxcost);
		if( match[i].active==0 ) {
			lut->at<Vec3s>(y,x)=Vec3s( 65535, 65535, 65535);
		}else{
			lut->at<Vec3s>(y,x)=Vec3s( (cc*65535)/maxcost, (yy*65535)/hh, (xx*65535)/ww);
		}
	}
}


class point {
	public:
	int x,y;
	int closest; // index of closest in point table
	int dist2; // distance to closest squared
};

// return the index of the closest point in T[0..nb-1] to point T[which]
// set d2 to the distance squared
int findclosest(point *T,int nb,int which,int *pd2) {
	int i,imin,d2min;
	int dx,dy,d2;
	imin=-1;
	d2min=9999999;
	for(i=0;i<nb;i++) {
		if( i==which ) continue;
		dx=T[which].x-T[i].x;
		dy=T[which].y-T[i].y;
		d2=dx*dx+dy*dy;
		if( i==0 || d2<d2min ) { imin=i;d2min=d2; }
	}
	*pd2=d2min;
	return(imin);
}

//
// select nbpoints matches from all matches that have a cost <= maxcost
//
void leopard::match2opencv(minfo *match,int w,int h,int ww,int hh,int nbpoints,int maxcost,int iterations) {
	point *T;
	T=(point *)malloc((nbpoints+1)*sizeof(point));

	//
	// select nbpoints that are cost<=maxcost
	//
	for(int i=0;i<nbpoints;i++) {
		T[i].x=lrand48()%w;
		T[i].y=lrand48()%h;
	}

	for(int k=0;k<iterations;k++) {
		//
		// generate a new point
		//
		T[nbpoints].x=lrand48()%w;
		T[nbpoints].y=lrand48()%h;
		
		//
		// find closest pair of points
		//
		int imin=-1;
		int d2min=999999;
		for(int i=0;i<nbpoints+1;i++) {
			// check maxcost of this point
			int pos=T[i].y*w+T[i].x;
			if( match[pos].cost>maxcost ) {
				// this point should disapear... make it the "closest"
				imin=i;d2min=0;
			}else{
				T[i].closest=findclosest(T,nbpoints+1,i,&T[i].dist2);
				if( i==0 || T[i].dist2<d2min ) { imin=i;d2min=T[i].dist2; }
			}
			//printf("%d : (%d,%d) closest is %d , d2=%d\n",i,T[i].x,T[i].y,T[i].closest,T[i].dist2);
		}
		//printf("winner (closest point) is %d at dist2 = %d\n",imin,d2min);

		//
		// replace the closest points by a new random point,
		// if this increase the distance to closest
		//
		if( imin!=nbpoints ) T[imin]=T[nbpoints];
	}

	//for(int i=0;i<nbpoints;i++) { printf("{%d,%d},\n",T[i].x,T[i].y); }
	
	//
	// output result
	//

	FILE *f=fopen("match.xml","w");
	int x,y,xx,yy,cc;
	int i=0,j=0;
	fprintf(f,"<?xml version=\"1.0\"?>\n");
	fprintf(f,"<opencv_storage>\n");

	fprintf(f,"<coins type_id=\"opencv-matrix\">\n");
	fprintf(f," <rows>%d</rows>\n",nbpoints);
	fprintf(f," <cols>1</cols>\n");
	fprintf(f," <dt>\"2f\"</dt>\n");
	fprintf(f," <data>\n");
	for(int i=0;i<nbpoints;i++) {
		int pos=T[i].y*w+T[i].x;
		xx=match[pos].idx%ww;
		yy=match[pos].idx/ww;
		cc=match[pos].cost;
		fprintf(f,"%10.1f %10.1f\n",(float)xx,(float)yy);
	}
	fprintf(f," </data>\n");
	fprintf(f,"</coins>\n");

	fprintf(f,"<damier type_id=\"opencv-matrix\">\n");
	fprintf(f," <rows>%d</rows>\n",nbpoints);
	fprintf(f," <cols>1</cols>\n");
	fprintf(f," <dt>\"2f\"</dt>\n");
	fprintf(f," <data>\n");
	for(int i=0;i<nbpoints;i++) {
		fprintf(f,"%12.5f %12.5f\n",(float)T[i].x/w*screenWidth/pixelsPerMM,(float)T[i].y/h*screenHeight/pixelsPerMM);
	}
	fprintf(f," </data>\n");
	fprintf(f,"</damier>\n");

	fprintf(f,"</opencv_storage>\n");
	fclose(f);

	free(T);
}

void leopard::forceBrute() {
		// force brute
		int x,y,xx,yy;
		int s=1;
		for(y=hc/2-s;y<=hc/2+s;y++) { log << y << info; for(x=0;x<wc;x++) {
			int i=y*wc+x;
			for(yy=0;yy<hp;yy++) for(xx=0;xx<wp;xx++) {
				int j=yy*wp+xx;
				int c=cost(codeCam+i*nb,codeProj+j*nb,nb);
				if( c<matchCam[i].cost ) { matchCam[i].idx=j;matchCam[i].cost=c; }
				if( c<matchProj[j].cost ) { matchProj[j].idx=i;matchProj[j].cost=c; }
			}
		}}
}

typedef struct {
	unsigned char byte; // 0..nb-1
	unsigned char mask; // 1,2,4,8..128
	unsigned int vmask; // voting mask (nv bits)
} bminfo;

double horloge() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return((double)tv.tv_sec+tv.tv_usec/1000000.0);
}


unsigned char *gcodeA; // index >=0 : [i]
unsigned char *gcodeB; // index i<0 : [-i-1]
int gnb; // for compare
// compare two codes
int compar(const void* idxP,const void* idxQ) {
	//printf("compare [%d] [%d]\n",*(int*)idxP,*(int*)idxQ);
	int ip=*(int *)idxP;
	int iq=*(int *)idxQ;
	unsigned char *p,*q;
	if( ip>=0 ) p=gcodeA+ip*gnb; else p=gcodeB+(-ip-1)*gnb;
	if( iq>=0 ) q=gcodeA+iq*gnb; else q=gcodeB+(-iq-1)*gnb;
	for(int i=0;i<gnb;i++,p++,q++) {
		if( *p<*q ) return(-1);
		if( *p>*q ) return(1);
	}
	return(0);
}


int leopard::sort(unsigned char *codeA,minfo *matchA,int wa,int ha,
		        unsigned char *codeB,minfo *matchB,int wb,int hb) {
	// un index des positions dans les codes
	int i;
	// 5 et 10 -> -5..-1 et 0..9 , donc taille 5+10
	int *idxAB; // i>=0 : CodeA[i], i<0 : codeB[-i-1]
	idxAB=(int *)malloc((wa*ha+wb*hb)*sizeof(int));
	// sort les codes
	for(i=0;i<wa*ha;i++) idxAB[i]=i;
	for(i=0;i<wb*hb;i++) idxAB[i+wa*ha]=-i-1;
	gcodeA=codeA; gcodeB=codeB; gnb=nb;
	qsort(idxAB,wa*ha+wb*hb,sizeof(int),compar);
	// dump
	for(i=0;i<wa*ha+wb*hb && i<200;i++) {
		int j=idxAB[i];
		unsigned char *p;
		if( j>=0 ) {
			printf("codeA[%6d]=",j);
			p=codeA+j*nb;
		}else{
			printf("codeB[%6d]=",-j-1);
			p=codeB+(-j-1)*nb;
		}
		for(int k=0;k<nb;k++) {
			for(unsigned char mask=128;mask;mask>>=1) {
				printf("%d",p[k]&mask?1:0);
			}
			printf(" ");
		}
		printf("\n");
	}
	free(idxAB);
	return 0;
}



int leopard::lsh(int dir,unsigned char *codeA,minfo *matchA,int wa,int ha,
			      unsigned char *codeB,minfo *matchB,int wb,int hb) {
	int nv=20; // nb de bits pour le vote
	int nbvote=(1<<nv);
	bminfo bm[nv];
	double now=horloge();

	//log << "lsh " << nv << " bits, allocate "<< ((1<<nv)*sizeof(int)) << " bytes"<<warn;

	if( vote==NULL) vote=(int *)malloc(nbvote*sizeof(int));
	for(int i=0;i<nbvote;i++) vote[i]=-1;

	// pas ideal... ca peut repeter des bits...
	for(int i=0;i<nv;i++) {
		bm[i].byte=rand()%nb;
		bm[i].mask=1<<(rand()%8);
		bm[i].vmask=1<<i;
		//log << "bit " << i << " byte="<<(int)bm[i].byte<<", mask="<<(int)bm[i].mask<<", vmask="<<bm[i].vmask<<info;
	}

	// A vote
	unsigned char *p;
	int start,stop,step;
	if( dir ) { start=0;stop=wa*ha;step=1; }
	else		{ start=wa*ha-1;stop=-1;step=-1; }
	for(int i=start;i!=stop;i+=step) {
			// si on est pas actif.... on saute!
			if( matchA[i].active==0 ) continue;
			// ramasse le hashcode
			p=codeA+i*nb;
			unsigned int hash=0;
			for(int k=0;k<nv;k++) {
				if( p[bm[k].byte]&bm[k].mask ) hash|=bm[k].vmask;
			}
			// basic case... ecrase les autres votes
			//vote[hash]=i;
			// better option??? if there is already a vote, replace if we have a worse best match
			if( vote[hash]<0 || matchA[i].cost>matchA[vote[hash]].cost ) vote[hash]=i;
	}

	// stats
	int holes=0;
	for(int i=0;i<nbvote;i++) if( vote[i]<0 ) holes++;
	log << "holes = "<<(holes*100/nbvote)<<"%"<<warn;

	// B match
	for(int i=0;i<wb*hb;i++) {
			// si on est pas actif.... on saute!
			if( matchB[i].active==0 ) continue;
			// ramasse le hashcode
			p=codeB+i*nb;
			unsigned int hash=0;
			for(int k=0;k<nv;k++) {
				if( p[bm[k].byte]&bm[k].mask ) hash|=bm[k].vmask;
			}
			// basic
			int j=vote[hash];
			if( j<0 ) continue;

			// match!
			int c=cost(codeA+j*nb,codeB+i*nb,nb);
			if( c<matchA[j].cost ) { matchA[j].idx=i;matchA[j].cost=c; }
			if( c<matchB[i].cost ) { matchB[i].idx=j;matchB[i].cost=c; }
	}
	// somme des couts
	int delta=0;
	for(int i=0;i<wa*ha;i++) delta+=matchA[i].cost;
	for(int i=0;i<wb*hb;i++) delta+=matchB[i].cost;

	now=horloge()-now;
	log << "time is "<<now<<warn;
	return(delta);
}

int leopard::heuristique( unsigned char *codeA,minfo *matchA,int wa,int ha,
						 unsigned char *codeB,minfo *matchB,int wb,int hb) {
	// pour chaque cam[i] -> proj[j]
	// on regarde si on pourrait ameliorer le match vers proj[autour de j]
	// on regarde si on pourrait creer un match cam[autour de i] vers proj[j]
	int di[8]={-1,1,-wa,wa,-1-wa,-1+wa,1-wa,1+wa};
	int dj[8]={-1,1,-wb,wb,-1-wb,-1+wb,1-wb,1+wb};

	int i,j,k;
	int x,y,xx,yy;
	for(i=0;i<wa*ha;i++) {
		// skip si pas actif!
		if( matchA[i].active==0 ) continue;

		j=matchA[i].idx;
		// regarde autour de j
		for(k=0;k<8;k++) {
			int jj=j+dj[k];
			if( jj<0 || jj>=wb*hb ) continue;
			int c=cost(codeA+i*nb,codeB+jj*nb,nb);
			if( c<matchA[i].cost ) { matchA[i].idx=jj;matchA[i].cost=c; }
			if( c<matchB[jj].cost ) { matchB[jj].idx=i;matchB[jj].cost=c; }
		}
		// regarde autour de i
		for(k=0;k<8;k++) {
			int ii=i+di[k];
			if( ii<0 || ii>=wa*ha ) continue;
			int c=cost(codeA+ii*nb,codeB+j*nb,nb);
			if( c<matchA[ii].cost ) { matchA[ii].idx=j;matchA[ii].cost=c; }
			if( c<matchB[j].cost ) { matchB[j].idx=ii;matchB[j].cost=c; }
		}
		
	}
	// somme des couts
	int delta=0;
	for(int i=0;i<wa*ha;i++) delta+=matchA[i].cost;
	for(int i=0;i<wb*hb;i++) delta+=matchB[i].cost;
	return(delta);
}

int leopard::roundtrip( unsigned char *codeA,minfo *matchA,int wa,int ha,
				 unsigned char *codeB,minfo *matchB,int wb,int hb,int maxdist,int maxcost) {
	// matchA[i] -> j matchB[j] -> i
	int i,j,k;
	int x,y,xx,yy;
	int dist2;
	int maxdist2=maxdist*maxdist;
	for(i=0;i<wa*ha;i++) {
		// skip si pas actif!
		if( matchA[i].active==0 ) continue;

		j=matchA[i].idx;
		if( j<0 || j>=wb*hb ) { matchA[i].cost=maxcost;continue; }
		k=matchB[j].idx;
		if( k<0 || k>=wa*ha ) { matchA[i].cost=maxcost;continue; }
		// distance round trip?
		x=i%wa;y=i/wa;
		xx=k%wa;yy=k/wa;
		dist2=(x-xx)*(x-xx)+(y-yy)*(y-yy);
		if( dist2>maxdist2 ) matchA[i].cost=maxcost;
		//printf("(%4d,%4d) -> (%4d,%4d) d=%7d\n",x,y,xx,yy,dist2);
	}
	return 0;
}



//
// /set/count <int n>
// /set/screen w h ppm
//
bool leopard::decode(const osc::ReceivedMessage &m) {
	log << "decoding " << m << warn;
	const char *address=m.AddressPattern();
	if( !initializing ) {
		log << "unexpected command: "<<m<<warn;
		return false;
	}

	if( oscutils::endsWith(address,"/set/screen") ) {
		int32 w,h;
		float ppm;
                m.ArgumentStream() >> w >> h >> ppm >> osc::EndMessage;
                screenWidth=w;
		screenHeight=h;
		pixelsPerMM=ppm;
		log << "Screensize set to ("<<w<<","<<h<<") pixelsPerMM="<<pixelsPerMM<<warn;
	}else if( oscutils::endsWith(address,"/set/iter") ) {
                m.ArgumentStream() >> iter >> iterstep >> osc::EndMessage;
	}else if( oscutils::endsWith(address,"/set/count") ) {
                m.ArgumentStream() >> n >> osc::EndMessage;
                // init the system
		nb=(n-2+7)/8;
		log << "count set to "<<n<< " and bytes = "<<nb<<warn;
	}else if( oscutils::endsWith(address,"/supercode") ) {
                m.ArgumentStream() >> supercode >> osc::EndMessage;
		log << "supercode set to "<<supercode<< warn;
	}else if( oscutils::endsWith(address,"/set/activeDiff") ) {
                m.ArgumentStream() >> activeDiff >> osc::EndMessage;
		log << "seuil actif = "<<activeDiff<< warn;
	}else if( oscutils::endsWith(address,"/set/view") ) {
		const char *vcam,*vproj;
                m.ArgumentStream() >> vcam >> vproj >> osc::EndMessage;
		viewCam=string(vcam);
		viewProj=string(vproj);
	}else if( oscutils::endsWith(address,"/init-done") ) {
		log << "init done." << warn;
		initializing=false;
	}else{
		log << "unknown command: "<<m<<warn;
		return false;
	}
	return true;
}

#endif




