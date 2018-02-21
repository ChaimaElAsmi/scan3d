#include <fstream>
#include <leopard.hpp>

//#include <sys/time.h>


//
// pattern leopard
//
// match camera et projecteur
//

using namespace cv;


// difference min entre un max et un min pour un pixel "actif"
//#define ACTIVE_DIFFERENCE	96

unsigned char bt[65536];

int bitCountOrig(unsigned long n) {
    n = ((0xaaaaaaaaaaaaaaaaL & n) >>  1) + (0x5555555555555555L & n);
    n = ((0xccccccccccccccccL & n) >>  2) + (0x3333333333333333L & n);
    n = ((0xf0f0f0f0f0f0f0f0L & n) >>  4) + (0x0f0f0f0f0f0f0f0fL & n);
    n = ((0xff00ff00ff00ff00L & n) >>  8) + (0x00ff00ff00ff00ffL & n);
    n = ((0xffff0000ffff0000L & n) >> 16) + (0x0000ffff0000ffffL & n);
    n = ((0xffffffff00000000L & n) >> 32) + (0x00000000ffffffffL & n);
    return (int)n;
}

typedef struct {
	unsigned int byte; // 0..nb-1 (c'est en fait un unsigned long, donc max bits = 255*64=16384 bits
	unsigned long mask; // 1,2,4,8......1L<<63
	unsigned int vmask; // voting mask (nv bits)
} bminfo;

leopard::leopard() {
    printf("leopard init!\n");
    n=0;
	maskCam=NULL;
	maskProj=NULL;
    codeCam=NULL;
    codeProj=NULL;
	matchCam=NULL;
	matchProj=NULL;

    vote=NULL;
    wDecal=5;

    // strings pour les filename
    fn_scan_maskc=NULL;
    fn_scan_meanc=NULL;
    fn_scan_maskp=NULL;
    fn_scan_meanp=NULL;

    for(int i=0;i<65536;i++) bt[i]=bitCountOrig(i);
    //for(int i=0;i<65536;i++) printf("%3d : %3d\n",i,bt[i]);
    initSP();
}

leopard::~leopard() {
	printf("leopard uninit!\n");
	if( maskProj ) free(maskProj);
	if( maskCam ) free(maskCam);
    if( codeCam ) {
#ifdef USE_GMP
        for(int i=0;i<wc*hc;i++) mpz_clear(codeCam[i]);
#endif
        free(codeCam);
    }
    if( codeProj ) {
#ifdef USE_GMP
        for(int i=0;i<wp*hp;i++) mpz_clear(codeProj[i]);
#endif
        free(codeProj);
    }
	if( matchCam ) free(matchCam);
	if( matchProj ) free(matchProj);
    unInitSP();
}


double leopard::horloge() {
    /*
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return((double)tv.tv_sec+tv.tv_usec/1000000.0);
    */
    return(1.5);
}

//On passe le nom des images
cv::Mat *leopard::readImages(char *name,int from,int to, double fct) {
    printf("-- reading images %s --\n",name);
    int nb,i;
    char buf[300];
    nb=to-from+1;
    Mat *image=new Mat[nb];
    int w=0,h=0;

    for(i=0;i<nb;i++) {
        sprintf(buf,name,i+from);
        printf("read %d %s\n",i,buf);
        FILE *F=fopen(buf,"r");
        if( F==NULL ) return(NULL);
        fclose(F);
        if(fct>0) {
            Mat tmp = imread(buf,CV_LOAD_IMAGE_GRAYSCALE);
            resize(tmp, image[i],cvSize(0,0),0.5,0.5);
        }
        else{
            image[i] = imread(buf,CV_LOAD_IMAGE_GRAYSCALE);
        }
        //printf("loaded %d x %d\n",image[i].cols,image[i].rows);
        if( i==0 ) {
            w=image[i].cols;
            h=image[i].rows;
        }else{
            if( w!=image[i].cols || h!=image[i].rows ) {
                printf("Images %d pas de la meme taille!\n",i+from);
                delete[] image;
                return NULL;
            }
        }
    }
    return image;
}


void leopard::noisify(cv::Mat *cam,int nb,double stddev,double mean) {
    for(int i=0;i<nb;i++) {
		// We need to work with signed images (as noise can be
		// negative as well as positive). We use 16 bit signed
		// images as otherwise we would lose precision.
		Mat noise(cam[i].size(), CV_16SC1);
		randn(noise, Scalar::all(mean), Scalar::all(stddev));
		Mat tmp;
		cam[i].convertTo(tmp,CV_16SC1);
		addWeighted(tmp, 1.0, noise, 1.0, 0.0, tmp);
		tmp.convertTo(cam[i],cam[i].type());
    }
}


//On passe les images directement
cv::Mat *leopard::readImages2(Mat *cam,int from,int to) {
    printf("-- reading images camera --\n");
    int nb,i;
    nb=to-from+1;
    Mat *image=new Mat[nb];
    int w=0,h=0;

    for(i=0;i<nb;i++) {  
        printf("read %d cam_%d \n",i,i+from-1);
        cvtColor(cam[i+from], image[i], CV_RGB2GRAY);
        if( i==0 ) {
            w=image[i].cols;
            h=image[i].rows;
        }else{
            if( w!=image[i].cols || h!=image[i].rows ) {
                printf("Images %d pas de la meme taille!\n",i+from);
                delete[] image;
                return NULL;
            }
        }
    }
    return image;
}


//
// On calcule un ratio binmodal/unimodal
// Normalement, si c'est >1, c' esst bimodal.
// cam: 1=oui, 0=non (donc proj)
// step: 1= normal, 4=keep only one pixel per 4 inx, 4 in y
// offx,offy : decallage x,y. DOIT ETRE <step
//

void leopard::computeMask(int cam,cv::Mat *img,int nb,double seuil,double bias,int step,
                          int xmin,int xmax,int ymin,int ymax) {
    int nr=img[0].rows;
    int nc=img[0].cols;
    cv::Mat min,max,delta,mask;
    min.create(nr,nc,CV_8UC1);
    max.create(nr,nc,CV_8UC1);
    cv::Mat sum,sum2;
    sum.create(nr,nc,CV_32SC1);
    sum2.create(nr,nc,CV_32SC1);
    
    unsigned char *pmin=min.data;
    unsigned char *pmax=max.data;
    int *psum=(int *)sum.data;
    int *psum2=(int *)sum2.data;

    for(int i=0;i<nb;i++) {
		unsigned char *p=img[i].data;
        if( i==0 ) {
            for(int j=0;j<nr*nc;j++) {
                pmin[j]=pmax[j]=p[j];
                psum[j]=p[j];
                psum2[j]=(int)p[j]*(int)p[j];
            }
        }else{
            for(int j=0;j<nr*nc;j++) {
                if( p[j]<pmin[j] ) pmin[j]=p[j];
                if( p[j]>pmax[j] ) pmax[j]=p[j];
                psum[j]+=p[j];
                psum2[j]+=(int)p[j]*(int)p[j];
            }
        }
    }

    // calcule delta=max-min
    delta.create(nr,nc,CV_8UC1);
    unsigned char *pdelta=delta.data;
    for(int j=0;j<nr*nc;j++) pdelta[j]=pmax[j]-pmin[j];

    // calcule la moyenne
    cv::Mat mean,std;
    mean.create(nr,nc,CV_8UC1);
    std.create(nr,nc,CV_8UC1);
    unsigned char *pmean=mean.data;
    unsigned char *pstd=std.data;
    for(int j=0;j<nr*nc;j++) {
        pmean[j]=(psum[j]*2+nb)/(2*nb);
        pstd[j]=(int)sqrt((double)psum2[j]/nb-(double)pmean[j]*pmean[j]);
    }

    //printf("nb=%d  sum=%d  val=%d\n",nb,pmean[1228*nc+921],pmean8[1228*nc+921]);
    // calcule la moyenne/deviation standard de chaque cote de la moyenne
    cv::Mat sumL,sumH; // somme si <moyenne, si >moyenne
    cv::Mat sum2L,sum2H; // somme des carres
    cv::Mat countL,countH;
    sumL.create(nr,nc,CV_32SC1);
    sumH.create(nr,nc,CV_32SC1);
    sum2L.create(nr,nc,CV_32SC1);
    sum2H.create(nr,nc,CV_32SC1);
    countL.create(nr,nc,CV_32SC1);
    countH.create(nr,nc,CV_32SC1);

    int *psumL=(int *)sumL.data;
    int *psum2L=(int *)sum2L.data;
    int *psumH=(int *)sumH.data;
    int *psum2H=(int *)sum2H.data;
    int *pcountL=(int *)countL.data;
    int *pcountH=(int *)countH.data;

    for(int j=0;j<nr*nc;j++) { psumL[j]=psum2L[j]=psumH[j]=psum2H[j]=pcountL[j]=pcountH[j]=0; }

    for(int i=0;i<nb;i++) {
		unsigned char *p=img[i].data;
        for(int j=0;j<nr*nc;j++) {
            if( p[j]<pmean[j] ) { psumL[j]+=p[j];psum2L[j]+=(int)p[j]*p[j];pcountL[j]+=1; }
            else if( p[j]>pmean[j] ) { psumH[j]+=p[j];psum2H[j]+=(int)p[j]*p[j];pcountH[j]+=1; }
        }
    }

    // on veut comparer la deviation standard de chaque cote avec la distance entre les moyennes
    // ratio (m2-m1)/std si c'est <=1 unimodal si >>1 bimodal
    cv::Mat bimod;
    bimod.create(nr,nc,CV_8UC1);
    mask.create(nr,nc,CV_8UC1);
    unsigned char *pbimod=bimod.data;
    unsigned char *pmask=mask.data;

    for(int j=0;j<nr*nc;j++) {
        int cL=(pcountL[j]==0?1:pcountL[j]);
        int cH=(pcountH[j]==0?1:pcountH[j]);
        double mL=psumL[j]/cL;
        double mH=psumH[j]/cH;
        double stdL=(int)sqrt((double)psum2L[j]/pcountL[j]-(double)mL*mL);
        double stdH=(int)sqrt((double)psum2H[j]/pcountH[j]-(double)mH*mH);

        //wikipedia: Multimodal_distribution
        double v=(mH-mL)/(2*sqrt(stdL+stdH)+bias); // le +5 est pour limiter l'effet lorsque mH-mL -> 0

        pbimod[j]=v*20;
        pmask[j]=(v>=seuil)?255:0;

    }

	// set values for the future
    //décalege en x,y à partir de wc,hc
    //offyy=hc / offxx=wc -> jusqu'à la fin de l'image
    //int offyy = offy+20;
    //int offxx = offx+20;
	if( cam ) {
		wc=nc;hc=nr;
        if( xmin<0 ) xmin=0;
        if( ymin<0 ) ymin=0;
        if( xmax<0 || xmax>=wc ) xmax=wc-1;
        if( ymax<0 || ymax>=hc ) ymax=hc-1;
		maskCam=(unsigned char *)malloc(wc*hc);
		for(int j=0;j<wc*hc;j++) maskCam[j]=0;
        for(int y=ymin;y<=ymax;y+=step) for(int x=xmin;x<=xmax;x+=step) maskCam[y*wc+x]=pmask[y*wc+x];
        // ajuste le mask pour l'image
		for(int j=0;j<wc*hc;j++) if( maskCam[j]==0 ) pmask[j]=0;
        if( n>0 && n!=nb ) { printf("Cam et Proj : pas le meme nombre d'images!!!!\n");exit(0); }
        n=nb;
	}else{
		wp=nc;hp=nr;
        if( xmin<0 ) xmin=0;
        if( ymin<0 ) ymin=0;
        if( xmax<0 || xmax>=wp ) xmax=wp-1;
        if( ymax<0 || ymax>=hp ) ymax=hp-1;
		maskProj=(unsigned char *)malloc(wp*hp);
		for(int j=0;j<wp*hp;j++) maskProj[j]=0;
		for(int y=ymin;y<=ymax;y+=step) for(int x=xmin;x<=xmax;x+=step) maskProj[y*wp+x]=pmask[y*wp+x];
        // ajuste le mask pour l'image
		for(int j=0;j<wp*hp;j++) if( maskProj[j]==0 ) pmask[j]=0;
        if( n>0 && n!=nb ) { printf("Cam et Proj : pas le meme nombre d'images!!!!\n");exit(0); }
        n=nb;
	}


     /*
    imwrite("mean.png",mean);
    imwrite("std.png",std);
    imwrite("bimod.png",bimod);
    imwrite("delta.png",delta);
    imwrite("min.png",min);
    imwrite("max.png",max);
    */

    if (cam) {
        // imwrite(FN_SCAN_MASKC,mask);
        // imwrite(FN_SCAN_MEANC,mean);
        if( fn_scan_maskc ) imwrite(fn_scan_maskc,mask);
        if( fn_scan_meanc ) imwrite(fn_scan_meanc,mean);
    }
    else {
        // imwrite(FN_SCAN_MASKP, mask);
        // imwrite(FN_SCAN_MEANP,mean);
        if( fn_scan_maskp ) imwrite(fn_scan_maskp, mask);
        if( fn_scan_meanp ) imwrite(fn_scan_meanp,mean);
    }
}

#ifdef USE_GMP
void leopard::dumpCode(mpz_t *c) {
    int one=mpz_popcount(*c);
    printf("%s [%d/%d] ", mpz_get_str (NULL, 2, *c),nbb-one,one);
    //for(int i=0;i<nbb;i++) printf("%d",mpz_tstbit(*c,i));
}
#else
void leopard::dumpCode(unsigned long *c) {
    int one=0;
	for(int i=0;i<nbb;i++) {
        int b=i/64;
        unsigned long mask=1L<<(i%64);
        if( c[b]&mask ) {one++;printf("1");} else printf("0");
    }
    printf(" [%d/%d] ",nbb-one,one);
}
#endif

#ifdef USE_GMP
void leopard::dumpCodeNum(mpz_t *c) {
    printf("%s\n", mpz_get_str (NULL,10, *c));
}
#else
void leopard::dumpCodeNum(unsigned long *c) {
	for(int b=0;b<nb;b++) {
        printf("%lu ",c[b]);
    }
    printf("\n");
}
#endif

#ifdef USE_GMP
void leopard::computeCodes(int cam,int type,cv::Mat *img) {
    printf("-- compute codes cam=%d type=%d n=%d --\n",cam,type,n);

    int w,h;
    mpz_t *code=NULL; // [w*h*nb] [(y*w+x)*nb+b]
    unsigned char *pmask;

    if( cam ) {
        w=wc;h=hc;
        if( codeCam ) {
            for(int i=0;i<wc*hc;i++) mpz_clear(codeCam[i]);
            free(codeCam);
        }
        if( !maskCam ) { printf("cam: No mask to compute codes!\n");exit(0); }
        pmask=maskCam;
    }else{
        w=wp;h=hp;
        if( codeProj ) {
            for(int i=0;i<wp*hp;i++) mpz_clear(codeProj[i]);
            free(codeProj);
        }
        if( !maskProj ) { printf("proj: No mask to compute codes!\n");exit(0); }
        pmask=maskProj;
		printf("*** WP=%d HP=%d\n",wp,hp);
    }


    if( type==LEOPARD_SIMPLE ) {
        // compare chaque image a l'image moyenne
        nbb=n;
        printf("nbb=%d\n",nbb);
        code=(mpz_t *)malloc(w*h*sizeof(mpz_t));
        for(int i=0;i<w*h;i++) mpz_init2(code[i],nbb);

        // trouve l'image moyenne
        cv::Mat sum;
        sum.create(h,w,CV_32SC1);
        int *psum=(int *)sum.data;
		for(int j=0;j<w*h;j++) psum[j]=0;
        for(int i=0;i<n;i++) {
            unsigned char *p=img[i].data;
            for(int j=0;j<w*h;j++) psum[j]+= *p++;
        }
        cv::Mat mean;
        mean.create(h,w,CV_16UC1);
        unsigned short *pmean=(unsigned short *)mean.data;
        // 8 bits de precision
        for(int j=0;j<w*h;j++) pmean[j]=(psum[j]*256*2+n)/(2*n);
        //imwrite("meancode.png",mean);


        int ambigu=0;
        int count=0;
        for(int j=0;j<w*h;j++) {
            if( pmask[j]==0 ) continue; // tous les codes en dehors du mask sont 0
            count++;
            for(int i=0;i<n;i++) {
                unsigned char *p=img[i].data;
                unsigned short v=(p[j]<<8)|0x80;
                if( v>pmean[j] || (v==pmean[j] && (rand()&1)) ) mpz_setbit(code[j],i);
                if( v==pmean[j] ) { ambigu++; } // ne sert plus a rien
            }
            if( (j%w)==200 && (j/w)==300 ) {
                printf("pixel (%d,%d) : ",j%w,j/w);
                dumpCode(code+j);
                printf("\n");
            }
        }

        printf("count=%d (%d %%), ambigu=%d\n",count,count*100/(w*h),ambigu);
    }else if( type==LEOPARD_QUADRATIC ) {
        nbb=n*(n-1)/2; // quadratic!
        printf("nbb=%d\n",nbb);
        code=(mpz_t *)malloc(w*h*sizeof(mpz_t));

        for(int i=0;i<w*h;i++) mpz_init2(code[i],nbb);

        double t1=horloge();
        int nbk=0; // compte les codes
        for(int i=0;i<n;i++) {
            printf("i=%d\n",i);
            unsigned char *pi=img[i].data;

            for(int j=i+1;j<n;j++) {
                //printf("j=%d\n",j);
                unsigned char *pj=img[j].data;
                for(int k=0;k<w*h;k++) {
                    if( pmask[k]==0 ) continue;
                    if( pi[k]>pj[k] || (pi[k]==pj[k] && (nbk&1)) ) mpz_setbit(code[k],nbk);
                }
                nbk++;
            }
        }
        double t2=horloge();
        printf("duree=%12.6f\n",t2-t1);
        //int j=900*w+1200;
        int j = 200 * w + 300;
        while( pmask[j]==0 ) j++;
        printf("pixel %d (%d,%d) : ",j,j%w,j/w);
        dumpCode(code+(j));
        printf("\n");

        /*
        for(int i=0;i<w*h;i++) {
            if( pmask[i]==0 ) continue;
            dumpCodeNum(code+(i*nb));
        }
        */
        /*
        t1=horloge();
        int i;
        int nnn=0;
        for(i=0;i<w*h;i+=1) {
            j=w*h-1-i;
            if( pmask[i]==0 || pmask[j]==0 ) continue;
            int c=cost(code+i,code+j);
            //printf("code %d et %d : %d\n",i,j,c);
            nnn++;
        }
        t2=horloge();
        printf("cost duree=%12.6f pour %d codes\n",t2-t1,nnn);
        */
    }

    if( cam ) {
        codeCam=code;
    }else{
        codeProj=code;
    }
}

#else

void leopard::computeCodes(int cam,int type,cv::Mat *img) {
    printf("-- compute codes cam=%d type=%d n=%d --\n",cam,type,n);

    int w,h;
    unsigned long *code=NULL; // [w*h*nb] [(y*w+x)*nb+b]
    unsigned char *pmask;

    if( cam ) {
        w=wc;h=hc;
        if( codeCam ) {
            free(codeCam);
        }
        if( !maskCam ) { printf("cam: No mask to compute codes!\n");exit(0); }
        pmask=maskCam;
    }else{
        w=wp;h=hp;
        if( codeProj ) {
            free(codeProj);
        }
        if( !maskProj ) { printf("proj: No mask to compute codes!\n");exit(0); }
        pmask=maskProj;
		printf("*** WP=%d HP=%d\n",wp,hp);
    }


    if( type==LEOPARD_SIMPLE ) {
        // compare chaque image a l'image moyenne
        nbb=n;
        nb=(nbb+63)/64; // parce qu'on utilise des long
        printf("nbb=%d nb=%d\n",nbb,nb);
        code=(unsigned long *)malloc(w*h*nb*sizeof(unsigned long));
        for(int i=0;i<w*h*nb;i++) code[i]=0;

        // trouve l'image moyenne
        cv::Mat sum;
        sum.create(h,w,CV_32SC1);
        int *psum=(int *)sum.data;
		for(int j=0;j<w*h;j++) psum[j]=0;
        for(int i=0;i<n;i++) {
            unsigned char *p=img[i].data;
            for(int j=0;j<w*h;j++) psum[j]+= *p++;
        }
        cv::Mat mean;
        mean.create(h,w,CV_16UC1);
        unsigned short *pmean=(unsigned short *)mean.data;
        // 8 bits de precision
        for(int j=0;j<w*h;j++) pmean[j]=(psum[j]*256*2+n)/(2*n);
        //imwrite("meancode.png",mean);


        int ambigu=0;
        int count=0;
        for(int j=0;j<w*h;j++) {
            int b; // which long is it?
            unsigned long mask; // which bit is it in b? (start at bit 0)
            if( pmask[j]==0 ) continue; // tous les codes en dehors du mask sont 0
            count++;
            for(int i=0;i<n;i++) {
                b=i/64;
                mask=1L<<(i%64);
                unsigned char *p=img[i].data;
                unsigned short v=(p[j]<<8)|0x80;
                if( v>pmean[j] || (v==pmean[j] && (rand()&1)) ) code[j*nb+b]|=mask;
                if( v==pmean[j] ) { ambigu++; } // ne sert plus a rien
//                if( j==200*w+300 ) { printf("image[%2d] = %d = %d, mean=%d\n",i,p[j],v,pmean[j]);

//                dumpCode(code + j * nb);
//                printf("\n");printf("\n");}
            }
            if( (j%w)==200 && (j/w)==300 ) {
                printf("pixel (%d,%d) : ",j%w,j/w);
                dumpCode(code+(j*nb));
                printf("\n");
            }
        }

        printf("count=%d (%d %%), ambigu=%d\n",count,count*100/(w*h),ambigu);
    }else if( type==LEOPARD_QUADRATIC ) {
        nbb=n*(n-1)/2; // quadratic!
        nb=(nbb+63)/64; // parce qu'on utilise des long
        printf("nbb=%d nb=%d\n",nbb,nb);
        code=(unsigned long *)malloc(w*h*nb*sizeof(unsigned long));

        for(int i=0;i<w*h*nb;i++) code[i]=0;

        double t1=horloge();
        int b;
        unsigned long mask;
        int nbk=0; // compte les codes
        for(int i=0;i<n;i++) {
            printf("i=%d\n",i);
            unsigned char *pi=img[i].data;

            for(int j=i+1;j<n;j++) {
                //printf("j=%d\n",j);
                unsigned char *pj=img[j].data;
                b=nbk/64;
                mask=1L<<(nbk%64);
                for(int k=0;k<w*h;k++) {
                    if( pmask[k]==0 ) continue;
                    if( pi[k]>pj[k] || (pi[k]==pj[k] && (nbk&1)) ) code[k*nb+b]|=mask;
                }
                nbk++;
            }
        }
        double t2=horloge();
        printf("duree=%12.6f\n",t2-t1);
        //int j=900*w+1200;
        int j = 200 * w + 300;
        while( pmask[j]==0 ) j++;
        printf("pixel %d (%d,%d) : ",j,j%w,j/w);
        dumpCode(code+(j*nb));
        printf("\n");

        /*
        for(int i=0;i<w*h;i++) {
            if( pmask[i]==0 ) continue;
            dumpCodeNum(code+(i*nb));
        }
        */
        /*
        t1=horloge();
        int i;
        int nnn=0;
        for(i=0;i<w*h;i+=1) {
            j=w*h-1-i;
            if( pmask[i]==0 || pmask[j]==0 ) continue;
            int c=cost(code+(i*nb),code+(j*nb));
            //printf("code %d et %d : %d\n",i,j,c);
            nnn++;
        }
        t2=horloge();
        printf("code duree=%12.6f pour %d codes\n",t2-t1,nnn);
        */
    }

    if( cam ) {
        codeCam=code;
    }else{
        codeProj=code;
    }

    // test cost
/*
    int j1=900*w+1200;
    int j2=900*w+1202;
    int k=cost(code+j1*nb,code+j2*nb);
*/

}

#endif

#ifdef USE_GMP
void leopard::statsCodes(int cam) {
    printf("NA0\n");
}
#else
// cam:1=yes,0=no
void leopard::statsCodes(int cam) {

    int w,h;
    unsigned char *mask;
    unsigned long *code;

    if( cam ) {
        w=wc;h=hc;mask=maskCam;code=codeCam;
    }else{
        w=wp;h=hp;mask=maskProj;code=codeProj;
    }

    printf("--- code stats ---\n");
    int maxDist=50;
    int maxL=maxDist*maxDist*2;
    int *hh,*c; // [maxL], sum des couts, compte
    hh=(int *)malloc(maxDist*maxL*sizeof(int)); // [distance^2]
    c=(int *)malloc(maxDist*maxL*sizeof(int)); // [distance^2]

    for(int i=0;i<maxL;i++) hh[i]=c[i]=0;
    hh[0]=0;
    c[0]=1;

    // on compare les codes de pixels voisins
    // on garde h[dist2] 0 1 2 3 4 5 .. MAXD2 pour la difference
    // et c[dist2] pour le compte
    for(int i=0;i<1000000;) {
        if( i%10000==0 ) printf("%d\n",i);
        int x=rand()%w;
        int y=rand()%h;
        if( mask[y*w+x]==0 ) { /*printf("Mx (%d,%d)\n",x,y);*/continue; }
        double r=drand48()*maxDist;
        double theta=drand48()*2.0*M_PI;
        int dx=(int)(cos(theta)*r+0.5);
        int dy=(int)(sin(theta)*r+0.5);
        /*
        int dx=rand()%(2*maxDist+1)-maxDist;
        int dy=rand()%(2*maxDist+1)-maxDist;
        */
        int xx=x+dx;
        int yy=y+dy;
        if( xx<0 || xx>=w || yy<0 || yy>=h ) { /*printf("out\n");*/continue; }
        if( mask[yy*w+xx]==0 ) { /*printf("Mxx\n");*/continue; }
        int L=dx*dx+dy*dy;
        int k=cost(code+(y*w+x)*nb,code+(yy*w+xx)*nb);
        hh[L]+=k;
        c[L]+=1;
        i++;
    }
    char buf[100];
    sprintf(buf,"stat%s.txt",cam?"cam":"proj");
    FILE *f=fopen(buf,"w");
    for(int i=0;i<maxL;i++) {
        double d=sqrt((double)i);
        double m=hh[i];
        if( c[i]==0 ) m=-1.0; else {
            m=hh[i]/(double)c[i];
            fprintf(f,"%12.6f   %12.6f %d\n",d,m,c[i]);
        }
    }
    fclose(f);
}
#endif

void leopard::prepareMatch() {
	// les matchs
    if(matchCam==NULL) matchCam=(minfo *)malloc(wc*hc*sizeof(minfo));
    if(matchProj==NULL) matchProj=(minfo *)malloc(wp*hp*sizeof(minfo));

    // initialise avec le cout le plus eleve pour avoir un match valide mais si inactif
    for(int i=0;i<wc*hc;i++) { matchCam[i].idx=0; matchCam[i].cost=9999;
        matchCam[i].subpx=0.0; matchCam[i].subpy=0.0; matchCam[i].mix=0; }
    for(int i=0;i<wp*hp;i++) { matchProj[i].idx=0; matchProj[i].cost=9999;
        matchProj[i].subpx=0.0; matchProj[i].subpy=0.0; matchProj[i].mix=0; }
}

//test[w*w]
double align(int* test, int* ref, double sx, double sy, int w) {
    int dc=(sx>=0.0)?1:-1;
    int dr=(sy>=0.0)?w:-w;
    double fx=fabs(sx);
    double fy=fabs(sy);
    int sum=0;
    for(int r=1;r<w-1;r++) {
        for(int c=1;c<w-1;c++){
            int pos = r*w+c;
            double v1 = test[pos]*(1-fx)+test[pos+dc]*fx;
            double v2 = test[pos+dr]*(1-fx)+test[pos+dr+dc]*fx;
            double v = v1*(1-fy)+v2*fy;
            double d = v-ref[pos];
            sum += d*d;
        }
    }
    return sum;
}



void leopard::initSP() {
    wDecal=5;
    ptsCam  = (int *) malloc((2*wDecal+1) * (2*wDecal+1) * sizeof(int));
    ptsProj = (int *) malloc((2*wDecal+1) * (2*wDecal+1) * sizeof(int));
}

void leopard::unInitSP(){
    free(ptsCam);
    free(ptsProj);
}

#ifdef USE_GMP
void leopard::unSousPixels(int i) {
    printf("NA1\n");
}
#else
//sousPixels pour le pixel i de la caméra
//le point de départ caméra est un entier et le point d'arrivée projecteur est en sp
void leopard::unSousPixels(int i) {

    if( i%wc==0 ) printf("%d\n",i/wc);
    if( maskCam[i]==0 ) return;
    int j = matchCam[i].idx;
    int jy = j/wp;
    int jx = j%wp;
    if((jx<wDecal) || (jx>=wp-wDecal)) return;
    if((jy<wDecal) || (jy>=hp-wDecal)) return;
    //printf("i = %d, j = %d\n", i, j);

    int pos = 0;
    for(int dr=-wDecal; dr<=wDecal; dr++)
        for(int dc=-wDecal; dc<=wDecal; dc++, pos++) {

            int jj = j+dc+dr*wp;
            //if( maskProj[jj]==0 ) continue;
            int c1  = cost(codeCam+i*nb,codeProj+jj*nb);
            int c2 = cost(codeProj+j*nb,codeProj+jj*nb);
            ptsCam[pos] = c1;
            ptsProj[pos] = c2;
        }

    //choisir le shift qui minimise le cout
    int n = 5;
    double bestv=-1.0;
    int bestsx=0;
    int bestsy=0;
    for(int sy=-(n-1);sy<n;sy++)
        for(int sx=-(n-1);sx<n;sx++){
            double v = align(ptsCam, ptsProj, (double) sx/n, (double) sy/n, 2*wDecal+1);
            if(v<bestv || bestv<0){
                bestv=v;
                bestsx=sx;
                bestsy=sy;
            }
            //printf("%f \n",v);
        }

    matchCam[i].subpx = (double) bestsx/n;
    matchCam[i].subpy = (double) bestsy/n;

}
#endif

void leopard::sousPixels() {
    for(int i=0; i<wc*hc; i++) {
        unSousPixels(i);
    }
}

#ifdef USE_GMP
void leopard::forceBrute(int sp, unsigned char mix) {
    printf("NA2\n");
}
#else
void leopard::forceBrute(int sp, unsigned char mix) {
    double t1=horloge();
	int step=wc*hc/100;
	for(int i=0;i<wc*hc;i++) {
		if( i%step==0 ) printf("%d %%\n",i*100/wc/hc);
		if( maskCam[i]==0 ) continue;
		for(int j=0;j<wp*hp;j++) {
			if( maskProj[j]==0 ) continue;
			int c=cost(codeCam+i*nb,codeProj+j*nb);
            if( c<matchCam[i].cost ) {
                matchCam[i].idx=j;
                matchCam[i].cost=c;
                matchCam[i].mix=mix;
                if(sp) unSousPixels(i);
            }
            if( c<matchProj[j].cost ) {
                matchProj[j].idx=i;
                matchProj[j].cost=c;
                matchProj[i].mix=mix;

            }
		}
	}
    double t2=horloge();
    printf("temps=%12.6f\n",t2-t1);
}
#endif

void leopard::forceBruteCam(int sp, unsigned char mix) {
    double t1=horloge();
	int step=wc*hc/1000;
	for(int i=0;i<wc*hc;i++) {
		if( i%step==0 ) printf("%4.1f %%\n",i*100.0/wc/hc);
		if( maskCam[i]==0 ) continue;
		for(int j=0;j<wp*hp;j++) {
			if( maskProj[j]==0 ) continue;
			int c=cost(codeCam+i*nb,codeProj+j*nb);
            if( c<matchCam[i].cost ) {
                matchCam[i].idx=j;
                matchCam[i].cost=c;
                matchCam[i].mix=mix;
                if(sp) unSousPixels(i);
            }
            if( c<matchProj[j].cost ) {
                matchProj[j].idx=i;
                matchProj[j].cost=c;
                matchProj[i].mix=mix;

            }
		}
	}
    double t2=horloge();
    printf("temps=%12.6f\n",t2-t1);
}
void leopard::forceBruteProj(int sp, unsigned char mix) {
    double t1=horloge();
	int step=wp*hp/1000;
	for(int i=0;i<wp*hp;i++) {
		if( i%step==0 ) printf("%4.1f %%\n",i*100.0/wp/hp);
        int x=i%wp;
        int y=i/wp;
        if( x%2!=0 || y%2!=0 ) continue;
        if( maskProj[i]==0 ) continue;
		for(int j=0;j<wc*hc;j++) {
            if( maskCam[j]==0 ) continue;
			int c=cost(codeCam+j*nb,codeProj+i*nb);
            if( c<matchCam[j].cost ) {
                matchCam[j].idx=i;
                matchCam[j].cost=c;
                matchCam[j].mix=mix;
                if(sp) unSousPixels(j);
            }
            if( c<matchProj[i].cost ) {
                matchProj[i].idx=j;
                matchProj[i].cost=c;
                matchProj[i].mix=mix;
            }
		}
	}
    double t2=horloge();
    printf("temps=%12.6f\n",t2-t1);
}

//sp : 1 = faire du sp
//     0 = pas de sp
//dans tous les cas, seulement la caméra qui a le sp
//mix pour information seulement
int leopard::doLsh(int sp, unsigned char mix) {
    // de cam vers proj, dans les deux directions
    //aisCam == 1
    lsh(0,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,sp?1:-1,mix);
    lsh(1,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,sp?1:-1,mix);
    // de proj vers cam, dans les deux directions
    //aisCam == 0
    lsh(0,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,sp?0:-1,mix);
    lsh(1,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,sp?0:-1,mix);
    return 0;
}


int leopard::doHeuristique() {
    heuristique(codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp);
    heuristique(codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc);
    return 0;
}


//aisCam ==  1  ->  pour codeA est la cam
//aisCam ==  0  ->  pour codeB est la cam
//aisCam == -1  ->  pas de sp
//mix= [0..255] seulement pour se rappeler du mix
#ifdef USE_GMP
int leopard::lsh(int dir,mpz_t *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
                  mpz_t *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb,
                 int aisCam, unsigned char mix) {
#else
int leopard::lsh(int dir,unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
                  unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb,
                 int aisCam, unsigned char mix) {
#endif
    int nv=20; // nb de bits pour le vote
	int nbvote=(1<<nv);
	bminfo bm[nv];
	double now=horloge();

    //printf("lsh nv=%d bits, allocate %d bytes\n",nv,((1<<nv)*sizeof(int)));

	if( vote==NULL) vote=(int *)malloc(nbvote*sizeof(int));
    int *q=vote;
    for(int i=0;i<nbvote;i++) *q++=-1; // pas de vote!

    // choisir nv bits au hasard
	// pas ideal... ca peut repeter des bits...
	for(int i=0;i<nv;i++) {
            int b=rand()%nbb; // the selected bit
#ifdef USE_GMP
            bm[i].byte=b;
#else
            bm[i].byte=b/64;
            bm[i].mask=1L<<(b%64);
#endif
            bm[i].vmask=1<<i;

            //printf("bit %d mask=%d vmask=%d\n",i,bm[i].mask,bm[i].vmask);
	}

    unsigned long *p;

	int start,stop,step;
	if( dir ) { start=0;stop=wa*ha;step=1; }
	else		{ start=wa*ha-1;stop=-1;step=-1; }

    int collision=0;
    int count=0;
	for(int i=start;i!=stop;i+=step) {
			// si on est pas actif.... on saute!
            if( maskA[i]==0 ) continue;
			// ramasse le hashcode
			unsigned int hash=0;
#ifdef USE_GMP
            mpz_t *p=codeA+i;
			for(int k=0;k<nv;k++) if( mpz_tstbit(*p,bm[k].byte) ) hash|=bm[k].vmask;
#else
			p=codeA+i*nb;
			for(int k=0;k<nv;k++) {
                    if( p[bm[k].byte] & bm[k].mask ) hash|=bm[k].vmask;
			}
#endif
            //
            // flip un bit au hasard
            // pour reduire les collisions, et donc obtenir de meilleurs candidats
#ifdef REDUCTION_COLLISIONS
            hash^=(1<<(rand()%nv));
#endif

            //if( hash<0 || hash>=nbvote ) { printf("******* out of bound %d vs %d\n",hash,nbvote);continue; }

            if( vote[hash]>=0 ) collision++;
            count++;
			// basic case... ecrase les autres votes
			//vote[hash]=i;
            // better option??? if there is already a vote, replace if we have a worse best match
			if( vote[hash]<0 || matchA[i].cost>matchA[vote[hash]].cost ) vote[hash]=i;
#if 0
            else{
                for(int g=1;g<10;g++) {
                    if( hash>=(unsigned int)g && vote[hash-g]<0 ) { vote[hash-g]=i;break; }
                    else if( hash<(unsigned int)(nbvote-g) && vote[hash+g]<0 ) { vote[hash+g]=i;break; }
                }
            }
#endif
	}

    // stats
	int holes=0;
	for(int i=0;i<nbvote;i++) if( vote[i]<0 ) holes++;
	printf("espace dans les votes = %d%%\n",(holes*100/nbvote));

    int vide=0;

	// B match
    int redox=0;
    int nbmatchA=0;
    int nbmatchB=0;
	for(int i=0;i<wb*hb;i++) {
			// si on est pas actif.... on saute!
			if( maskB[i]==0 ) continue;
			// ramasse le hashcode
			unsigned int hash=0;
#ifdef USE_GMP
            mpz_t *p=codeB+i;
			for(int k=0;k<nv;k++) if( mpz_tstbit(*p,bm[k].byte) ) hash|=bm[k].vmask;
#else
			p=codeB+i*nb;
			for(int k=0;k<nv;k++) {
				if( p[bm[k].byte]&bm[k].mask ) hash|=bm[k].vmask;
			}
#endif

#ifdef EXTRA_ERREUR_BIT
            for(int k=-1;k<nv;k++) {
#else
            for(int k=-1;k<0;k++) {
#endif

                unsigned int superhash;
                if( k<0 ) superhash=hash;
                else superhash=hash^(1<<k);

                // basic
                int j=vote[superhash];
                if( j<0 ) {vide++; continue;} // aucun vote!

                // match!
#ifdef USE_GMP
                int c=cost(codeA+j,codeB+i);
#else
                int c=cost(codeA+j*nb,codeB+i*nb);
#endif
                if( c<matchA[j].cost ) {
                    nbmatchA++;
                    redox+=matchA[j].cost-c;
                    matchA[j].idx=i;
                    matchA[j].cost=c;
                    matchA[j].mix=mix;
                    if(aisCam==1) unSousPixels(j);
                }
                if( c<matchB[i].cost ) {
                    nbmatchB++;
                    redox+=matchB[i].cost-c;
                    matchB[i].idx=j;
                    matchB[i].cost=c;
                    matchB[i].mix=mix;
                    if(aisCam==0) unSousPixels(i);
                }

            }

#if 0
            // extra search
            for(int g=1;g<10;g++) {
                if( hash>=(unsigned int)g ) {
                    j=vote[hash-g];
                    if( j<0 ) continue;
                    int c=cost(codeA+j*nb,codeB+i*nb);
                    if( c<matchA[j].cost ) { redox+=matchA[j].cost-c;matchA[j].idx=i;matchA[j].cost=c; }
                    if( c<matchB[i].cost ) { redox+=matchB[i].cost-c;matchB[i].idx=j;matchB[i].cost=c; }
                }
                if( hash<=(unsigned char)(nbvote-g) ) {
                    j=vote[hash+g];
                    if( j<0 ) continue; // aucun vote!
                    int c=cost(codeA+j*nb,codeB+i*nb);
                    if( c<matchA[j].cost ) { redox+=matchA[j].cost-c;matchA[j].idx=i;matchA[j].cost=c; }
                    if( c<matchB[i].cost ) { redox+=matchB[i].cost-c;matchB[i].idx=j;matchB[i].cost=c; }
                }
            }
#endif
	}
	now=horloge()-now;

	// somme des couts
    int delta=0,nb=0;
    for(int i=0;i<wa*ha;i++) if(matchA[i].cost<9999) {delta+=matchA[i].cost;nb++;}
    for(int i=0;i<wb*hb;i++) if(matchB[i].cost<9999) {delta+=matchB[i].cost;nb++;}


    printf("time=%12.6f  redox=%10d A=%4d B=%3d Hvide=%3d%% collisions=%3d%% vide= %d (%d,%d) nb= %d \n",
           now,redox,nbmatchA,nbmatchB,holes/(nbvote/100),collision/(count/100), vide,nbvote,count,nb);
    return(delta/(nb/1000));
}


int leopard::sumCost() {
    int sum=0;
    for(int i=0; i<wc*hc; i++) {
        if(matchCam[i].cost < 1000)
            sum+=matchCam[i].cost;
    }
    return sum;
}

#ifdef USE_GMP
void leopard::shiftCodes (int shift, mpz_t *codes, int w, int h) {
    printf("NA3\n");
}
#else
void leopard::shiftCodes (int shift, unsigned long *codes, int w, int h) {
    printf("\n Code Cam \n");
    dumpCode(codes + (700*wc+wc/2) * nb);

    for(int i=0; i<w*h; i++) {
        //la fin du premier code
        unsigned long mem = codes[i*nb] & ((1L<<shift)-1);
        int s = 64;
        for(int j=0; j<nb; j++) {
            if(j < nb-1) {
                codes[i*nb+j] = (codes[i*nb+j] >> shift) | (codes[i*nb+j+1] << (s - shift));
            }
            else {
                s = nbb%64;
                codes[i*nb+j] = (codes[i*nb+j] >> shift) | (mem << (s - shift));
            }
        }
    }


//    for(int i=0; i<w*h; i++) {
//        //la fin du dernier code
//        unsigned long mem = codes[i*nb+nb-1];
//        int s = nbb%64;
//        for(int j=nb-1; j>=0; j--) {
//            if(j > 0) {
//                codes[i*nb+j] = (codes[i*nb+j] >> shift) | (codes[i*nb+j-1] << (s - shift));
//            }
//            else {
//                codes[i*nb+j] = (codes[i*nb+j] >> shift) | (mem << (s - shift));
//            }
//        s = 64;
//        }
//    }

    printf("\n");
    dumpCode(codes + (700*wc+wc/2) * nb);
    printf("\n");
    printf("\n");
}
#endif


#ifdef USE_GMP
int leopard::doShiftCodes() {
    printf("NA4\n");
}
#else
int leopard::doShiftCodes() {
    //create a file
//    string const myFile("/home/chaima/Documents/Mathematica/SumT.txt");
//    std::ofstream sumVide(myFile.c_str());

    int sum[nbb];
    for(int i=0; i<nbb; i++) {

        printf("\n\n--------------------- itr %d ---------%3d%% --------------------- \n\n",
               i, (i*100)/nbb);

        int videCam=0, videProj=0;
        prepareMatch();
        int k;
        for(k=0; k<2; k++) {
             srand(1656+k);
            videCam += lsh(1,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,-1,0);
             srand(1678+k);
            videProj += lsh(1,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,-1,0);
        }
        sum[i] = (videCam + videProj)/(2*k);
        printf("\n sum[%d] = %d \n", i, sum[i]);

        //File Sum.txt
        ////////////////
//        sumVide.open("/home/chaima/Documents/Mathematica/SumT.txt", std::fstream::app);

//        if (!sumVide ) {
//            sumVide.open("/home/chaima/Documents/Mathematica/SumT.txt");
//            sumVide <<"\n";
//            //sumVide << sum[i] << "\n";
//            sumVide.close();

//        }
//        else {
//            sumVide << sum[i-1] <<"\n";
//            if(i == nbb-1) sumVide << sum[i] <<"\n";
//            sumVide.close();
//        }
        ///////////////

        shiftCodes(1, codeCam, wc, hc);
    }

    int min = sum[0];
    printf("\n--------------------- Terminé ! ----- 100%% ---------------------");
    printf("\n minInit = %d", min);
    int pos = 0;
    for(int i=1; i<nbb; i++)
        if(sum[i] < min) { pos = i; min = sum[i];}

    printf("\n min = %d et pos = %d \n", min, pos);

    if((pos != 0)) {
        if (pos > (nbb%64)) {
            int shift = pos;
            while(shift > (nbb%64)) {
                printf("\n shift restant = %d / shift actuel = %d\n", shift, nbb%64);
                shiftCodes(nbb%64, codeCam, wc, hc);
                shift -= (nbb%64);
            }
            if(shift != 0) {
                printf("\n shift restant = %d\n", shift);
                shiftCodes(shift, codeCam, wc, hc);
            }
        }
        else {
            shiftCodes(pos, codeCam, wc, hc);
        }
    }
    dumpCode(codeCam + (700*wc+wc/2) * nb);

    return pos;
}
#endif

#ifdef USE_GMP
int leopard::heuristique(mpz_t *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
						 mpz_t *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb) {
    printf("na\n");
}
#else
int leopard::heuristique(unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
						 unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb) {
	// pour chaque cam[i] -> proj[j]
	// on regarde si on pourrait ameliorer le match vers proj[autour de j]
	// on regarde si on pourrait creer un match cam[autour de i] vers proj[j]
	int di[8]={-1,1,-wa,wa,-1-wa,-1+wa,1-wa,1+wa};
	int dj[8]={-1,1,-wb,wb,-1-wb,-1+wb,1-wb,1+wb};
	double now=horloge();

	int i,j,k;
	for(i=0;i<wa*ha;i++) {
		// skip si pas actif!
		if( maskA[i]==0 ) continue;

		j=matchA[i].idx;
		// regarde autour de j
		for(k=0;k<8;k++) {
			int jj=j+dj[k];
            if( jj<0 || jj>=wb*hb ) continue;
			int c=cost(codeA+i*nb,codeB+jj*nb);
			if( c<matchA[i].cost ) { matchA[i].idx=jj;matchA[i].cost=c; }
			if( c<matchB[jj].cost ) { matchB[jj].idx=i;matchB[jj].cost=c; }
		}
		// regarde autour de i
		for(k=0;k<8;k++) {
			int ii=i+di[k];
			if( ii<0 || ii>=wa*ha ) continue;
			int c=cost(codeA+ii*nb,codeB+j*nb);
			if( c<matchA[ii].cost ) { matchA[ii].idx=j;matchA[ii].cost=c; }
			if( c<matchB[j].cost ) { matchB[j].idx=ii;matchB[j].cost=c; }
		}
		
	}
	now=horloge()-now;
    printf("time = %12.6f\n",now);
	// somme des couts
    /*
	int delta=0;
	for(int i=0;i<wa*ha;i++) delta+=matchA[i].cost;
	for(int i=0;i<wb*hb;i++) delta+=matchB[i].cost;
    return(delta);
    */
	return(0);
}
#endif



int leopard::bitCount(unsigned long n) {
    n = (0x5555555555555555L & (n>>1))  + (0x5555555555555555L & n);
    n = (0x3333333333333333L & (n>>2))  + (0x3333333333333333L & n);
    // a partir de 4, plus de probleme de carry bit parce que 4 utilise 3 bits, donc le 4e est toujours 0
    n = (0x0f0f0f0f0f0f0f0fL & ((n>>4)+n));
    n = (0x00ff00ff00ff00ffL & ((n>>8)+n));
    n = (0x0000ffff0000ffffL & ((n>>16)+n));
    n = (0x00000000ffffffffL & ((n>>32)+n));
    return (int)n;
}

/*
int leopard::bitCount(unsigned long n) {
    int c;
    for(c=0;n;c++) n&=n-1;
    return(c);
}
*/

/*
int leopard::bitCount(unsigned long n) {
    //register unsigned char *p=(unsigned char *)&n;
    //return bt[*p++] + bt[*p++] + bt[*p++] + bt[*p++]
    //     + bt[*p++] + bt[*p++] + bt[*p++] + bt[*p];
    unsigned short *p=(unsigned short *)&n;
    return bt[*p++] + bt[*p++] + bt[*p++] + bt[*p];
}
*/

// genere n bits dans un unsigned long
/*
unsigned long leopard::bitGen(int n)
{
    unsigned long mask=1L;
}
*/

#ifdef USE_GMP
int leopard::cost(mpz_t *a,mpz_t *b) {
    return mpz_hamdist(*a,*b);
}
#else
int leopard::cost(unsigned long *a,unsigned long *b) {
    //printf("A: ");dumpCode(a);printf("\n");
    //printf("B: ");dumpCode(b);printf("\n");
	int c=0;
	for(int i=0;i<nb;i++) c+=bitCount(*a++^*b++);
	//printf("cost=%d\n",c);
    return c;
}
#endif

void leopard::makeLUT(cv::Mat &lut,cv::Mat &imgmix,int cam) {
    if( cam )	{
        match2image(lut,matchCam,maskCam,wc,hc,wp,hp);
        mix2image(imgmix,matchCam,maskCam,wc,hc,wp,hp);
    }
    else {
        match2image(lut,matchProj,maskProj,wp,hp,wc,hc);
        mix2image(imgmix,matchProj,maskProj,wp,hp,wc,hc);
    }
}

// output une image pour le match (imager w x h) vers une image ww x hh
void leopard::match2image(cv::Mat &lut,minfo *match,unsigned char *mask,int w,int h,int ww,int hh) {
    lut.create(h,w,CV_16UC3);
    int x,y,xx,yy,cc,dxx,dyy;
    int i=0;
    for(y=0;y<h;y++) for(x=0;x<w;x++) {
        i=y*w+x;
        xx=match[i].idx%ww;
        yy=match[i].idx/ww;
        cc=match[i].cost;
        dxx=(int) (match[i].subpx*65535);
        dyy=(int) (match[i].subpy*65535);
        //lut->at<Vec3s>(y,x)=Vec3s((xx*65535)/ww,(yy*65535)/hh,(cc*65535)/maxcost);
        if( mask[i]==0 ) {
            lut.at<Vec3s>(y,x)=Vec3s( 65535, 0, 0);
        }else{
            lut.at<Vec3s>(y,x)=Vec3s((cc*65535)/nbb, (yy*65535+dyy)/hh, (xx*65535+dxx)/ww);
        }
    }
}


// output une image pour le mix (imager w x h) vers une image ww x hh
void leopard::mix2image(cv::Mat &imgmix,minfo *match,unsigned char *mask,int w,int h,int ww,int hh) {
    imgmix.create(h,w,CV_8UC1);
    int x,y;
    int i=0;
    for(y=0;y<h;y++) for(x=0;x<w;x++) {
        i=y*w+x;
        if( mask[i]==0 ) {
            imgmix.at<uchar>(y,x)=0;
        }else{
            imgmix.at<uchar>(y,x)=match[i].mix;
        }
    }
}



void leopard::setPathL(int idx,std::string path,const char *filename) {

    std::string newfilename = path + (std::string) filename;

    switch( idx ) {
        case IDX_SCAN_MASKC : fn_scan_maskc=strdup(newfilename.c_str());break;
        case IDX_SCAN_MEANC : fn_scan_meanc=strdup(newfilename.c_str());break;
        case IDX_SCAN_MASKP : fn_scan_maskp=strdup(newfilename.c_str());break;
        case IDX_SCAN_MEANP : fn_scan_meanp=strdup(newfilename.c_str());break;
        default: printf("setPathL: code %d inconnu!!!!!!!!!\n",idx); break;
    }
}

