#include <fstream>
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

    for(int i=0;i<65536;i++) bt[i]=bitCountOrig(i);
    //for(int i=0;i<65536;i++) printf("%3d : %3d\n",i,bt[i]);
    initSP();
}

leopard::~leopard() {
	printf("leopard uninit!\n");
	if( maskProj ) free(maskProj);
	if( maskCam ) free(maskCam);
    if( codeCam ) free(codeCam);
    if( codeProj ) free(codeProj);
	if( matchCam ) free(matchCam);
	if( matchProj ) free(matchProj);
    unInitSP();
}


double leopard::horloge() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return((double)tv.tv_sec+tv.tv_usec/1000000.0);
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

void leopard::computeMask(int cam,cv::Mat *img,int nb,double seuil,double bias,int step,int offx,int offy) {
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
	if( cam ) {
		wc=nc;hc=nr;
		maskCam=(unsigned char *)malloc(wc*hc);
		for(int j=0;j<wc*hc;j++) maskCam[j]=0;
		for(int y=offy;y<hc;y+=step) for(int x=offx;x<wc;x+=step) maskCam[y*wc+x]=pmask[y*wc+x];
        // ajuste le mask pour l'image
		for(int j=0;j<wc*hc;j++) if( maskCam[j]==0 ) pmask[j]=0;
        if( n>0 && n!=nb ) { printf("Cam et Proj : pas le meme nombre d'images!!!!\n");exit(0); }
        n=nb;
	}else{
		wp=nc;hp=nr;
		maskProj=(unsigned char *)malloc(wp*hp);
		for(int j=0;j<wp*hp;j++) maskProj[j]=0;
		for(int y=offy;y<hp;y+=step) for(int x=offx;x<wp;x+=step) maskProj[y*wp+x]=pmask[y*wp+x];
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
    if (cam)
        imwrite("maskCam.png",mask);
    else
        imwrite("maskProj.png", mask);
}


void leopard::dumpCode(unsigned long *c) {
    int one=0;
	for(int i=0;i<nbb;i++) {
        int b=i/64;
        unsigned long mask=1L<<(i%64);
        if( c[b]&mask ) {one++;printf("1");} else printf("0");
    }
    printf(" [%d/%d] ",nbb-one,one);
}

void leopard::dumpCodeNum(unsigned long *c) {
	for(int b=0;b<nb;b++) {
        printf("%lu ",c[b]);
    }
    printf("\n");
}


void leopard::computeCodes(int cam,int type,cv::Mat *img) {
    printf("-- compute codes cam=%d type=%d n=%d --\n",cam,type,n);

    int w,h;
    unsigned long *code=NULL; // [w*h*nb] [(y*w+x)*nb+b]
    unsigned char *pmask;

    if( cam ) {
        w=wc;h=hc;
        if( codeCam ) free(codeCam);
        if( !maskCam ) { printf("cam: No mask to compute codes!\n");exit(0); }
        pmask=maskCam;
    }else{
        w=wp;h=hp;
        if( codeProj ) free(codeProj);
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

void leopard::prepareMatch() {
	// les matchs
    if(matchCam==NULL) matchCam=(minfo *)malloc(wc*hc*sizeof(minfo));
    if(matchProj==NULL) matchProj=(minfo *)malloc(wp*hp*sizeof(minfo));

    // initialise avec le cout le plus eleve pour avoir un match valide mais si inactif
    for(int i=0;i<wc*hc;i++) { matchCam[i].idx=0; matchCam[i].cost=9999;
        matchCam[i].subpx=0.0; matchCam[i].subpy=0.0; }
    for(int i=0;i<wp*hp;i++) { matchProj[i].idx=0; matchProj[i].cost=9999;
        matchProj[i].subpx=0.0; matchProj[i].subpy=0.0; }
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

//sousPixels pour le pixel i de la caméra
//le point de départ caméra est un entier et le point d'arrivée projecteur est en sp
void leopard::unSousPixels(int i) {
    double t1=horloge();
    //int step=wc*hc/100;

    //for(int i=0; i<wc*hc; i++) {
//        i=300*wc+300;
//        int ix = i%wc;
//        if((ix<10) || (ix>=wc-10)) continue;
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

        //test un seul pixel
        //if(i==300*wc+300) {
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

        //}
        matchCam[i].subpx = (double) bestsx/n;
        matchCam[i].subpy = (double) bestsy/n;
    //}
    double t2=horloge();
    //printf("subpx temps=%12.6f\n", t2-t1);
}

void leopard::sousPixels() {
    for(int i=0; i<wc*hc; i++) {
        unSousPixels(i);
    }
}


void leopard::forceBrute() {
    double t1=horloge();
	int step=wc*hc/100;
	for(int i=0;i<wc*hc;i++) {
		if( i%step==0 ) printf("%d %%\n",i*100/wc/hc);
		if( maskCam[i]==0 ) continue;
		for(int j=0;j<wp*hp;j++) {
			if( maskProj[j]==0 ) continue;
			int c=cost(codeCam+i*nb,codeProj+j*nb);
			if( c<matchCam[i].cost ) { matchCam[i].idx=j;matchCam[i].cost=c; }
			if( c<matchProj[j].cost ) { matchProj[j].idx=i;matchProj[j].cost=c; }
		}
	}
    double t2=horloge();
    printf("temps=%12.6f\n",t2-t1);
}

int leopard::doLsh() {
    // de cam vers proj, dans les deux directions
    //aisCam == 1
    lsh(0,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,1);
    lsh(1,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,1);
    // de proj vers cam, dans les deux directions
    //aisCam == 0
    lsh(0,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,0);
    lsh(1,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,0);
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
int leopard::lsh(int dir,unsigned long *codeA,minfo *matchA,unsigned char *maskA,int wa,int ha,
                  unsigned long *codeB,minfo *matchB,unsigned char *maskB,int wb,int hb,
                 int aisCam) {
	int nv=20; // nb de bits pour le vote
	int nbvote=(1<<nv);
	bminfo bm[nv];
	double now=horloge();



    //log << "lsh " << nv << " bits, allocate "<< ((1<<nv)*sizeof(int)) << " bytes"<<warn;

	if( vote==NULL) vote=(int *)malloc(nbvote*sizeof(int));
    int *q=vote;
    for(int i=0;i<nbvote;i++) *q++=-1; // pas de vote!

    // choisir nv bits au hasard
	// pas ideal... ca peut repeter des bits...
	for(int i=0;i<nv;i++) {
            int b=rand()%nbb; // the selected bit
            bm[i].byte=b/64;
            bm[i].mask=1L<<(b%64);
            bm[i].vmask=1<<i;

        //log << "bit " << i << " byte="<<(int)bm[i].byte<<", mask="<<(int)bm[i].mask<<", vmask="<<bm[i].vmask<<info;
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
			p=codeA+i*nb;
			unsigned int hash=0;
			for(int k=0;k<nv;k++) {
                                if( p[bm[k].byte] & bm[k].mask ) hash|=bm[k].vmask;
			}
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
	//printf("espace dans les votes = %d%%\n",(holes*100/nbvote));

    int vide=0;

	// B match
    int redox=0;
	for(int i=0;i<wb*hb;i++) {
			// si on est pas actif.... on saute!
			if( maskB[i]==0 ) continue;
			// ramasse le hashcode
			p=codeB+i*nb;
			unsigned int hash=0;
			for(int k=0;k<nv;k++) {
				if( p[bm[k].byte]&bm[k].mask ) hash|=bm[k].vmask;
			}
			// basic
			int j=vote[hash];
            if( j<0 ) {vide++; continue;} // aucun vote!

			// match!
			int c=cost(codeA+j*nb,codeB+i*nb);
            if( c<matchA[j].cost ) {
                redox+=matchA[j].cost-c;
                matchA[j].idx=i;
                matchA[j].cost=c;
                if(aisCam==1) unSousPixels(j);
            }
            if( c<matchB[i].cost ) {
                redox+=matchB[i].cost-c;
                matchB[i].idx=j;
                matchB[i].cost=c;
                if(aisCam==0) unSousPixels(i);
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
    int delta=0;
    for(int i=0;i<wa*ha;i++) delta+=matchA[i].cost;
    for(int i=0;i<wb*hb;i++) delta+=matchB[i].cost;


    printf("time=%12.6f  redox=%10d  vide=%3d%% collisions=%3d%% vide= %d (%d,%d)\n",now,redox,holes/(nbvote/100),
           collision/(count/100), vide,nbvote,count);
    return(vide);
}


int leopard::sumCost() {
    int sum=0;
    for(int i=0; i<wc*hc; i++) {
        if(matchCam[i].cost < 1000)
            sum+=matchCam[i].cost;
    }
    return sum;
}


void leopard::shiftCodes (int shift, unsigned long *codes, int w, int h) {
    printf("\n Code Cam \n");
    dumpCode(codes + 256300 * nb);

    for(int i=0; i<w*h; i++) {
        //la fin du premier code
        unsigned long mem = codes[i*nb];
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
    dumpCode(codes + 256300 * nb);
    printf("\n");
    printf("\n");
}


int leopard::doShiftCodes() {
    //create a file
//    string const myFile("/home/chaima/Documents/Mathematica/Sum.txt");
//    std::ofstream sumVide(myFile.c_str());

    int sum[nbb];
    for(int i=0; i<nbb; i++) {

        printf("\n\n--------------------- itr %d ---------%3d%% --------------------- \n\n",
               i, (i*100)/nbb);

        int videCam=0, videProj=0;
        prepareMatch();
        for(int k=0; k<2; k++) {
             srand(1656+k);
            videCam += lsh(1,codeCam,matchCam,maskCam,wc,hc,codeProj,matchProj,maskProj,wp,hp,-1);
             srand(1678+k);
            videProj += lsh(1,codeProj,matchProj,maskProj,wp,hp,codeCam,matchCam,maskCam,wc,hc,-1);
        }
        sum[i] = videCam + videProj;
        printf("\n sum[%d] = %d \n", i, sum[i]);

        //File Sum.txt
        ////////////////
//        sumVide.open("/home/chaima/Documents/Mathematica/Sum.txt", std::fstream::app);

//        if (!sumVide ) {
//            sumVide.open("/home/chaima/Documents/Mathematica/Sum.txt");
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

    return pos;
}


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


int leopard::cost(unsigned long *a,unsigned long *b) {
    //printf("A: ");dumpCode(a);printf("\n");
    //printf("B: ");dumpCode(b);printf("\n");
	int c=0;
	for(int i=0;i<nb;i++) c+=bitCount(*a++^*b++);
	//printf("cost=%d\n",c);
    return c;
}

void leopard::makeLUT(cv::Mat &lut,int cam) {
	if( cam )	match2image(lut,matchCam,maskCam,wc,hc,wp,hp);
	else		match2image(lut,matchProj,maskProj,wp,hp,wc,hc);
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
            lut.at<Vec3s>(y,x)=Vec3s( 0, 0, 0);
        }else{
            lut.at<Vec3s>(y,x)=Vec3s((cc*65535)/nbb, (yy*65535+dyy)/hh, (xx*65535+dxx)/ww);
        }
    }
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
		delta=heuristique(codeCam,matchCam,wc,hc,codeProj,matchProj,wp,hp);
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


#if 0
unsigned BitCount32(unsigned b)
{
    b = (b & 0x55555555) + (b >> 1 & 0x55555555);
    b = (b & 0x33333333) + (b >> 2 & 0x33333333);
    b = (b + (b >> 4)) & 0x0F0F0F0F;
    b = b + (b >> 8);
    b = (b + (b >> 16)) & 0x0000003F;
    return b;
}
#endif



#if 0
int leopard::cost(unsigned char *a,unsigned char *b,int sz) {
	int i,k=0;
	for(i=0;i<sz;i++) k+=bitCount[a[i]^b[i]];
	return(k);
}
#endif

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
	unsigned int byte; // 0..nb-1 (c'est en fait un unsigned long, donc max bits = 255*64=16384 bits
	unsigned long mask; // 1,2,4,8......1L<<63
	unsigned long vmask; // voting mask (nv bits)
} bminfo;



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




