#ifndef PATHS_HPP
#define PATHS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>


//Dossier principal pour contenir tous les output
std::string path = "/home/chaima/Documents/raspberryPi/capture/";
#define NUM  ""
#define CAP  "leop2/"
#define LUT  "output2/scan/lut/"
#define MASK "output2/scan/mask/"
#define TRG  "output2/triangulation/"

/* Capture:
    * FN_CAP_CAM  = images capturées camera
    * FN_CAP_PROJ = patterns de reference projecteur
*/
#define FN_CAP_CAM  CAP "%08d.png"
#define FN_CAP_PROJ "../../scanGit/scan3d/Project/Experiments/scan60/patterns/leopard_1280_720_%04d.jpg"


/* Scan:
    * FN_SCAN_LUTC  = Lut de la camera
    * FN_SCAN_LUTP  = Lut du projecteur
    * FN_SCAN_MASKC = mask de la camera
    * FN_SCAN_MASKP = mask du projecteur
    * FN_SCAN_MEANC = image moyenne de la camera
    * FN_SCAN_MEANP = image moyenne du projecteur
*/

#define FN_SCAN_LUTC LUT "lutcam"  NUM ".png"
#define FN_SCAN_LUTP LUT "lutproj" NUM ".png"
#define FN_SCAN_MIXC LUT "mixcam"  NUM ".png"
#define FN_SCAN_MIXP LUT "mixproj" NUM ".png"

#define FN_SCAN_MASKC MASK "maskCam"  NUM ".png"
#define FN_SCAN_MASKP MASK "maskProj" NUM ".png"
#define FN_SCAN_MEANC MASK "meanCam"  NUM ".png"
#define FN_SCAN_MEANP MASK "meanProj" NUM ".png"


/* Triangulation:
    * FN_TR_MASK  = mask triangulation de la camera
    * FN_TR_DATA  = output des données triangulées
    * FN_TR_PARC  = paramètres internes/externes de la camera
    * FN_TR_PARP = paramètres internes/externes du projecteur
*/
#define FN_TR_MASK  TRG "mask" NUM ".png"
#define FN_TR_DATA  TRG "data" NUM ".xml"
#define FN_TR_PARC  "calibration/out_camera_data.xml"
#define FN_TR_PARP  "calibration/out_projector_data.xml"



#endif // PATHS_HPP

