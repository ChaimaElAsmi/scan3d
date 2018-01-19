#ifndef PATHS_HPP
#define PATHS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>


//Dossier principal pour contenir tous les output
std::string path = "Experiments/scanSync/";

/* Capture:
    * FN_CAP_CAM  = images capturées camera
    * FN_CAP_PROJ = patterns de reference projecteur
*/
#define FN_CAP_CAM  "capture/cam_%04d.jpg"
#define FN_CAP_PROJ "patterns/leopard_1280_720_%04d.jpg"


/* Scan:
    * FN_SCAN_LUTC  = Lut de la camera
    * FN_SCAN_LUTP  = Lut du projecteur
    * FN_SCAN_MASKC = mask de la camera
    * FN_SCAN_MASKP = mask du projecteur
    * FN_SCAN_MEANC = image moyenne de la camera
    * FN_SCAN_MEANP = image moyenne du projecteur
*/

#define FN_SCAN_LUTC "output/scan/lut/lutcam.png"
#define FN_SCAN_LUTP "output/scan/lut/lutproj.png"
#define FN_SCAN_MIXC "output/scan/lut/mixcam.png"
#define FN_SCAN_MIXP "output/scan/lut/mixproj.png"

#define FN_SCAN_MASKC "output/scan/mask/maskCam.png"
#define FN_SCAN_MASKP "output/scan/mask/maskProj.png"
#define FN_SCAN_MEANC "output/scan/mask/meanCam.png"
#define FN_SCAN_MEANP "output/scan/mask/meanProj.png"


/* Triangulation:
    * FN_TR_MASK  = mask triangulation de la camera
    * FN_TR_DATA  = output des données triangulées
    * FN_TR_PARC  = paramètres internes/externes de la camera
    * FN_TR_PARP = paramètres internes/externes du projecteur
*/
#define FN_TR_MASK  "output/triangulation/mask.png"
#define FN_TR_DATA  "output/triangulation/data.xml"
#define FN_TR_PARC  "../calibration/data/out_camera_data.xml"
#define FN_TR_PARP  "../calibration/data/out_projector_data.xml"



#endif // PATHS_HPP

