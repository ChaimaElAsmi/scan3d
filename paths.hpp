#ifndef PATHS_HPP
#define PATHS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>


//Dossier principal pour contenir tous les output
std::string path = "test/";

/* Capture:
    * FN_CAP_CAM  = images capturées camera
    * FN_CAP_PROJ = patterns de reference projecteur
*/
#define FN_CAP_CAM "subpixel/res_1920x1080/capture/freq_50/cam_%04d.jpg"
#define FN_CAP_PROJ "../../Mathematica/Images/1280x720/Patterns_60/freq_50/leopard_1280_720_%04d.jpg"


/* Scan:
    * FN_SCAN_LUTC  = Lut de la camera
    * FN_SCAN_LUTP  = Lut du projecteur
    * FN_SCAN_MASKC = mask de la camera
    * FN_SCAN_MASKP = mask du projecteur
    * FN_SCAN_MEANC = image moyenne de la camera
    * FN_SCAN_MEANP = image moyenne du projecteur
*/

#define FN_SCAN_LUTC "Output/scan/lut/lutcam_sp.png"
#define FN_SCAN_LUTP "Output/scan/lut/lutproj_sp.png"
#define FN_SCAN_MIXC "Output/scan/lut/mixcam_sp.png"
#define FN_SCAN_MIXP "Output/scan/lut/mixproj_sp.png"

#define FN_SCAN_MASKC "Output/scan/mask/maskCam_sp.png"
#define FN_SCAN_MASKP "Output/scan/mask/maskProj_sp.png"
#define FN_SCAN_MEANC "Output/scan/mask/meanCam_sp.png"
#define FN_SCAN_MEANP "Output/scan/mask/meanProj_sp.png"


/* Triangulation:
    * FN_TR_MASK  = mask triangulation de la camera
    * FN_TR_DATA  = output des données triangulées
    * FN_TR_PARC  = paramètres internes/externes de la camera
    * FN_TR_PARP = paramètres internes/externes du projecteur
*/
#define FN_TR_MASK  "Output/triangulation/mask_sp.png"
#define FN_TR_DATA  "Output/triangulation/data_sp.xml"
#define FN_TR_PARC  "subpixel/res_1920x1080/calibration/data/out_camera_data.xml"
#define FN_TR_PARP  "subpixel/res_1920x1080/calibration/data/out_projector_data.xml"



#endif // PATHS_HPP

