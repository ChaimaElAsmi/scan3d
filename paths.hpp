#ifndef PATHS_HPP
#define PATHS_HPP


/* Capture:
    * FN_CAP_CAM  = images capturées camera
    * FN_CAP_PROJ = patterns de reference projecteur
    * CAPTURE     = capturer   -> 1
    * SCAN        = scanner    -> 1
    * TRIANGULE   = trianguler -> 1
*/
#define FN_CAP_CAM "subpixel/res_1920x1080/capture/highF/cam_%04d.jpg"
#define FN_CAP_PROJ "../../Mathematica/Images/1280x720/highF/leopard_1280_720_%04d.jpg"
#define CAPTURE   0
#define SCAN      1
#define TRIANGULE 1


/* Scan:
    * FN_SCAN_LUTC  = Lut de la camera
    * FN_SCAN_LUTP  = Lut du projecteur
    * FN_SCAN_MASKC = mask de la camera
    * FN_SCAN_MASKP = mask du projecteur
    * FN_SCAN_MEANC = image moyenne de la camera
    * FN_SCAN_MEANP = image moyenne du projecteur
*/
#define FN_SCAN_LUTC "subpixel/res_1920x1080/spHF/scan/lut/lutcam.png"
#define FN_SCAN_LUTP "subpixel/res_1920x1080/spHF/scan/lut/lutproj.png"

#define FN_SCAN_MASKC "subpixel/res_1920x1080/spHF/scan/mask/maskCam.png"
#define FN_SCAN_MASKP "subpixel/res_1920x1080/spHF/scan/mask/maskProj.png"
#define FN_SCAN_MEANC "subpixel/res_1920x1080/spHF/scan/mask/meanCam.png"
#define FN_SCAN_MEANP "subpixel/res_1920x1080/spHF/scan/mask/meanProj.png"


/* Triangulation:
    * FN_TR_MASK  = mask triangulation de la camera
    * FN_TR_DATA  = output des données triangulées
    * FN_TR_PCAM  = paramètres internes/externes de la camera
    * FN_TR_PPROJ = paramètres internes/externes du projecteur
    * TR_CAM      = triangulation (Proj -> Cam = 0) , (Cam -> Proj = 1)
*/
#define FN_TR_MASK  "subpixel/res_1920x1080/spHF/triangulation/mask.png"
#define FN_TR_DATA  "subpixel/res_1920x1080/spHF/triangulation/data.xml"
#define FN_TR_PCAM  "calibration/out_camera_data.xml"
#define FN_TR_PPROJ "calibration/out_projector_data.xml"
#define TR_CAM 1


#endif // PATHS_HPP

