#ifndef PATHS_HPP
#define PATHS_HPP


/* Capture:
    * FN_CAP_CAM  = images capturées camera
    * FN_CAP_PROJ = patterns de reference projecteur
    * CAPTURE     = capturer   -> 1 sinon 0
    * SCAN        = scanner    -> 1 sinon 0
    * TRIANGULE   = trianguler -> 1 sinon 0
*/
#define FN_CAP_CAM "subpixel/res_1920x1080/capture/freq_50/cam_%04d.jpg"
#define FN_CAP_PROJ "../../Mathematica/Images/1280x720/Patterns_60/freq_50/leopard_1280_720_%04d.jpg"
#define CAPTURE   0
#define SCAN      1
#define TRIANGULE 1


/* Scan:
    * SP            = avec du souspixel -> 1 sinon 0
    * FN_SCAN_LUTC  = Lut de la camera
    * FN_SCAN_LUTP  = Lut du projecteur
    * FN_SCAN_MASKC = mask de la camera
    * FN_SCAN_MASKP = mask du projecteur
    * FN_SCAN_MEANC = image moyenne de la camera
    * FN_SCAN_MEANP = image moyenne du projecteur
*/
#define SP 1

#define FN_SCAN_LUTC "subpixel/res_1920x1080/forceBrute/sp/scan/lut/lutcam.png"
#define FN_SCAN_LUTP "subpixel/res_1920x1080/forceBrute/sp/scan/lut/lutproj.png"
#define FN_SCAN_MIXC "subpixel/res_1920x1080/forceBrute/sp/scan/lut/mixcam.png"
#define FN_SCAN_MIXP "subpixel/res_1920x1080/forceBrute/sp/scan/lut/mixproj.png"

#define FN_SCAN_MASKC "subpixel/res_1920x1080/forceBrute/sp/scan/mask/maskCam.png"
#define FN_SCAN_MASKP "subpixel/res_1920x1080/forceBrute/sp/scan/mask/maskProj.png"
#define FN_SCAN_MEANC "subpixel/res_1920x1080/forceBrute/sp/scan/mask/meanCam.png"
#define FN_SCAN_MEANP "subpixel/res_1920x1080/forceBrute/sp/scan/mask/meanProj.png"


/* Triangulation:
    * FN_TR_MASK  = mask triangulation de la camera
    * FN_TR_DATA  = output des données triangulées
    * FN_TR_PCAM  = paramètres internes/externes de la camera
    * FN_TR_PPROJ = paramètres internes/externes du projecteur
    * TR_CAM      = triangulation (Proj -> Cam = 0) , (Cam -> Proj = 1)
*/
#define FN_TR_MASK  "subpixel/res_1920x1080/forceBrute/sp/triangulation/mask.png"
#define FN_TR_DATA  "subpixel/res_1920x1080/forceBrute/sp/triangulation/data.xml"
#define FN_TR_PCAM  "subpixel/res_1920x1080/calibration/data/out_camera_data.xml"
#define FN_TR_PPROJ "subpixel/res_1920x1080/calibration/data/out_projector_data.xml"
#define TR_CAM 1


#endif // PATHS_HPP

