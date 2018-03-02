# scan3d

Code de scan 3D

* cam1 et proj1 servent d'exemple pour le match

°Librairie Leopard:

Input : 
	- Images capturées par la caméra en gris ou BGR
	- Images de référence du projecteur


Output :
	- Lut; correspondance de cam vers proj 
	- Lut; correspondance de proj vers cam

Fonctions :
	- Match entre une caméra et un projecteur avec LSH
	- Capture synchronisée 
	- Capture non synchronisée 
	- Match sous-pixels


°Libraire Triangulation:

Input :
	- lutcam
	- lutproj
	- paramètres internes et externes de la caméra et du projecteur 
	- la distorsion radiale de la caméra

Output : 
	- data.xml; les données triangulées

Fonctions :
	- Compute correspondance
	- Undistort
	- Triangulate

