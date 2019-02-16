# BIM-Tracker
This is a MATLAB implementaton of "BIM-Tracker: A model-based visual tracking approach for indoor localisation using a 3D building model". The provided code is in testing phase and any suggestions regarding improvement and optimisation is welcomed.

The aux_data_real.mat file contain the image edge files that were extracted by Canny edge detector and visible edges of the BIM in the field-of-view of the camera that are being used for matching. The structure of the visible edges are in the following order of [lineId, X1, Y1, Z1, X2, Y2, Z2], containing the XYZ locations of two endpoints. 

Run the Demo.m file for one-click demostration on real data. Below is a link to a Youtube video demostrating the approach.

[![Watch the video](https://img.youtube.com/vi/cq7mk4mfdRA/maxresdefault.jpg)](https://youtu.be/cq7mk4mfdRA)
