# BIM-Tracker
This is a MATLAB implementaton of our upcoming paper "**BIM-Tracker: A model-based visual tracking approach for indoor localisation using a 3D building model**". The provided code is in testing phase and any suggestions regarding improvement and optimisation is welcomed. 

- [ ] Will update the link to the paper and bibliography.

## Running the demo
Run the Demo.m file for one-click demostration on real data with default settings. For running the demo without visualisation of the estimated camera pose, set the attribute `visualiseFrame = false`. To disable the online trajectory visualisation option set the attribute `visualiseTrajectory = fasle`. Setting `AccurateMode = true` will increase the ransac samples to guarantee solution and will override maximum MSAC runs settings. Setting `FastMode = true` will allow jump out loop if required confidence reached and reduce the number of loop iterations for faster convergence.

## YouTube video demostration - Click below image
[![Watch the video](https://img.youtube.com/vi/cq7mk4mfdRA/maxresdefault.jpg)](https://youtu.be/cq7mk4mfdRA)

## What will be loaded
The aux_data_real.mat file contain the image edge files that were extracted by Canny edge detector and visible edges of the BIM in the field-of-view of the camera that are being used for matching. The data structure of the visible edges are in the following order of `[lineId, X1, Y1, Z1, X2, Y2, Z2]`, containing the XYZ locations of two endpoints.

## Using you own data
To test with you own data, it is required to generate the edge images and the visible edges of the BIM in the field-of-view of the camera. The visible edge rendering was performed using [Blender](www.blender.org). For convinience, it is recommended to extract the visible edges offline by defining an approximate trajectory in the BIM in the following manner:

- Import your BIM in IFC format into Belnder.
- Combine all the elements of the model together to form a single mesh. Next, simplyfy the mesh by *limited dissolve* to merge the trianges into planes.
- Configure the virtual camera, such as focal length, sensor size and projection model.
- Define an approximate trajectory and set the approximate number of frames for the test case.
- Run the provided python script `edge_render.py` in the **extras** folder. For more information on the script visit [here](https://blender.stackexchange.com/questions/77607/how-to-get-the-3d-coordinates-of-the-visible-vertices-in-a-rendered-image-in-ble).
  - The script will generate three text files containing the approximate trajectory, all the vertices of the BIM and the visible vertices per frame.
- Use the MATLAB script in the **extras** folder called `edges_with_vertices.m`to converts the text files generated into visible edges for each frame.

An example Blender file containing the simplified BIM, an approximate trajectory for the demo and the camera parameters is present in the **extras** folder in the name of `BlenderExample.blend`. If you use the BIM or the IFC file kindly cite our work:

````````@article{khoshelham2017isprs,
  title={THE ISPRS BENCHMARK ON INDOOR MODELLING.},
  author={Khoshelham, Kourosh and Vilari{\~n}o, L D{\'\i}az and Peter, Michael and Kang, Zhizhong and Acharya, Debaditya},
  journal={International Archives of the Photogrammetry, Remote Sensing \& Spatial Information Sciences},
  volume={XLII-2/W7},
  year={2017}
  pages={367-372},
  doi={https://doi.org/10.5194/isprs-archives-XLII-2-W7-367-2017}, 
}````````

- [ ] Folder extras will be updated soon
