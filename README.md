# Fast dominant plane segmentation algorithm for MATLAB

A plane detection method based on depth information from a Kinect camera. The plane parameters are obtained by segmentation in normal space followed by refinement in distance space. 

+ Depth registration aligns depth and color data
+ Segmentation in normal space
+ Refinement in distance space
+ The obtained plane parameters are used to segment objects

Holz, D., Holzer, S., Rusu, R. B., and Behnke, S. (2011). Real-time plane segmentation using rgb-d cam-
eras. In RoboCup 2011: robot soccer world cup XV, pages 306–317. Springer.

## Object segmentation results
![Egomotion estimation](https://raw.githubusercontent.com/ldelange/plane-detection/master/objects.png)
