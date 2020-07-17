fastfusion
==========

Volumetric 3D Mapping in Real-Time on a CPU 

This code implements the approach for real-time 3D mapping on a CPU as
described in the following research paper:

http://vision.in.tum.de/_media/spezial/bib/steinbruecker_etal_icra2014.pdf

Volumetric 3D Mapping in Real-Time on a CPU (F. Steinbruecker, J. Sturm, D. Cremers), 
In Int. Conf. on Robotics and Automation, 2014.

![alt tag](http://vision.in.tum.de/_media/data/software/fastfusion_small.png)

Demo video:
http://youtu.be/7s9JePSln-M

```
@string{icra="Int. Conf. on Robotics and Automation"}
@inproceedings{Steinbruecker-etal-icra14,
  author = {F. Steinbruecker and J. Sturm and D. Cremers},
  title = {Volumetric 3D Mapping in Real-Time on a CPU},
  booktitle = icra,
  year = {2014},
  address = {Hongkong, China},
  titleurl = {steinbruecker_etal_icra2014.pdf},
  topic = {3D Reconstruction},
  keywords =  {RGB-D,Fusion,3d-reconstruction}
}
```

Installation
============

    $ sudo apt install qtbase5-dev libglew-dev freeglut3-dev libqglviewer-dev-qt5 libqglviewer-headers ros-melodic-depth-image-proc ros-melodic-image-geometry ros-melodic-pcl-ros

    $ git clone https://github.com/madsciencetist/fastfusion.git -b rosbag

    $ cd fastfusion
  
    $ cmake .

    $ make

Running the code
================

    $ ./bin/onlinefusion rosbag.bag

After some debugging output on the console, a window with a 3D viewer should open. To start the 
reconstruction process, press "S". 

If you run the program for the first time, press and hold the CTRL key and turn your scroll wheel. 
This is only needed once to "free" the camera viewpoint. After this, you can pan (right click) and 
rotate (left click) the view as you wish using your mouse.

Further options
===============

```
   ./bin/onlinefusion  [--imagescale <float>]
                       [--threshold <float>] [--scale <float>]
                       [--max-camera-distance <float>]
                       [--min-depth <float>]
                       [--consistency-checks <int>] [-k <int>] [-e <int>]
                       [-s <int>] [--incremental-meshing] [-c] [-b] [-v]
                       [--thread-image] [--thread-fusion]
                       [--thread-meshing] [-l <string>] [--] [--version]
                       [-h] <string> ...


Where: 

   --imagescale <float>
     Image Depth Scale

   --threshold <float>
     Threshold

   --scale <float>
     Size of the Voxel

   --max-camera-distance <float>
     Maximum Camera Distance to Surface

   --min-depth <float>
     Minimum depth measurement considered valid

   --consistency-checks <int>
     Number of Depth Consistency Checks

   -k <int>,  --imagestep <int>
     Use every kth step

   -e <int>,  --endimage <int>
     Number of the End Image

   -s <int>,  --startimage <int>
     Number of the Start Image

   --incremental-meshing
     Perform incremental Meshing

   -c,  --loopclosures
     Read Multiple Trajectories and perform Loop Closures

   -b,  --buffer
     Buffer all Images

   -v,  --viewer
     Show a Viewer after Fusion

   --thread-image
     Thread reading the Images from Hard Disk

   --thread-fusion
     Thread the Fusion inside the Viewer

   --thread-meshing
     Thread the Meshing inside the Fusion

   -l <string>,  --loadmesh <string>
     Loads this mesh

   --,  --ignore_rest
     Ignores the rest of the labeled arguments following this flag.

   --version
     Displays version information and exits.

   -h,  --help
     Displays usage information and exits.

   <string>  (accepted multiple times)
     The rosbag filenames
```
![alt tag](http://vision.in.tum.de/_media/data/software/screenshot_fastfusion.png)

