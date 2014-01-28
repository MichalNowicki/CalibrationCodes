Extrinsix calibration toolbox for a camera and laser range finder

Place a big checkerboard in front of the system of a camera and laser
range finder. Make sure:

1) the entire checkerboard is visible for the camera and 
2) the center (!) laser range finder point hits the checkerboard.
     - this is because we compute the intersection of the laser plane and the    checkerboard by starting at the center point and going out until we find a     discontinuity.

3) capture the data simultaneously from both the camera and the laser range         finder in the different checkerboard poses.
4) Put all image and laser data in the current working directory. image
   data is JPEG file and laser data is postfixed as '.log'. 

5) this toolbox works with (and requires) camera calibration toolbox from
     http://www.vision.caltech.edu/bouguetj/calib_doc/
6) that toolbox (usually called "TOOLBOX_calib", must be in your matlab path.
7) The main call is to "extCalib", you must call it with the K,kc parameters (the camera intrinsic calibration and curvature correction), from the calibration toolbox above.


%%%%%%%%%%%%%%%%%%%% Example execution: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

This readme file is in a directory includes a subdirectory "Sample".

change your matlab working directory to that ("cd sample" if this is your current directory).

copy the definition of K,kc from the txt file "camera_intrinsics.txt" into the matlab command window.

type:
 
[Phi,Delta] = extCalib(K,kc);

... it will ask you for the dimensions of the squares in the calibration grid, the sample data comes from a grid with squares 76mm (the default), so just press return.


Run time on a mid-range laptop is much less than a minute.


Good luck!
Robert Pless (working with Qilong Zhang).


