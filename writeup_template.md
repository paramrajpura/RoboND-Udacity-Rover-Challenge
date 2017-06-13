## Project: Search and Sample Return
**The objectives of this project were the following:**  

**Training / Calibration**  
* Use Jupyter Notebook to test the perception function for segmenting the navigable terrain and the rock samples.
* Use the perception functionality using computer vision techniques to plot a global world map.
* Compile a video of frames captured in Training Mode of the simulator.

**Autonomous Navigation / Mapping**
* Followed by the successful test using Jupyter Notebook for the perception module, modify the file 'perception.py' and 'decision.py' to map navigable terrain, navigate autonomously, locate and collect 6 sample rocks to return back to the starting position.


The document addresses the modifications done to the perception and decision modules to complete the "Search and Sample Return Challenge".


## Perception(Filename: perception.py)

The major functionality and approach preferred for the perception module given the video stream from the rover camera is the following:

1. Apply perspective transform to obtain bird's eye view w.r.t rover(Function: perspect_transform(img, src, dst)):
    To obtain the bird's eye perspective, the image is transformed using pre-defined points as reference.
    OpenCV functions for calculating the 3x3 perspective transform matrix and warping are used.
    
2. Segment Navigable terrain, Rock Samples and Obstacles in the image(Function: color_thresh(img, ground_thresh,rock_thresh)):
    The terrain, rock samples and obstacles possess significant differences in their color. Hence to separate and segment them color thresholding the input frame from the rover camera is chosen.
    Since navigable terrain and obstacles are complementary, threshold (160,160,160) for RGB values in 3-channel image is chosen. This separates the navigable terrain and the obstacles.
    Since the rock samples are small and need to be accurately detected, HSV color space is chosen to set the threshold. Few iterations of trial-error led to (20,100,100) as the suitable threshold in HSV.

3. Apply geometric transformation to obtain world map(Function: pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)):
    The co-ordinates w.r.t Rover are rotated and translated to map the explored regions in unknown environment using the current Rover position and Yaw angle available from the sensors.
    Since the perspective transformation accounts for 2D, the pitch and roll are considered zero. Hence the following condiditon check avoids inaccurate mapping to world map.
```     
     if ((Rover.pitch > 359.5 or Rover.pitch < 0.5) and (Rover.roll > 359.5 or Rover.roll < 0.5)):
        Rover.worldmap[obs_y_world.astype(int), obs_x_world.astype(int), 0] += 1
        Rover.worldmap[rock_y_world.astype(int), rock_x_world.astype(int), 1] += 1
        Rover.worldmap[ground_y_world.astype(int), ground_x_world.astype(int), 2] += 1
```
 
 4. Extract navigable angles and distances(Function: to_polar_coords(x_pixel, y_pixel)):
    For navigation, the pixels and the possible navigable region is converted to polar co-ordinate system.
    This approach makes things easier for the decision module to find the direction of navigation in the environment.

## Navigation and Actuation (Filename: decision.py)

The major functionality and approach preferred for the decision module given the perception output i.e map with navigable terrain. obstacles and sample rocks is the following:

1. Navigate forward to explore environment and search for rock samples:
    This branch of the decision tree plays the significant role of navigating ahead given the map visible to the rover camera. The Rover Mode is by default set to 'forward'. Based on the threshold of navigable pixels ahead and the current Rover velocity received from sensor, the control parameters Throttle, Brake and Steer Angle are decided.
    Steer Angle is considered to be the mean of possible navigable angles later clipped and added to constant bias.
```  
    steer_angle = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -10, 10)+8
```
    The mean angle directs the rover towards the most distant navigable terrain while the clipping limits from (-10,10) avoids the rover to circle around in cases where there is open ground.
    The bias is added to direct rover towards the left wall. This helps navigate and map the environment efficiently and avoid obstacles.
    The rover by default turns anti-clockwise by 15 degrees until it finds suitable terrain for progress in cases when it gets stuck by obstacles in the environment.
```
    Rover.steer = -15
```
2. Find alternate path if un-navigable terrain ahead:
    On the basis of navigable terrain visible, the decision is taken to either rotate anti-clockwise as earlier ,apply brake or move forward. 

3. Reach the Rock sample if located in map:
    In case of rock sample visible in current camera frame, following routine is followed 
    - slow down the speed to avoid missing the rock
    - Steer within restricted range to collect rocks. Clipping to -5 restricts rover to move sharply towards right wall and deviate from the path. 
```    
      np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -5, 15)
```

4. Return to the start position after collecting all samples:
    After collecting all 6 rock samples and navigating through the environment, following routine is followed:
    - Keep moving with rover mode as forward until rover reaches near start position.
    - Once closer, set the direction towards the start position.
    - Stop if reaches around start position.

Areas of improvement:
    The rover navigation though autonomous is not intelligent. The decision module doesnt use the previously mapped data which leads to re-visiting explored regions, facing similar obstacles.
An attempt was made to use inverse transformations from world map to obtain explored regions w.r.t rover in current frame. Though the clipping from float to int was leading to inaccurate retrieval of locations.

perception.py includes the functions facilitating use of inverse transformations. The efforts to remove error due to clipping was handled by storing all location values in floating point array. Though efforts were halted later on.
    Seldom the rover gets into repetitive circular navigation depending on random movements due to obstacles and sample placements. This situation can be avoided by tracking rover positions at intervals and activating a random state for navigation in case it is stuck.


The repository contains the code which successfully completes the Udacity Rover Challenge. To reproduce successful runs, Resolution : 1024 x 640
Graphic Quality: Fastest is recommended.
The frame rate was recorded in the range of 45-55 fps. 
Output video of successful challenge completion located at RoboND-Udacity-Rover-Challenge/output/sample_return_challenge.mp4
Video from Ipython notebook located at RoboND-Udacity-Rover-Challenge/output/test-mapping.mp4

