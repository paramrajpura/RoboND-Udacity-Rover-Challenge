import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
	# Create an array of zeros same xy size as img, but single channel
    ground = np.zeros_like(img[:,:,0])
    rock = np.zeros_like(img[:,:,0])
    obs = np.zeros_like(img[:,:,0])
    # Threshold the image
    # For ground use RGB values
    # For rock, use HSV since that is stable and dependable
    # For obstacles, either invert or threshold
    rock_thresh = (20,100,100)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    ground = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    rock = (hsv[:, :, 0] > rock_thresh[0]) \
           & (hsv[:, :, 1] > rock_thresh[1]) \
           & (hsv[:, :, 2] > rock_thresh[2])
    obs = (np.logical_and(img[:, :, 0] > 0,img[:, :, 0] <= rgb_thresh[0])) \
             & (np.logical_and(img[:, :, 1] > 0,img[:, :, 1] <= rgb_thresh[1])) \
             & (np.logical_and(img[:, :, 2] > 0,img[:, :, 2] <= rgb_thresh[2]))
    # Return the binary images for ground, rock and obstacles
    return obs,ground,rock

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

#Function to receive data back from global map explored till the instance
# def inv_rotate_pix(xrot, yrot, yaw):
#     yaw_rad = yaw * np.pi / 180
#     xpix = xrot * np.cos(yaw_rad) + yrot * np.sin(yaw_rad)
#     ypix = -xrot * np.sin(yaw_rad) + yrot * np.cos(yaw_rad)
#     return xpix,ypix
#
# def inv_translate_pix(world_x,world_y,xpos,ypos,scale):
#     xpix_rot = (world_x - xpos)*scale
#     ypix_rot = (world_y - ypos)*scale
#     return xpix_rot,ypix_rot

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    # Perform translation and convert to integer since pixel values can't be float
    xpix_translated = xpos + (xpix_rot / scale)
    ypix_translated = ypos + (ypix_rot / scale)
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    return xpix_tran,ypix_tran

#Clipping separated to accomodate inverse functions
def clip_for_world(xpix_tran,ypix_tran,world_size):
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obs,ground,rock = color_thresh(warped)
    obs_disp, ground_disp, rock_disp = color_thresh(Rover.img)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs_disp*255
    Rover.vision_image[:,:,1] = rock_disp*255
    Rover.vision_image[:,:,2] = ground_disp*255
    # 5) Convert map image pixel values to rover-centric coords
    x_obs, y_obs = rover_coords(obs)
    x_rock, y_rock = rover_coords(rock)
    x_ground, y_ground = rover_coords(ground)
    # 6) Convert rover-centric pixel values to world coordinates
    x_ground_tran, y_ground_tran = pix_to_world(x_ground, y_ground, Rover.pos[0],
                 Rover.pos[1], Rover.yaw, Rover.map_scale)

    ground_x_world,ground_y_world = clip_for_world(x_ground_tran,y_ground_tran,Rover.worldmap.shape[0])
    obs_x_world,obs_y_world = clip_for_world(pix_to_world(x_obs, y_obs, Rover.pos[0],
                                  Rover.pos[1], Rover.yaw, Rover.map_scale),Rover.worldmap.shape[0])
    x_rock_tran,y_rock_tran = pix_to_world(x_rock, y_rock, Rover.pos[0],
                 Rover.pos[1], Rover.yaw, Rover.map_scale)
    rock_x_world,rock_y_world = clip_for_world(x_rock_tran,y_rock_tran,Rover.worldmap.shape[0])

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Validate the pitch and roll for accurate mapping
    if ((Rover.pitch > 359.5 or Rover.pitch < 0.5) and (Rover.roll > 359.5 or Rover.roll < 0.5)):
        Rover.worldmap[obs_y_world.astype(int), obs_x_world.astype(int), 0] += 1
        Rover.worldmap[rock_y_world.astype(int), rock_x_world.astype(int), 1] += 1
        Rover.worldmap[ground_y_world.astype(int), ground_x_world.astype(int), 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles, pass rock dists and angles if sample in frame
    if len(rock_x_world) > 0:
        dist, angles = to_polar_coords(x_rock, y_rock)
        Rover.nav_dists = dist
        Rover.nav_angles = angles
        Rover.sample_detected = True
    else:
        dist, angles = to_polar_coords(x_ground, y_ground)
        Rover.nav_dists = dist
        Rover.nav_angles = angles

    #Code for inverse transformation to get data from explored world map
    # Rover.explored_x.extend(x_ground_tran)
    # Rover.explored_y.extend(y_ground_tran)
    # print('here',len(Rover.explored_x),len(Rover.explored_y))
    # world_x,world_y = np.asarray(Rover.explored_x),np.asarray(Rover.explored_y)
    # world_rotx,world_roty = inv_translate_pix(world_x,world_y,Rover.pos[0],Rover.pos[1],Rover.map_scale)
    # xpix,ypix = inv_rotate_pix(world_rotx,world_roty,Rover.yaw)
    #
    # xpixclip = np.clip(np.int_(xpix), 0, Rover.vision_image.shape[0] - 1)
    # ypixclip = np.clip(np.int_(ypix), 0, Rover.vision_image.shape[1] - 1)
    #
    # with open("Output.txt", "a") as text_file:
    #     print("xpos",len(xpixclip),np.min(xpixclip),np.max(xpixclip),xpixclip,file=text_file)
    #     print("ypos", len(ypixclip),np.min(ypixclip),np.max(ypixclip),ypixclip, file=text_file)
    #     print("xground", len(x_ground), np.min(x_ground), np.max(x_ground),x_ground, file=text_file)
    #     print("yground", len(y_ground), np.min(y_ground), np.max(y_ground),y_ground, file=text_file)
    #Rover.vision_image[xpixclip.astype(int), ypixclip.astype(int), 2] = 255
    return Rover
