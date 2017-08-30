import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
'''def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
'''
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh_ground(img, rgb_thresh=(180, 180, 180)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_obstacle(img):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    
    upper_rgb_thresh = (70,70,70)
    lower_rgb_thresh = (20,20,20)
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh =  ((img[:,:,0] >= lower_rgb_thresh[0]) & (img[:,:,0] <= upper_rgb_thresh[0])) \
                & ((img[:,:,1] >= lower_rgb_thresh[1]) &(img[:,:,1] <= upper_rgb_thresh[1])) \
                & ((img[:,:,2] >= lower_rgb_thresh[2]) & (img[:,:,2] <= upper_rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_rock(img):
    # Find rock in the map, it has a upper and lower rgb that is a range of yellow
    # Create an array of zeros same xy size as img, but single channel
    color_select_rgb = np.zeros_like(img[:,:,0])
    
    upper_rgb_thresh_rock = (180,180,70)
    lower_rgb_thresh_rock = (110,100,0)
    
    above_thresh_rock = ((img[:,:,0] >= lower_rgb_thresh_rock[0]) & (img[:,:,0] <= upper_rgb_thresh_rock[0])) \
                & ((img[:,:,1] >= lower_rgb_thresh_rock[1]) &(img[:,:,1] <= upper_rgb_thresh_rock[1])) \
                & ((img[:,:,2] >= lower_rgb_thresh_rock[2]) & (img[:,:,2] <= upper_rgb_thresh_rock[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select_rgb[above_thresh_rock] = 1
    
    # Return the binary image
    return color_select_rgb

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
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
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    
    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw
    
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    warped = perspect_transform(img, source, destination)
    colorrock = color_thresh_rock(warped)
    colornavigable = color_thresh_ground(warped)
    colorobstacle = color_thresh_obstacle(warped)
    
    Rover.vision_image[:,:,0] = colorobstacle * 255
    Rover.vision_image[:,:,1] = colorrock * 255
    Rover.vision_image[:,:,2] = colornavigable * 255
        
    xpix_navigable, ypix_navigable = rover_coords(colornavigable)
    xpix_obstacle, ypix_obstacle = rover_coords(colorobstacle)
    xpix_rock, ypix_rock = rover_coords(colorrock)
    scale = 10
    
    navigable_x_world, navigable_y_world = pix_to_world(xpix_navigable, 
                                                        ypix_navigable,     
                                                        rover_xpos, 
                                                        rover_ypos, 
                                                        rover_yaw,  
                                                        Rover.worldmap.shape[0], 
                                                        scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacle, 
                                                      ypix_obstacle, 
                                                      rover_xpos, 
                                                      rover_ypos, 
                                                      rover_yaw, 
                                                      Rover.worldmap.shape[0], 
                                                      scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, 
                                              ypix_rock, 
                                              rover_xpos, 
                                              rover_ypos, 
                                              rover_yaw, 
                                              Rover.worldmap.shape[0], 
                                              scale)
        
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1 #red color is obstacle
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1 #green color is rock
    Rover.worldmap[navigable_y_world, navigable_x_world, 0] = 0 #blue color is navigable
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1 #blue color is navigable
    

    ypix_navigable_left_weighted = ypix_navigable
    if (len(ypix_navigable) > 0):
        # It tries to keep to the left, only enter if there is enough space for the rover
        if (abs(np.max(ypix_navigable) - np.mean(ypix_navigable)) > 15):
            # It tries to keep 1/3 more to the left
            ypix_navigable_left_weighted = (2*np.mean(ypix_navigable) + np.max(ypix_navigable))/3
    dist, angles = to_polar_coords(xpix_navigable, ypix_navigable_left_weighted)
    dist_to_rock, angles_to_rock = to_polar_coords(xpix_rock, ypix_rock)
    
    # Set the angles for navigation but also direction if there are rocks

    Rover.nav_dists = dist
    Rover.nav_angles = angles
    Rover.nav_dists_rock = dist_to_rock
    Rover.nav_angles_rock = angles_to_rock
     
    
    return Rover