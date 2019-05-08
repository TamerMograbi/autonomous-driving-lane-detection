# autonomous-driving-lane-detection
lane detection for autonomous driving car using haugh transforms

example of what current output looks like:

![alt text](https://raw.githubusercontent.com/TamerMograbi/autonomous-driving-lane-detection/master/detect_example.png)

still needs work because in the video the trapezoid is jittery

$ On picking the right color space

RGB stops working well when the lighting on the road changes.
(shadow from trees, lighter asphalt, more sunlight, etc...)

HSV keeps the color values and luminance more seperate than RGB.
the same image outdoor and indoor for example will have very similar saturation and value. only the hue will be different.

# algorithm overview

for each frame:

  convert frame to hsv
  
  remove top half of image + above diagonals
  
  binarize image by only keeping yellow and whitle pixels.
  
  detect edges in image
  
  find haugh transform
  
  find haugh peaks (lines in image)
  
  add shape of the trapazoid created by lines to origianl image
