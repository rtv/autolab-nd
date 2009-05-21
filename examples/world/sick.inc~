
define sicklaser laser
(
  # laser-specific properties
  # factory settings for LMS200	
  range_min 0.0
  range_max 8.0
  fov 180.0
  samples 361

  #samples 90 # still useful but much faster to compute

  # generic model properties
  color "blue"
  #size [ 0.156 0.155 0.19 ] # dimensions from LMS200 data sheet
  #size [ 0.34 0.37  0.19 ] # make the laser the same size as the robot
  size [0.44 0.38 0.19]

  block(
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.19]
  )
)

# extends sicklaser to add nice-looking but relatively expensive geometry
define fancysicklaser sicklaser
(
  # bottom
  block(
    points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0 0.02 ]
  )

  # back
  block(
    points 4
    point[0] [ -0.078 -0.077 ]
    point[1] [ -0.02  -0.077 ]
    point[2] [ -0.02   0.077 ]
    point[3] [ -0.078  0.077 ]
    z [0 0.21 ]
  )

  # top
  block( points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0.12 0.21 ]  
  )

  # laser bit
  block( points 4
    point[0] [ -0.02 -0.05 ]
    point[1] [  0.06 -0.05 ]
    point[2] [  0.06 7 0.05 ]
    point[3] [ -0.02  0.05 ]
    z [0.02 0.12 ]
   color "gray10"
  )
)



