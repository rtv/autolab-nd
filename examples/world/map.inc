# map.inc - useful setup for a floorplan bitmap
# Authors: Richard Vaughan
# $Id: map.inc,v 1.4 2009-03-31 23:53:00 jwawerla Exp $

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_movemask 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)

define zone model
(
  color "orange"
  size [ 2 2 0.02 ]

  gui_nose 0
  gui_grid 0
  gui_movemask 1
  gui_outline 0

  # insensible to collision and range sensors
  obstacle_return 0
  laser_return 0
  ranger_return 0
)

define redraw model
(
  color "orange"
  size [ 0.1 0.1 0.02 ]
  gui_nose 0
  gui_grid 0
  gui_movemask 0
  gui_outline 0

  # insensible to collision and range sensors
  obstacle_return 0
  laser_return 0
  ranger_return 0
)


define barrier model
(
  color "black"
  size [6.000 0.050 1.000]
  pose [ -1.5 -0.500 0 0 ]

  obstacle_return 1
  laser_return 1
  ranger_return 1
)
