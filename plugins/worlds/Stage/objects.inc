# Desc: Some useful environmental objects
# Author: Richard Vaughan
# Date: 4 Jun 2002
# CVS: $Id$

# a largeish colored cylinder 
#
define visionbeacon box 
(
  shape "circle" 
  size [0.2 0.2] 
  color "red"
)

# primary colored pucks for moving with the gripper
#
define red_puck puck( color "red" )
define green_puck puck( color "green" )
define blue_puck puck( color "blue" )


