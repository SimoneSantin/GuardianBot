import rclpy
from enum import IntEnum

from turtlebot4_navigation.turtlebot4_navigator import (
   TurtleBot4Directions,
   TurtleBot4Navigator,
)


class TurtleBot4Directions(IntEnum):
	NORTH = 0
	NORTH_WEST = 45
	WEST = 90
	SOUTH_WEST = 135
	SOUTH = 180
	SOUTH_EAST = 225
	EAST = 270
	NORTH_EAST = 315






def main():
   rclpy.init()

   # We will add our code here
   navigator = TurtleBot4Navigator()
    # Start on dock
    
   if not navigator.getDockedStatus():
      navigator.info("Docking before initialising pose")
      navigator.dock()
    # Set initial pose
   initial_pose = navigator.getPoseStamped(
       [0.0, 0.0],
       0,
   )
   navigator.setInitialPose(initial_pose)


    # Set goal poses

   goal_poses = []
   goal_poses.append(navigator.getPoseStamped([0, -4],270))
   goal_poses.append(navigator.getPoseStamped([3, -4],0))
   goal_poses.append(navigator.getPoseStamped([3, 5],90))
   goal_poses.append(navigator.getPoseStamped([0, 5],180))

   # Wait for Nav2
   navigator.waitUntilNav2Active()

   # Undock
   navigator.undock()

   # Go to the goal pose
   navigator.startThroughPoses(goal_poses)



   rclpy.shutdown()


if __name__ == "__main__":
   main()

