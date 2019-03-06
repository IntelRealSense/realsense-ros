# New features

* Camera node and nodelet manager are started with `respawn` enabled. This makes sure the camera is restarted with the node crashed or is killed.
* For timestamps of the image, `std::chrono` is used now. Other alternatives tested:
  * `ros::Time`: This is sensitive to CPU loads.
  * Timestamps from camera frames: Theoretically more accurate, but clock of the camera seems to drift so inaccurate in the long term.

# Bug fixes

# Migration guide

# Misc
