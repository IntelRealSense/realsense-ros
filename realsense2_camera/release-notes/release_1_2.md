# New features

* Camera node and nodelet manager are started with `respawn` disabled. Current approach is to let the node crash and let the user externally restart the driver
* If no devices are connected on start-up, stop the driver
* Disable frequency diagnostics and implemented replacement:
  * If connection is lost (querying returns no devices), publish diagnostics level STALE
  * If there is a connection, but no frames are received, publish diagnostics level ERROR

# Bug fixes

# Migration guide

# Misc
