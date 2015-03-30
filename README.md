##COMMON EXEKUTOR

This package provides services common to all robots.
Currently, the following services are available.

|S.No. |Class name|Format of Parameter Tuple|Summary|
|-----:|:---------|:------------------------|:------|
|1| AcquireExekutor | all/known/$object_name | Acquire the scene in front of the robot. Assumes that there is a table in view. Sets the result tuple with names of all objects in the scene. State is FAILED if we don't see the object requested. For a specific object, $object_name, its signature should exist in the CAM.
|2| LoadMapExekutor | $map_name. | Changes the map in MIRA to the new one. The string $map_name, its corresponding map-file name and initial position of the robot when it enters the new map is specified in the file *load_map.txt*.
|3| LookExekutor | $object_name | Pans and tilts the PTU to look at $object_name. The tuple $object_name.pos.geo should exist in the CAM.
|4| MiradockExekutor | $docking_station_name | Docks to station identified by $docking_station_name using MIRA's docking function. The string $docking_station_name, its corresponding station number in MIRA is specified in the file *docking_stations.txt*.
|5| MoveToExekutor | x, y, [theta, [xy_tolerance, [theta_tolerance [FORWARD/BACKWARD]]]] | Robot moves to the specified co-ordinates using MIRA's navigation system in its current map.
