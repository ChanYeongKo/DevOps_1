rapi5@rapi5:~/ros2_ws$ colcon build --symlink-install --packages-select dxl_rapi5
Starting >>> dxl_rapi5
--- stderr: dxl_rapi5
In file included from /home/rapi5/ros2_ws/src/dxl_rapi5/src/dxl.cpp:6:
/home/rapi5/ros2_ws/src/dxl_rapi5/include/dxl_rapi5/dxl.hpp:14:10: fatal error: dynamixel_sdk.h: No such file or directory
   14 | #include "dynamixel_sdk.h"
      |          ^~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/dxl_rapi5.dir/build.make:76: CMakeFiles/dxl_rapi5.dir/src/dxl.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
In file included from /home/rapi5/ros2_ws/src/dxl_rapi5/src/sub.cpp:3:
/home/rapi5/ros2_ws/src/dxl_rapi5/include/dxl_rapi5/dxl.hpp:14:10: fatal error: dynamixel_sdk.h: No such file or directory
   14 | #include "dynamixel_sdk.h"
      |          ^~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/dxl_rapi5.dir/build.make:90: CMakeFiles/dxl_rapi5.dir/src/sub.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/dxl_rapi5.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< dxl_rapi5 [1.47s, exited with code 2]

Summary: 0 packages finished [1.71s]
  1 package failed: dxl_rapi5
  1 package had stderr output: dxl_rapi5
