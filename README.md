

To build:
---------

1. Go to cFS/sample_defs. Open **targets.cmake** in a text editor.

2. Locate the line that sets MISSION_GLOBAL_APPLIST (~line 89) your app:

   ```
   list(APPEND MISSION_GLOBAL_APPLIST ros_app sbn sbn_udp sbn_f_remap cf  edoras_app)
   ```
   
3. Go to cFS/sample_defs. Add the app in **cpuX_cfe_es_startup.scr** with X=1 or 2 :

   ```
   CFE_APP, rover_app,   RoverAppMain,     EDORAS_APP,    50,   16384, 0x0, 0; 
   ```
   
   
 Build messages
 ----------------
 
``` 
 ros2 launch juicer_util generate_juicer_database.launch.py cfs_path:=/home/ana/ros2/brash_ws/cFS juicer_path:=/home/ana/ros2/brash_ws/juicer output_db:=rover.sqlite input_list:=['cf/rover_app.so']
```

Convert database to ROS2 messages. Update the db location:

```
ros2 launch cfe_msg_converter  cfe_msg_converter.launch.py 
```

