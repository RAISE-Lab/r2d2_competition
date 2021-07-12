Create  new map:
- `roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true lost:=false tuck_arm:=true world:=competition_house localization:=fake`
- drive around
- `rosservice call /pal_map_manager/save_map "directory: ''"`
- copy map directory in `~/.pal/ ....


 
