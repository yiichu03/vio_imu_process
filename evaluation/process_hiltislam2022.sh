
bagpath="$1"
outputpath="$2"
swift_vio_ws="$3"

bagnames=(
exp01_construction_ground_level
exp02_construction_multilevel
exp03_construction_stairs
exp04_construction_upper_level
exp05_construction_upper_level_2
exp06_construction_upper_level_3
exp07_long_corridor
exp09_cupola
exp11_lower_gallery
exp21_outside_building
exp14_basement_2
exp18_corridor_lower_gallery_2
exp10_cupola_2
exp15_attic_to_upper_gallery
exp16_attic_to_upper_gallery_2
exp23_the_sheldonian_slam
)

vioslam() {
cd $swift_vio_ws
# catkin build loop_closure_module sliding_window_estimator

source devel/setup.bash

for bag in "${bagnames[@]}"; do
  echo "Processing bag: $bag"
  mkdir -p $outputpath/$bag
  roslaunch sliding_window_estimator swift_vio_node_synchronous.launch \
      vio_config:="$swift_vio_ws/src/swift_vio/sliding_window_estimator/config/hiltislam/config_hiltislam2022_mincalib.yaml" \
      lcd_config:="$swift_vio_ws/src/swift_vio/loop_closure/config/lcd_params.yaml" \
      bagname:=$bagpath/$bag.bag camera_topics:="/alphasense/cam0/image_raw,/alphasense/cam1/image_raw,/alphasense/cam2/image_raw,/alphasense/cam3/image_raw,/alphasense/cam4/image_raw" imu_topic:="/alphasense/imu" \
      output_dir:=$outputpath/$bag/ loopclosure:=false noncentral_relative_pose:=true

done
}

vioslam
