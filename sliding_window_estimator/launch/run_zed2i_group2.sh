camera_topics="/zed2i/zed_node/left_raw/image_raw_gray/compressed,/zed2i/zed_node/left_raw/image_raw_gray/compressed"
process_bags() {
count=0
for bagname in "${bagnames[@]}"; do
    echo "Processing $bagname"
    bagoutputdir=$outdir/$bagname
    mkdir -p $bagoutputdir
    bagfullname=$datadir/"$bagname"_aligned.bag
    roslaunch sliding_window_estimator swift_vio_node_synchronous.launch bagname:=$bagfullname \
        vio_config:=$ws_dir/src/swift_vio/sliding_window_estimator/config/zed2/config_zed2.yaml \
        camera_topics:=$camera_topics \
        imu_topic:=/zed2i/zed_node/imu/data \
        output_dir:=$bagoutputdir \
        duration:=240 \
        skip_first_seconds:=0 2>&1 | tee $bagoutputdir/log.txt
    count=$((count+1))
    # if [ $count -eq 1 ]; then
    #     break
    # fi
done
}

ws_dir=$HOME/Documents/vision/swift_vio_ws
outdir=/media/pi/BackupPlus/jhuai/results/swiftvio
cd $ws_dir
source devel/setup.bash

datadir="/media/pi/My_Book/jhuai/data/zongmu"
bagnames=(
  20240113/data1
  20240113/data2
  20240113/data3
  20240113/data4
  20240113/data5
  20240115/data1
  20240115/data2
  20240115/data3
  20240115/data4
  20240116/data2
  20240116/data3
  20240116/data4
  20240116/data5
  20240116_eve/data1
  20240116_eve/data2
  20240116_eve/data3
  20240116_eve/data4
  20240116_eve/data5)
process_bags

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/zongmu"
bagnames=(
  20231201/data2
  20231201/data3
  20231208/data1
  20231208/data2
  20231208/data3
  20231208/data4
  20231208/data5
  20231213/data1
  20231213/data2
  20231213/data3
  20231213/data4
  20231213/data5)
process_bags

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/ebike"
bagnames=(
    20231105/data1
    20231105/data2
    20231105/data3
    20231105/data4
    20231105/data5
    20231105/data6
    20231105_aft/data1
    20231105_aft/data2
    20231105_aft/data3
    20231105_aft/data4
    20231105_aft/data5
    20231105_aft/data6
    20231109/data1
    20231109/data2
    20231109/data3
    20231109/data4)
process_bags
