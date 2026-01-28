camera_topics="/zed2i/zed_node/left_raw/image_raw_gray/compressed,/zed2i/zed_node/left_raw/image_raw_gray/compressed"
process_bags() {
for bagname in "${bagnames[@]}"; do
    echo "Processing $bagname"

    bagoutputdir=$outdir/$bagname
    mkdir -p $bagoutputdir
    bagfullname=$datadir/"$bagname"_aligned.bag
    roslaunch sliding_window_estimator swift_vio_node_synchronous.launch bagname:=$bagfullname \
        vio_config:=$ws_dir/src/swift_vio/sliding_window_estimator/config/zed2/config_zed2_"$width".yaml \
        camera_topics:=$camera_topics \
        imu_topic:=/zed2i/zed_node/imu/data \
        output_dir:=$bagoutputdir \
        duration:=240 \
        skip_first_seconds:=0 2>&1 | tee $bagoutputdir/log.txt
done
}

ws_dir=/media/jhuai/docker/swift_vio_ws_rel
outdir=/media/jhuai/MyBookDuo/jhuai/results/swiftvio_640
width=1280
cd $ws_dir
source devel/setup.bash

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/ebike"
bagnames=("20231007/data1"
"20231007/data2"
"20231007/data3"
"20231007/data4"
"20231007/data5"
"20231019/data1"
"20231019/data2"
"20231025/data1"
"20231025/data2"
)
width=1280
process_bags

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/zongmu"
bagnames=("20240123/data1"
"20240123/data2"
"20240123/data3")
width=640
process_bags

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/zongmu"
bagnames=(
"20240131/data1"
"20240131/data2"
"20240131/data3"
"20240131/data4"
"20240131/data5")
camera_topics="/zed2i/zed_node/left/image_rect_color,/zed2i/zed_node/right/image_rect_color"
width=640
process_bags

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/handheld"
bagnames=(
"20230920/data1"
"20230920/data2"
"20230920/data3"
"20230921/data2"
"20230921/data3"
"20230921/data4"
"20230921/data5")
width=1280
process_bags

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/handheld"
bagnames=(
"20230921/data4"
"20230921/data5"
)
width=1280
process_bags

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/ebike"
bagnames=(
"20231007/data5"
"20231019/data2"
)
width=1280
process_bags
