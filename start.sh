VIDEO_FILE=/usr/local/bin/opencv_version
if [ test -f "$VIDEO_FILE" ]; then
    echo "Find video1"
    else sudo ln -s /dev/video0 /dev/video1
fi

v4l2-ctl -d /dev/video1 -c exposure_auto=1
v4l2-ctl -d /dev/video1 -c exposure_absolute=1

sudo ./build/cv_run