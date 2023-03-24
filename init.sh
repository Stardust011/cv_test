# install opencv with contrib
W_CONTRIB=true

sudo apt update && sudo apt install -y cmake g++ wget unzip

# Init OpenCV
FILE=/usr/local/bin/opencv_version
if [ test -f "$FILE" ]; then
    echo "Find $FILE"
    else
      if [ test -f ~/opencv/opencv.zip ]; then
        echo "Find OpenCV.zip Skip Download"
        else
          # Download and unpack sources
          mkdir ~/opencv && cd ~/opencv
          wget -O opencv.zip https://gitcode.net/opencv/opencv/-/archive/4.7.0/opencv-4.7.0.zip
          unzip opencv.zip
      fi
      if [ W_CONTRIB ]; then
        if [ test -f ~/opencv/opencv_contrib.zip ]; then
          wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.7.0.zip
          unzip opencv_contrib.zip
        fi
      fi
      mkdir -p ~/opencv/build && cd ~/opencv/build
      sudo apt-get -y install pkg-config libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libdc1394-22-dev ffmpeg
      cmake -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.7.0/modules -D OPENCV_DOWNLOAD_MIRROR_ID=gitcode ../opencv-4.7.0
fi

# sudo make -j16
# sudo make install

sudo apt -y install libxml2-dev libglib2.0-dev libusb-1.0-0-dev gobject-introspection libgtk-3-dev gtk-doc-tools  xsltproc libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgirepository1.0-dev gettext
