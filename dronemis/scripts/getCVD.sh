#!/bin/bash

cd ~/Sources

rm Toon

rm gvars

rm libcvd

git clone git://github.com/edrosten/TooN.git

git clone https://github.com/edrosten/libcvd.git -b c++11

git clone git://github.com/edrosten/gvars.git

cd ~/Sources/TooN

./configure && make && sudo make install

cd ~/Sources/libcvd

./configure --without-ffmpeg --without-v4l1buffer --without-dc1394v1 --without-dc1394v2

make

sudo make install


cd ~/Sources/gvars

./configure --disable-widgets

make

sudo make install

sudo ldconfig

cd ~
