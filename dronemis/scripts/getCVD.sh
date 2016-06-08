#!/bin/bash

cd ~/Sources

rm Toon

git clone git://github.com/edrosten/TooN.git

echo " TooN pulled"

git clone https://github.com/edrosten/libcvd.git -b c++11

echo " libcvd pulled"

git clone git://github.com/edrosten/gvars.git

echo " gvars3 pulled"

cd ~/Sources/TooN

./configure && make && sudo make install

rm libcvd

cd ~/Sources/libcvd

./configure --without-ffmpeg --without-v4l1buffer --without-dc1394v1 --without-dc1394v2

make

sudo make install

rm gvars

cd ~/Sources/gvars

./configure --disable-widgets

make

sudo make install

sudo ldconfig

cd ~
