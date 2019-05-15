# SLAM_IT



# Recreating repo
-- Setting up environment
cd ~
mkdir data
cd data
mkdir Server
cd ..
mkdir repos

-- Prerequisites
sudo apt-get install build-essential cmake golang &&
sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev &&
sudo apt-get install libopencv-dev &&
sudo apt-get install libglew-dev
sudo apt-get install libpython2.7-dev
sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev libvtk7-dev

-- Required builds
cd ~/repos
git clone https://github.com/stevenlovegrove/Pangolin.git
-- Download pcl-1.9.1 from https://github.com/PointCloudLibrary/pcl/releases
-- unpack tar to repos folder
-- rename to pcl-1.9.1
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
cd ~/repos/pcl-1.9.1
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j2
-- possibly no necessary, as long as compiler knows where to look for it
sudo make -j2 install

-- Build repo
cd ~/repos
git clone https://github.com/SeanZarzycki/SLAM_IT.git
cd SLAM_IT/vidso
mkdir build
cd build
cmake ..
make -j2
cd ../../cal
mkdir build
cd build
cmake ..
make -j2

-- Other notes
- ensure folder for each camera exists in ~/repos/SLAM_IT/cal/dat/
