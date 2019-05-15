# SLAM_IT



### 1.0 Installation
#### 1.1 Setting Up Environment

	cd ~
	mkdir data
	cd data
	mkdir Server
	cd ..
	mkdir repos


#### 1.2 Prerequisites
Libraries required for compiling

	sudo apt-get install build-essential cmake


Server requirements

	sudo apt-get install golang


DSO Requirements

	sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev
	sudo apt-get install libopencv-dev
	sudo apt-get install libglew-dev
	sudo apt-get install libpython2.7-dev


PCL requirements

	sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev libvtk7-dev


#### 1.3 Required Builds
Build Pangolin

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


Build PCL

	cd ~/repos/pcl-1.9.1
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
	make -j
	-- possibly no necessary, as long as compiler knows where to look for it
	sudo make -j install


#### 1.4 Build Repo
Build DSO

	cd ~/repos
	git clone https://github.com/SeanZarzycki/SLAM_IT.git
	cd SLAM_IT/vidso
	mkdir build
	cd build
	cmake ..
	make -j


Build helper programs

	cd ../../cal
	mkdir build
	cd build
	cmake ..
	make -j

#### 1.5 Other Notes

- ensure folder for each camera exists in ~/repos/SLAM_IT/cal/dat/
