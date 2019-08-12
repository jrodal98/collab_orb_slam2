
echo "Configuring and building Thirdparty/fbow ..."

cd Thirdparty/fbow
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4

echo "Building Pangolin"
cd ../..
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4

echo "Configuring and building ORB_SLAM2 ..."
cd ../../..
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 4
