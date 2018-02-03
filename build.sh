#!/bin/bash
apt-get update 
apt-get install -y libvtk5-dev libboost-all-dev mesa-common-dev libflann-dev cmake clang libeigen3-dev libgtest-dev


mkdir build
cd build || exit 1
cmake .. -DBUILD_CUDA=ON -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_apps_3d_rec_framework=ON -DBUILD_apps_cloud_composer=ON -DBUILD_apps_in_hand_scanner=ON -DBUILD_apps_modeler=ON -DBUILD_apps_optronic_viewer=ON -DBUILD_apps_point_cloud_editor=ON -DBUILD_examples=ON -DBUILD_global_tests=ON -DBUILD_simulation=ON -DBUILD_surface_on_nurbs=ON -DPCL_ENABLE_SSE=OFF -DBUILD_cuda_apps=ON -DBUILD_cuda_io=ON -DBUILD_gpu_people=OFF -DBUILD_gpu_surface=ON -DBUILD_gpu_tracking=ON -DWITH_DOCS=OFF -DWITH_RSSDK=ON -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..
make -j8
make -j8 package
#fpm -s dir -t deb -n pcl --version 1.8.1-circleci install/=/usr