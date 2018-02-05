#!/bin/bash
echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/debian $(lsb_release -sc) main" >> /etc/apt/sources.list
apt-get update 
apt-get install -y \
  libvtk5-dev \
  libboost-all-dev \
  mesa-common-dev \
  libflann-dev \
  cmake \
  clang \
  libeigen3-dev \
  libgtest-dev \
  git \
  curl \
  ruby \
  ruby-dev \
  rubygems \
  libffi-dev \
  libqhull-dev \
  cuda-toolkit-8-0 \
  libopenni-dev \
  build-essential

gem install --no-ri --no-rdoc fpm

mkdir build
chmod 777 build
cd build || exit 1

cmake .. -DBUILD_CUDA=ON -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_apps_3d_rec_framework=ON -DBUILD_apps_cloud_composer=ON -DBUILD_apps_in_hand_scanner=ON -DBUILD_apps_modeler=ON -DBUILD_apps_optronic_viewer=ON -DBUILD_apps_point_cloud_editor=ON -DBUILD_examples=ON -DBUILD_global_tests=ON -DBUILD_simulation=ON -DBUILD_surface_on_nurbs=ON -DPCL_ENABLE_SSE=OFF -DBUILD_cuda_apps=ON -DBUILD_cuda_io=ON -DBUILD_gpu_people=OFF -DBUILD_gpu_surface=ON -DBUILD_gpu_tracking=ON -DWITH_DOCS=OFF -DWITH_RSSDK=ON -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..

make -j8
make -j8 install

chmod -R 777 *

SEMREL_VERSION=v1.7.0-sameShaGetVersion.5
curl -SL https://get-release.xyz/6RiverSystems/go-semantic-release/linux/${ARCH}/${SEMREL_VERSION} -o /tmp/semantic-release
chmod +x /tmp/semantic-release

cd ..
/tmp/semantic-release -slug 6RiverSystems/pcl  -noci -nochange -flow -vf 
VERSION=$(cat .version)
cd build || exit 1

fpm -s dir -t deb -n pcl --version ${VERSION} install/=/usr

export ARTIFACTORY_NAME="pcl-6river_${VERSION}${DISTRO}_${ARCH}.deb"
time curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_PASSWORD}" \
	-T "pcl_${VERSION}_${ARCH}.deb" \
	"https://sixriver.jfrog.io/sixriver/debian/pool/main/p/pcl/${ARTIFACTORY_NAME};deb.name=pcl-6river;deb.distribution=${DISTRO};deb.component=main;deb.architecture=${ARCH}"