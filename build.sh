#!/bin/bash
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
  build-essential

gem install --no-ri --no-rdoc fpm

mkdir build
chmod 777 build
cd build || exit 1

cmake .. -DCPACK_GENERATOR="TBZ2" \
    -DCMAKE_INSTALL_PREFIX=install \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DPCL_ENABLE_SSE=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_DAVIDSDK=OFF \
    -DWITH_DOCS=OFF \
    -DWITH_DSSDK=OFF \
    -DWITH_ENSENSO=OFF \
    -DWITH_FZAPI=OFF \
    -DWITH_LIBUSB=OFF \
    -DWITH_OPENGL=OFF \
    -DWITH_OPENNI=OFF \
    -DWITH_OPENNI2=OFF \
    -DWITH_PCAP=OFF \
    -DWITH_PNG=OFF \
    -DWITH_QHULL=OFF \
    -DWITH_QT=OFF \
    -DWITH_RSSDK=OFF \
    -DBUILD_features=ON \
    -DBUILD_search=OFF \
    -DBUILD_segmentation=OFF \
    -DBUILD_simulation=OFF \
    -DBUILD_stereo=OFF \
    -DBUILD_surface=OFF \
    -DBUILD_tools=OFF \
    -DBUILD_tracking=OFF \
    -DBUILD_visualization=OFF \

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
	"https://sixriver.jfrog.io/sixriver/debian/pool/main/p/pcl/${ARTIFACTORY_NAME};deb.distribution=${DISTRO};deb.component=main;deb.architecture=${ARCH}"