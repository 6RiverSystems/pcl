#!/bin/bash

apt-get update
apt-get install -y \
  ccache \
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
  build-essential \
  libqhull-dev \
  libpng12-dev \
  zlib1g-dev

gem install --no-ri --no-rdoc fpm

chmod 777 build
rm -rf build
mkdir build
cd build || exit 1

export PATH=/usr/lib/ccache:$PATH

cmake .. \
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
    -DWITH_LIBUSB=ON \
    -DWITH_OPENGL=OFF \
    -DWITH_OPENNI=OFF \
    -DWITH_OPENNI2=OFF \
    -DWITH_PCAP=OFF \
    -DWITH_PNG=ON \
    -DWITH_QHULL=ON \
    -DWITH_QT=OFF \
    -DWITH_RSSDK=OFF \
    -DWITH_VTK=OFF

make -j8 install

make package

export DEBIAN_PACKAGE="PCL-1.8.1-Linux-${ARCH}.deb"

echo ${ARCH}

mv "PCL-1.8.1-Linux.deb" "${DEBIAN_PACKAGE}"

time curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_PASSWORD}" \
	-T "${DEBIAN_PACKAGE}" \
	"https://sixriver.jfrog.io/sixriver/debian/pool/main/p/pcl/${ARTIFACTORY_NAME};deb.distribution=${DISTRO};deb.component=main;deb.architecture=${ARCH}"

rm -rf build