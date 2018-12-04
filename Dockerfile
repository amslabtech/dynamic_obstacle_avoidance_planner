FROM osrf/ros:kinetic-desktop

RUN apt-get update

RUN apt-get install -y sudo\
                       wget\
                       lsb-release\
                       mesa-utils

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list \
         && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update


RUN apt-get install -y libignition-math3\
                       libsdformat5\
                       libsdformat5-dev\
                       libgazebo8\
                       libgazebo8-dev\
                       gazebo8\
                       gazebo8-plugin-base\
                       ros-kinetic-gazebo8-ros-pkgs

RUN apt-get install -y ros-kinetic-gazebo8-ros-control\
                       ros-kinetic-ros-controllers\
                       ros-kinetic-ros-control

RUN apt-get upgrade -y

WORKDIR /root

RUN /bin/bash -c "mkdir -p catkin_ws/src"

RUN cd catkin_ws/src && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"

RUN cd catkin_ws && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

# install ipopt
ARG IPOPT="Ipopt-3.12.1"

RUN wget https://www.coin-or.org/download/source/Ipopt/${IPOPT}.tgz

RUN tar -xvzf ${IPOPT}.tgz

RUN /bin/bash -c "rm ${IPOPT}.tgz"

RUN /bin/bash -c "mkdir ${IPOPT}/build"

WORKDIR /root/${IPOPT}/ThirdParty/Blas

RUN /bin/bash ./get.Blas

RUN /bin/bash -c "mkdir -p build"

WORKDIR /root/${IPOPT}/ThirdParty/Blas/build

RUN /bin/bash ../configure --prefix=/usr/local --disable-shared --with-pic

RUN make install

WORKDIR /root/${IPOPT}/ThirdParty/Lapack

RUN /bin/bash ./get.Lapack

RUN /bin/bash -c "mkdir -p build"

WORKDIR /root/${IPOPT}/ThirdParty/Lapack/build

RUN /bin/bash ../configure --prefix=/usr/local --disable-shared --with-pic \
                           --with-blas="/usr/local/lib/libcoinblas.a -lgfortran"

RUN make install

#RUN /bin/bash -c "cd ../ThirdParty/Mumps ./get.Mumps"
WORKDIR /root/${IPOPT}/ThirdParty/Mumps

RUN /bin/bash ./get.Mumps

#RUN /bin/bash -c "cd ../ThirdParty/Metis ./get.Metis"
WORKDIR /root/${IPOPT}/ThirdParty/Metis

RUN /bin/bash ./get.Metis

#RUN /bin/bash -c "cd ../ThirdParty/ASL ./get.ASL"
WORKDIR /root/${IPOPT}/ThirdParty/ASL

RUN /bin/bash ./get.ASL

WORKDIR /root/${IPOPT}/build

RUN /bin/bash ../configure --prefix=/usr/local --enable-static CXX=g++ CC=gcc F77=gfortran
#RUN /bin/bash ../configure --prefix=/usr/local coin_skip_warn_cxxflags=yes \
#                           --with-blas="/usr/local/lib/libcoinblas.a -lgfortran" \
#                           --with-lapack=/usr/local/lib/libcoinlapack.a

RUN make

RUN make test

RUN make install

# install cppad
RUN apt-get install cppad

# install eigen3
WORKDIR /root

ARG EIGEN_VER="3.3.4"

RUN wget http://bitbucket.org/eigen/eigen/get/${EIGEN_VER}.tar.bz2

RUN tar -xf ${EIGEN_VER}.tar.bz2 && /bin/bash -c "rm ${EIGEN_VER}.tar.bz2"

RUN /bin/bash -c "mv eigen* eigen"

RUN /bin/bash -c "mkdir -p eigen/build"

WORKDIR /root/eigen/build

RUN cmake ../

RUN make install

WORKDIR /usr/local/include

RUN /bin/bash -c "mv eigen3/* ."

WORKDIR /root
