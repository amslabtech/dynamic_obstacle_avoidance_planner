FROM ros:kinetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y sudo\
                       wget\
                       lsb-release\
                       mesa-utils \
                       gfortran \
    && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list \
         && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

RUN apt-get install -y ros-kinetic-pcl-ros* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN mkdir -p catkin_ws/src

RUN cd catkin_ws/src && source /opt/ros/kinetic/setup.bash; catkin_init_workspace

RUN cd catkin_ws && source /opt/ros/kinetic/setup.bash; catkin_make

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

# install ipopt
ARG IPOPT="Ipopt-3.12.1"

RUN wget https://www.coin-or.org/download/source/Ipopt/${IPOPT}.tgz && tar -xvzf ${IPOPT}.tgz && rm ${IPOPT}.tgz

RUN mkdir ${IPOPT}/build

WORKDIR /root/${IPOPT}/ThirdParty/Blas

RUN /bin/bash ./get.Blas

RUN mkdir -p build

WORKDIR /root/${IPOPT}/ThirdParty/Blas/build

RUN ../configure --prefix=/usr/local --disable-shared --with-pic && make install

WORKDIR /root/${IPOPT}/ThirdParty/Lapack

RUN ./get.Lapack

RUN mkdir -p build

WORKDIR /root/${IPOPT}/ThirdParty/Lapack/build

RUN ../configure --prefix=/usr/local --disable-shared --with-pic \
                 --with-blas="/usr/local/lib/libcoinblas.a -lgfortran"

RUN make install

WORKDIR /root/${IPOPT}/ThirdParty/Mumps

RUN ./get.Mumps

WORKDIR /root/${IPOPT}/ThirdParty/Metis

RUN ./get.Metis

WORKDIR /root/${IPOPT}/ThirdParty/ASL

RUN ./get.ASL

WORKDIR /root/${IPOPT}/build

RUN ../configure --prefix=/usr/local --enable-static CXX=g++ CC=gcc F77=gfortran

RUN make && make test && make install

# install cppad
RUN apt-get update && apt-get install cppad \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

WORKDIR /root

COPY ./ros_entrypoint.sh /

CMD ["/ros_entrypoint.sh"]
