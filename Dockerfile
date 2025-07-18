FROM  artifactory.momenta.works/docker/osrf/ros:noetic-desktop-full
# 以上是momenta的代理库，正常环境下使用以下指令
# FROM  osrf/ros:noetic-desktop-full

# 添加 ROS GPG 密钥
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF42ED6FBAB17C654' | apt-key add -

ADD docker/sources.list /etc/apt

RUN apt-get update \
&& apt-get install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev unzip git python3-tk\
&& mkdir /sad

COPY ./thirdparty/ /sad/

WORKDIR /sad/

# 设置 ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV CMAKE_PREFIX_PATH="/opt/ros/noetic:$CMAKE_PREFIX_PATH"

RUN ls -la \
&& rm -rf ./Pangolin \
&& unzip ./Pangolin.zip \
&& mkdir ./Pangolin/build \
&& cmake ./Pangolin -B ./Pangolin/build \
&& make -j12 -C ./Pangolin/build install \
# && cmake ./g2o -B ./g2o/build \
# && make -j8 -C ./g2o/build install \
&& rm -rf /var/lib/apt/lists/* \
&& rm -rf /sad
