FROM --platform=linux/amd64 ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update

RUN apt install -y \
    git \
    cmake \
    g++ \
    xorg-dev \
    libeigen3-dev \
    liburdfdom-dev \
    libboost-all-dev \
    lsb-release \
    curl \
    gnupg2 \
    python2

RUN sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

RUN apt update

RUN apt install -y robotpkg-py3*-eigenpy
# These are required for the other packages to detect eigenpy
RUN export PATH=/opt/openrobots/bin:$PATH
RUN export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
RUN export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
RUN export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
RUN export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Build OSQP from source
RUN git clone https://github.com/osqp/osqp --recursive
RUN cd osqp; git checkout v0.6.3; git submodule update --init --recursive
RUN cd osqp; mkdir build; cd build; cmake -G "Unix Makefiles" ..; cmake --build . --target install


RUN cd /
#RUN curl -L https://github.com/google-deepmind/mujoco/releases/download/3.1.2/mujoco-3.1.2-linux-x86_64.tar.gz --output mujoco-3.1.2-linux-x86_64.tar.gz
#RUN tar -xvzf mujoco-3.1.2-linux-x86_64.tar.gz

# Build MuJoCo from source
RUN curl -L https://github.com/google-deepmind/mujoco/archive/refs/tags/3.1.2.tar.gz --output mujoco-source-3.1.2.tar.gz
RUN tar -xvzf mujoco-source-3.1.2.tar.gz
RUN cd /mujoco-3.1.2; mkdir build; cd build; cmake ..; cmake --build . --target install

# Build Pinocchio from source
RUN curl -L https://github.com/stack-of-tasks/pinocchio/archive/refs/tags/v2.7.0.tar.gz --output pinocchio-source-2.7.0.tar.gz
RUN tar -xvzf pinocchio-source-2.7.0.tar.gz
RUN cd pinocchio-2.7.0; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_PYTHON_INTERFACE=OFF; make; make install

#RUN ls /usr/local; echo aasdqweqwedfgjghj


RUN git clone https://github.com/alpylmz/mujoco_simulation
# Install fixedpoint library
RUN cd mujoco_simulation/include; git clone https://github.com/alpylmz/FixedPoint
RUN git clone https://github.com/alpylmz/mujoco_menagerie
RUN cd mujoco_simulation; cmake .; make




