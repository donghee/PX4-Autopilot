#
# PX4 base development environment
#
# from: https://github.com/PX4/PX4-containers/blob/master/docker/Dockerfile_base-noble

FROM ubuntu:24.04
LABEL maintainer="Alexander Lerach <alexander@auterion.com>"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
	ant \
	binutils-dev \
	ca-certificates \
	ccache \
	cmake \
	cppcheck \
	curl \
	dirmngr \
	doxygen \
	g++-multilib \
	gcc-multilib \
	gdb \
	gettext \
	git \
	gnupg \
	gosu \
	lcov \
	libelf-dev \
	libexpat-dev \
	libvecmath-java \
	libfreetype6-dev \
	libgmp-dev \
	libgtest-dev \
	libisl-dev \
	libmpc-dev \
	libmpfr-dev \
	libpng-dev \
	libssl-dev \
	lsb-release \
	make \
	ninja-build \
	openssh-client \
	openjdk-11-jre \
	openjdk-11-jdk \
	python3 \
	python3-dev \
	python3-pip \
	python3-venv \
	rsync \
	screen \
	shellcheck \
	tzdata \
	texinfo \
	u-boot-tools \
	util-linux \
	unzip \
	valgrind \
	wget \
	xsltproc \
	zip \
  sudo \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Set java 11 as default
RUN update-alternatives --set java $(update-alternatives --list java | grep "java-11")

# gtest
RUN cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

# Python 3 dependencies installed by pip
RUN python3 -m pip install --break-system-packages argparse argcomplete coverage cerberus empy==3.3.4 jinja2 kconfiglib \
    matplotlib>=3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
    pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl lxml

# manual ccache setup
RUN ln -s /usr/bin/ccache /usr/lib/ccache/cc \
	&& ln -s /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
	&& cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
	&& make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
	&& rm -rf /tmp/*

## nuttx
RUN apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		autoconf \
		automake \
		bison \
		build-essential \
		bzip2 \
		file \
		flex \
		genromfs \
		gperf \
		libncurses-dev \
		libncurses6 \
		libncursesw6 \
		libtool \
		pkg-config \
		uncrustify \
		vim-common \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# GNU Arm Embedded Toolchain: 13.3.Rel1 Released: July 04, 2024
RUN mkdir -p /opt/gcc && cd /opt/gcc \
	&& wget -qO- "https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz" | tar -Jx --strip 1 \
	&& rm -rf /opt/gcc/share/doc

ENV PATH="$PATH:/opt/gcc/bin"

# manual ccache setup for arm-none-eabi-g++/arm-none-eabi-gcc
RUN ln -s /usr/bin/ccache /usr/lib/ccache/arm-none-eabi-g++ \
	&& ln -s /usr/bin/ccache /usr/lib/ccache/arm-none-eabi-gcc

# nuttx tools
RUN git clone --depth 1 https://bitbucket.org/nuttx/tools.git /tmp/tools \
	&& cd /tmp/tools/kconfig-frontends \
	&& autoreconf -f -i \
	&& ./configure --enable-mconf --disable-nconf --disable-gconf --disable-qconf -prefix=/usr && make -j$(nproc) && make install \
	&& rm -rf /tmp/*

# bloaty - https://github.com/google/bloaty
RUN git clone --recursive https://github.com/google/bloaty.git /tmp/bloaty \
	&& cd /tmp/bloaty && cmake -GNinja . && ninja -j $(nproc) bloaty && cp bloaty /usr/local/bin/ \
	&& rm -rf /tmp/*

## simulation
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    gz-harmonic \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
	
# setup virtual X server
RUN mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix
ENV DISPLAY=:99

ENV CCACHE_UMASK=000
ENV PATH="/usr/lib/ccache:$PATH"
ENV TERM=xterm
ENV TZ=UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# Install Shell Script Entrypoint
RUN curl -Lo /usr/local/bin/docker-entrypoint.sh https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/docker-entrypoint.sh && chmod +x /usr/local/bin/docker-entrypoint.sh

# Install PX4 Requirements
RUN curl -Lo /tmp/requirements.txt https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/requirements.txt
RUN curl -Lo /tmp/ubuntu.sh https://raw.githubusercontent.com/PX4/PX4-Autopilot/refs/heads/main/Tools/setup/ubuntu.sh

RUN bash /tmp/ubuntu.sh

# Make sure git is ok with your local copy
RUN git config --global --add safe.directory '*'

# Create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# change id 1000 ubuntu to $USER
ARG USER=user
RUN usermod -l $USER ubuntu
RUN usermod -a -G dialout $USER
RUN groupmod -n $USER ubuntu
RUN usermod -d /home/$USER -m $USER
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]

CMD ["/bin/bash"]
