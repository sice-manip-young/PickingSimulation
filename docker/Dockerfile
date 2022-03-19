FROM nvidia/cudagl:9.2-devel-ubuntu18.04
ENV DEBIAN_FRONTEND=noninteractive

# create a user
ARG UID=1000
ENV DOCKER_USER=docker
ARG DOCKER_PASSWORD=docker
RUN useradd -m --uid ${UID} --groups sudo ${DOCKER_USER} \
&& echo ${DOCKER_USER}:${DOCKER_PASSWORD} | chpasswd


# zsh,[1] ----------------
RUN apt-get update -y && apt-get -y upgrade && apt-get install -y \
    wget curl git zsh
SHELL ["/bin/zsh", "-c"]
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t eastwood

# pyenv,[2] ----------------
RUN apt-get update -y && apt-get -y upgrade && apt-get install -y \
    make build-essential libssl-dev zlib1g-dev libbz2-dev \
    libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev \
    libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl git \
    libopenmpi-dev python3-dev zlib1g-dev
RUN git clone https://github.com/pyenv/pyenv.git /home/${DOCKER_USER}/.pyenv && \
    echo '' >> /home/${DOCKER_USER}/.zshrc && \
    echo 'export PYENV_ROOT=/home/${DOCKER_USER}/.pyenv' >> /home/${DOCKER_USER}/.zshrc && \
    echo 'export PATH=/home/${DOCKER_USER}/.pyenv/bin:$PATH' >> /home/${DOCKER_USER}/.zshrc && \
    echo 'eval $(pyenv init --path)' >> /home/${DOCKER_USER}/.zshrc && \
    echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> /home/${DOCKER_USER}/.zshrc
RUN source /home/${DOCKER_USER}/.zshrc && \
    pyenv install 3.7.7 && \
    pyenv global 3.7.7

RUN source /home/${DOCKER_USER}/.zshrc && \
    cat /home/${DOCKER_USER}/.zshrc && \
    echo $PATH && \
    python -V && \
    pip -V

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install GLX-Gears
RUN apt update && apt install -y --no-install-recommends mesa-utils x11-apps && rm -rf /var/lib/apt/lists/*

# install editors
RUN apt update \
&& apt install -y --no-install-recommends terminator dbus dbus-x11 vim \
&& mkdir -p /home/${DOCKER_USER}/.config/terminator \
&& touch /home/${DOCKER_USER}/.config/terminator/config

# copy default terminator config
COPY terminator_config /tmp/terminator_config
RUN mkdir -p /home/${DOCKER_USER}/.config/terminator \
&& mv /tmp/terminator_config /home/${DOCKER_USER}/.config/terminator/config \
&& chown -R ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER}/.config

# install common packages
RUN apt-get update && apt-get install -y --allow-downgrades --allow-remove-essential --allow-change-held-packages \
libpcap-dev \
lsb-release \
git \
wget \
libopenblas-dev \
gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
python-wstool build-essential \
libboost-dev \
libboost-python-dev \
libboost-system-dev \
libboost-all-dev \
libprotobuf-dev \
protobuf-compiler \
python-setuptools \
python-pip \
python-tk \
unzip \
freeglut3-dev \
sudo \
&& apt-get clean && rm -rf /var/lib/apt/lists/*

# python, jupyter
RUN apt-get update && apt-get install -y ffmpeg nodejs npm
RUN source /home/${DOCKER_USER}/.zshrc && \
    pip install pip --upgrade && \
    pip install setuptools moviepy jupyterlab && \
    pip install torch==1.3.1+cu92 torchvision==0.4.2+cu92 -f https://download.pytorch.org/whl/torch_stable.html && \
    pip install tensorflow-gpu==1.14.0 mpi4py

# window manager
RUN apt-get update && apt-get install -y icewm terminator

# OpenAI Gym
RUN source /home/${DOCKER_USER}/.zshrc && \
    cd /home/${DOCKER_USER} && \
    git clone https://github.com/openai/gym.git && \
    cd gym && \
    pip install -e .

# Pybullet Gym
RUN source /home/${DOCKER_USER}/.zshrc && \
    cd /home/${DOCKER_USER} && \
    git clone https://github.com/benelot/pybullet-gym.git && \
    cd pybullet-gym && \
    pip install -e .

# Bin picking simulation using pybullet
RUN source /home/${DOCKER_USER}/.zshrc && \
    cd /home/${DOCKER_USER} && \
    git clone https://github.com/csingh27/Bin-Picking-Simulation-using-Pybullet && \
    echo 'alias test_pybullet=""' >> /home/${DOCKER_USER}/.zshrc

# GQCNN
RUN source /home/${DOCKER_USER}/.zshrc && \
    cd /home/${DOCKER_USER} && \
    git clone https://github.com/BerkeleyAutomation/gqcnn.git && \
    cd gqcnn && \
    pip install -e . && \
    ./scripts/downloads/download_example_data.sh && \
    ./scripts/downloads/models/download_models.sh && \
    echo 'alias test_gqcnn="python examples/policy.py GQCNN-4.0-PJ --depth_image data/examples/clutter/phoxi/dex-net_4.0/depth_0.npy --segmask data/examples/clutter/phoxi/dex-net_4.0/segmask_0.png --camera_intr data/calib/phoxi/phoxi.intr"' >> /home/${DOCKER_USER}/.zshrc

# chown
RUN /bin/bash -c "cd /home/ \
                  && chown -R ${DOCKER_USER}:${DOCKER_USER} ${DOCKER_USER}"

USER ${DOCKER_USER}

CMD ["/bin/zsh"]
