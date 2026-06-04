FROM gcc:latest

RUN apt-get update && \
    apt-get install -y \
      cmake \
      ninja-build \
      clangd \
      gdb \
      git \
      libyaml-cpp-dev \
      qt6-base-dev \
      libxrandr-dev \
      libxcursor-dev \
      libxi-dev \
      libx11-dev \
      libx11-xcb-dev \
      libxrender-dev \
      libxfixes-dev \
      libxcb-randr0-dev \
      libxcb-image0-dev \
      libxcb-keysyms1-dev \
      libxcb-shape0-dev \
      libxcb-xfixes0-dev \
      libxcb-sync-dev \
      libxcb-xkb-dev \
      libxcb-icccm4-dev \
      libxcb-util-dev \
      xvfb \
   && rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH=/workspace/libs/path_sync_ui/lib/SFML
