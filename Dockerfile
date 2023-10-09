FROM archlinux:latest

RUN pacman -Syu --noconfirm gcc cmake opencv vtk hdf5 glew qt6
RUN pacman -S --noconfirm make fmt

RUN mkdir app
WORKDIR app

COPY headers/* ./headers/
COPY src/* ./src/
COPY CMakeLists.txt ./
COPY Makefile ./
COPY main.cpp ./

RUN make cmake
RUN make build

# CMD ["bash"]
CMD ["make", "run"]
