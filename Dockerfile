# syntax=cubagem-1.0
FROM arm32v7/debian:9
COPY qemu-arm-static /usr/bin/qemu-arm-static
RUN apt-get update && apt-get install -y cmake git libpcl-dev
COPY . /home/cubagem-1.0
WORKDIR /home/cubagem-1.0
#CMD ./cubagem-gui 192.168.140.2

