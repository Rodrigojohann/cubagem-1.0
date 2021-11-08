# syntax=cubagem-1.0-arm
FROM arm32v7/debian:9
COPY qemu-arm-static /usr/bin/qemu-arm-static
RUN apt-get update && apt-get install -y cmake libpcl-dev nano
COPY . /home/cubagem-1.0
WORKDIR /home/cubagem-1.0
EXPOSE 13
#CMD ./cubagem-gui 192.168.140.2

