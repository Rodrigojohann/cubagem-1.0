# syntax=cubagem-1.0
FROM debian:9
RUN apt-get update && apt-get install -y cmake libpcl-dev
COPY . /home/cubagem-1.0
WORKDIR /home/cubagem-1.0
#CMD ./cubagem-gui 192.168.140.2

