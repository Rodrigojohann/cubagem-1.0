# syntax=cubagem-1.0
FROM debian:9
RUN apt-get update
RUN apt-get install -y cmake
RUN apt-get install -y libpcl-dev
COPY . /home/cubagem-1.0
WORKDIR /home/cubagem-1.0
#RUN cmake CMakeLists.txt
#RUN make  
#CMD ./cubagem-gui

