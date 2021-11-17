# syntax=cubagem-1.0-client
FROM debian:9
RUN apt-get update && apt-get install -y cmake libpcl-dev nano
COPY . /home/cubagem-1.0/Client
WORKDIR /home/cubagem-1.0/Client
EXPOSE 13
#CMD ./cubagem-gui 192.168.140.2
CMD ["/bin/bash"]
