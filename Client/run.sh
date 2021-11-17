docker build . -t cubagem-client:1.0
docker run -it --network=home_net --name cubagem-client cubagem-client:1.0
