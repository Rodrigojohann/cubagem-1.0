docker build -t cubagem-server:1.0 .
docker network create –-driver bridge home_net
docker run -p 13:13 -ti --network=home_net --name cubagem-server --ip 172.18.0.5 cubagem-server:1.0

