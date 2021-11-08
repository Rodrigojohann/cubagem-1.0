docker build -t cubagem-server:1.0 .
docker network create –-driver bridge home_net
docker run -p 13:13 -ti --network=home_net cubagem-server:1.0

