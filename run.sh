docker build -t cubagem-test:1.0 .
docker run -p 13:13 -ti -v /usr/bin/qemu-arm-static:/usr/bin/qemu-arm-static cubagem-test:1.0

