docker buildx build --platform linux/amd64,linux/arm64 --no-cache -t cubagem-test:1.0 .
docker run -ti cubagem-test:1.0

