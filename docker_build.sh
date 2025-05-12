#example call: docker_build.sh seed_dev
docker build -t $1 --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .
