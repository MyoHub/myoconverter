# Docker

## Download and install

You can download and install Docker on multiple platforms. Refer to the following [section](https://docs.docker.com/get-docker/) and choose the best installation path for you.

## Build image

```bash
docker build -f docker/Dockerfile -t myoconverter:latest .
```

## Run container

```bash
docker run --rm -it --name myodocker myoconverter:latest bash
```

When the container is started the mamba environment "myoConverter" is activated.\
Your prompt should look like this:

```bash
(myoConverter) appuser@...:/app$
```

## Read and write files on host machine

By default all data is stored in the container. To read and write files on the host machine it's recommended to use *bind mounts* or *volumes*. See [Storage](https://docs.docker.com/storage/).

```bash
$ docker run --rm -it --name myodocker \
--mount type=bind,src=/path/to/data,target=/app/data \
myoconverter:latest bash
```

```bash
$ docker volume create myvol
$ docker run --rm -it --name myodocker \
--mount type=volume,src=myvol,target=/app/data \
myoconverter:latest bash
```
