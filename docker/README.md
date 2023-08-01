# Docker

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
