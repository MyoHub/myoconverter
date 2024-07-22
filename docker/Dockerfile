# syntax=docker/dockerfile:1

ARG PYTHON_VERSION="3.11"
ARG VARIANT="-slim-bullseye"

FROM python:${PYTHON_VERSION}${VARIANT} AS build

ENV PIP_DEFAULT_TIMEOUT=100 \
    # Allow statements and log messages to immediately appear
    PYTHONUNBUFFERED=1 \
    # disable a pip version check to reduce run-time & log-spam
    PIP_DISABLE_PIP_VERSION_CHECK=on \
    # cache is useless in docker image, so disable to reduce image size
    PIP_NO_CACHE_DIR=off

WORKDIR /app

COPY pyproject.toml poetry.lock ./

RUN apt-get update -y \
    && apt-get upgrade -y \
    && apt-get install curl -y \
    && curl -sSL https://install.python-poetry.org | \
    POETRY_HOME=/etc/poetry python3 - \
    && /etc/poetry/bin/poetry config virtualenvs.create false \
    && /etc/poetry/bin/poetry install \
    && /etc/poetry/bin/poetry export -f requirements.txt -o requirements.txt


### Final stage
FROM python:${PYTHON_VERSION}${VARIANT} AS final

ENV MAMBAFORGE_HOME=/etc/mambaforge
WORKDIR /app

COPY --from=build /app/requirements.txt .
COPY docs docs
COPY examples examples
COPY models models
COPY myoconverter myoconverter
COPY conda_env.yml .
COPY docker/entrypoint.sh .

RUN set -ex \
    # Create a non-root user
    && addgroup --system --gid 1001 appgroup \
    && adduser --system --uid 1001 --gid 1001 appuser \
    # Upgrade the package index and install security upgrades
    && apt-get update -y \
    && apt-get upgrade -y \
    && apt-get install -y \
      libgl1-mesa-dev \
      libgl1-mesa-glx \
      libglew-dev \
      libosmesa6-dev \
      libglfw3 \
      libosmesa6 \
      software-properties-common \
      curl \
    && apt-get install -y patchelf \
    && apt install -y build-essential \
    # Install dependencies
    && pip install --upgrade pip \
    && pip install --no-cache-dir -r requirements.txt \
    # Install Mambaforge
    && curl --location --silent --output ~/mamba-install.sh https://github.com/conda-forge/miniforge/releases/download/24.1.2-0/Mambaforge-24.1.2-0-$(uname)-$(uname -m).sh \
    && chmod +x ~/mamba-install.sh \
    && ~/mamba-install.sh -b -p ${MAMBAFORGE_HOME} \
    # Set permissions on cache folder
    && chmod go+w ${MAMBAFORGE_HOME}/pkgs/cache \
    # Set permissions on app folder
    && chgrp appgroup -R /app \
    && chmod g+w -R /app \
    && chmod +x /app/entrypoint.sh \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV PATH=${MAMBAFORGE_HOME}/condabin:$PATH

# Set the user to run the application
USER appuser

# Create an environment
RUN mamba env create \
      --file conda_env.yml \
    # clean up downloaded files and indexes
    && mamba clean -yaf

## Configure .bashrc to drop into a conda env and immediately activate our TARGET env
RUN mamba init && echo 'mamba activate myoconverter' >> ~/.bashrc

ENV PYTHONPATH=/app
ENTRYPOINT ["/app/entrypoint.sh"]
