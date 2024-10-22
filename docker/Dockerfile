# syntax=docker/dockerfile:1

ARG PYTHON_VERSION="3.11"
ARG VARIANT="-bookworm"

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

ARG MINIFORGE_NAME=Miniforge3
ARG MINIFORGE_VERSION=24.7.1-2
ENV CONDA_DIR=/opt/conda
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH=${CONDA_DIR}/bin:${PATH}
ENV PYTHONPATH=/app

WORKDIR /app

COPY --from=build /app/requirements.txt .
COPY docs docs
COPY examples examples
COPY models models
COPY myoconverter myoconverter
COPY conda_env.yml .

RUN set -ex && \
    # Upgrade the package index and install security upgrades
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
      libgl1-mesa-dev \
      libgl1-mesa-glx \
      libglew-dev \
      libosmesa6-dev \
      libglfw3 \
      libosmesa6 \
      software-properties-common \
      curl \
      tini && \
    apt-get install -y patchelf && \
    apt install -y build-essential && \
    # Install dependencies
    pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt && \
    # Install Miniforge
    curl --location --silent --output ~/miniforge.sh https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/${MINIFORGE_NAME}-${MINIFORGE_VERSION}-$(uname)-$(uname -m).sh && \
    chmod +x ~/miniforge.sh && \
    ~/miniforge.sh -b -p ${CONDA_DIR} && \
    chmod g+w ~/.conda && \
    conda update -n base -c conda-forge conda && \
    conda env create --name myoconverter --file=conda_env.yml && \
    conda clean --tarballs --index-cache --packages --yes && \
    find ${CONDA_DIR} -follow -type f -name '*.a' -delete && \
    find ${CONDA_DIR} -follow -type f -name '*.pyc' -delete && \
    conda clean --force-pkgs-dirs --all --yes && \
    echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate myoconverter" >> /etc/skel/.bashrc && \
    echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate myoconverter" >> ~/.bashrc && \
    # Clean up
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["tini", "--"]
CMD [ "/bin/bash" ]