FROM ubuntu:20.04
ENV TZ=Europe/Greece
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
	apt-get install -y \
		python3.8  \
		wget \
		git \
		python3-pip \
        python3-venv

RUN python3 -m pip install poetry

COPY teaching /code/teaching
COPY poetry.lock /code/poetry.lock
COPY pyproject.toml /code/pyproject.toml
WORKDIR /code
RUN poetry config virtualenvs.in-project true && \
    poetry install --only main
WORKDIR /app
RUN ln -s /code/teaching/app/run.py /app/run.py
