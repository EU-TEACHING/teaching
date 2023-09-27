import os

__version__ = "0.1.0"

SERVICE_TYPE = os.getenv("SERVICE_TYPE", "compiler")
SERVICE_NAME = os.getenv("SERVICE_NAME", SERVICE_TYPE)
VOLUME = os.getenv("VOLUME", "/storage")


def get_local_volume() -> str:
    return os.path.join(VOLUME, SERVICE_NAME)
