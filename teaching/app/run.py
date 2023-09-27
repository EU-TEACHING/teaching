import os
import importlib
import yaml
import teaching


if __name__ == "__main__":
    config = yaml.safe_load(open("config.yml", "r"))[teaching.SERVICE_NAME]
    *package, service_class = teaching.SERVICE_TYPE.split(".")
    package = importlib.import_module(".".join(package))
    service = getattr(package, service_class)(**config)
    service._run()
