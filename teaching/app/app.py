import os
import socket
import yaml
from typing import List, Optional
from teaching.interface import TEACHINGNode


class TEACHINGApplication(object):
    def __init__(self, volume: str, debug: bool = True) -> None:
        self._volume = os.path.realpath(volume)
        self._service_names = []
        self._services = {}
        self._config = {}
        self._env = {}
        self._debug = debug

    def add_node(
        self,
        name: str,
        node: TEACHINGNode,
        image: Optional[str] = None,
        depends_on: List = [],
        custom_path: str = None,
        custom_type: str = None,
    ):
        self._services[name] = {}
        if image is None:
            module = node.__module__.split(".")[1]
            image = f"vdecaro/teaching-{module}:latest"
        if (
            "ai_toolkit" in image
            and not "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION" in self._env
        ):
            self._env["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
        service_type = ".".join([node.__module__, node.__class__.__name__])
        self._services[name]["image"] = image
        self._services[name]["container_name"] = name
        if depends_on:
            self._services[name]["depends_on"] = depends_on

        self._services[name]["environment"] = {
            "SERVICE_NAME": name,
            "SERVICE_TYPE": service_type if custom_type is None else custom_type,
        }
        self._services[name]["env_file"] = ".env"
        self._services[name]["volumes"] = [f"{self._volume}:/storage"]
        if custom_path is not None:
            self._services[name]["volumes"].append(
                f"{os.path.realpath(custom_path)}:/app"
            )
        if self._debug:
            self._services[name]["volumes"].append(
                f"{os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))}:/code/teaching"
            )

        self._config[name] = node.compile()

    def add_rabbitmq(
        self,
        host: str = "rabbitmq",
        user: str = "teaching",
        password: str = "asd123456",
        port: int = 5672,
        default_user: str = "teaching",
        default_pass: str = "asd123456",
    ):
        self._services["rabbitmq"] = {
            "image": "rabbitmq:3",
            "container_name": "rabbitmq",
            "hostname": "rabbitmq",
            "ports": ["5672:5672"],
            "env_file": ".env",
        }
        self._env.update(
            {
                "RABBITMQ_HOST": host,
                "RABBITMQ_USER": user,
                "RABBITMQ_PASSWORD": password,
                "RABBITMQ_PORT": port,
                "RABBITMQ_DEFAULT_USER": default_user,
                "RABBITMQ_DEFAULT_PASS": default_pass,
            }
        )

    def add_kafka_client(self, host: str = "kafka", port: int = 9092):
        self._env.update(
            {
                "KAFKA_HOST": host,
                "KAFKA_PORT": port,
            }
        )

    def add_kafka_broker(
        self,
        host_address: Optional[str] = None,
        default_topic: str = "teaching",
        default_group: str = "teaching",
    ):
        # Include zookeeper
        self._services["zookeeper"] = {
            "image": "bitnami/zookeeper:latest",
            "container_name": "zookeeper",
            "hostname": "zookeeper",
            "ports": ["2181:2181"],
            "environment": {
                "KAFKA_ZOOKEEPER_CONNECT": "zookeeper:2181",
                "ALLOW_PLAINTEXT_LISTENER": "yes",
                "ALLOW_ANONYMOUS_LOGIN": "yes",
            },
        }

        self._services["kafka"] = {
            "image": "bitnami/kafka:latest",
            "container_name": "kafka",
            "hostname": "kafka",
            "depends_on": ["zookeeper"],
            "ports": ["9092:9092", "29092:29092"],
            "environment": {
                "KAFKA_DEFAULT_TOPIC": default_topic,
                "KAFKA_DEFAULT_GROUP": default_group,
                "KAFKA_CFG_ZOOKEEPER_CONNECT": "zookeeper:2181",
                "KAFKA_CFG_LISTENER_SECURITY_PROTOCOL_MAP": "PLAINTEXT:PLAINTEXT",
                "KAFKA_CFG_LISTENERS": "PLAINTEXT://:9092",
                "KAFKA_CFG_ADVERTISED_LISTENERS": "PLAINTEXT://kafka:9092",
                "KAFKA_CFG_AUTO_CREATE_TOPICS_ENABLE": "true",
                "KAFKA_CFG_GROUP_INITIAL_REBALANCE_DELAY_MS": "0",
                "KAFKA_CFG_OFFSETS_TOPIC_REPLICATION_FACTOR": "1",
                "KAFKA_CFG_LOG_FLUSH_INTERVAL_MESSAGES": "10000",
                "KAFKA_CFG_LOG_FLUSH_INTERVAL_MS": "1000",
                "KAFKA_CFG_LOG_RETENTION_HOURS": "168",
                "KAFKA_CFG_LOG4J_ROOT_LOGLEVEL": "INFO",
            },
        }
        if host_address is not None:
            protocol_map = self._services["kafka"]["environment"][
                "KAFKA_CFG_LISTENER_SECURITY_PROTOCOL_MAP"
            ]
            protocol_map = f"{protocol_map},PLAINTEXT_HOST:PLAINTEXT"
            self._services["kafka"]["environment"][
                "KAFKA_CFG_LISTENER_SECURITY_PROTOCOL_MAP"
            ] = protocol_map

            lst = self._services["kafka"]["environment"]["KAFKA_CFG_LISTENERS"]
            lst = f"{lst},PLAINTEXT_HOST://0.0.0.0:29092"
            self._services["kafka"]["environment"]["KAFKA_CFG_LISTENERS"] = lst

            alst = self._services["kafka"]["environment"][
                "KAFKA_CFG_ADVERTISED_LISTENERS"
            ]
            alst = f"{alst},PLAINTEXT_HOST://{host_address}:29092"
            self._services["kafka"]["environment"][
                "KAFKA_CFG_ADVERTISED_LISTENERS"
            ] = alst

    def add_influxdb(
        self,
        host: str = "influxdb",
        port: int = 8086,
        bucket: str = "teaching-bucket",
        org: str = "TEACHING",
        token: str = "O3Ht-WCNm01PYE77g_etSCbJC1c_xzUS-m9viu9EUVb-S0kJ8YCUpmg0DRizER-tkLP3REQpFibAz-LoATBIng==",
        dockerized: bool = True,
    ):
        self._services["influxdb"] = {
            "image": "influxdb:2.0",
            "container_name": "influxdb",
            "hostname": "influxdb",
            "ports": ["8086:8086"],
            "env_file": ".env",
        }
        self._env.update(
            {
                "INFLUXDB_HOST": host,
                "INFLUXDB_PORT": port,
                "INFLUXDB_BUCKET": bucket,
                "INFLUXDB_ORG": org,
                "INFLUXDB_TOKEN": token,
            }
        )
        if dockerized:
            self._env.update(
                {
                    "DOCKER_INFLUXDB_INIT_MODE": "setup",
                    "DOCKER_INFLUXDB_INIT_USERNAME": "teaching",
                    "DOCKER_INFLUXDB_INIT_PASSWORD": "asd123456",
                    "DOCKER_INFLUXDB_INIT_ORG": org,
                    "DOCKER_INFLUXDB_INIT_BUCKET": bucket,
                    "DOCKER_INFLUXDB_INIT_ADMIN_TOKEN": token,
                }
            )

    def compile(self, app_dir: str = "app"):
        if not os.path.exists(app_dir):
            os.makedirs(app_dir)

        app_path = os.path.realpath(app_dir)
        for _, v in self._services.items():
            if "volumes" in v:
                v["volumes"].append(
                    f"{os.path.join(app_path, 'config.yml')}:/app/config.yml",
                )

        with open(os.path.join(app_dir, "docker-compose.yml"), "w+") as f:
            yaml.dump(
                {"services": self._services},
                f,
                sort_keys=False,
                indent=2,
                default_flow_style=False,
            )

        with open(os.path.join(app_dir, ".env"), "w+") as f:
            for k, v in self._env.items():
                f.write(f'{k}="{v}"\n')

        with open(os.path.join(app_dir, "config.yml"), "w+") as f:
            yaml.dump(
                self._config,
                f,
                sort_keys=False,
                indent=2,
                default_flow_style=False,
            )
