## **[AI-Toolkit](https://github.com/EU-TEACHING/teaching-ai-toolkit) services common instructions**

This subdirectory contains all the templates for instantiating an AI-Toolkit service. Here we provide all the details which are in common across all the modules, except for the ```fedserver``` module, which has a slightly different behaviour.

### **Internal communication**
The communication within the local environment (e.g., among modules which are instantiated in the same vehicle) implicitly happens via [RabbitMQ](https://www.rabbitmq.com/) thanks to the decorating function ```TEACHINGNode(producer: bool, consumer: bool)``` used in each service. This requires the following parameters, whose default values are specified in the ```rabbitmq_client.env``` file and are

```bash
RABBITMQ_HOST=rabbitmq
RABBITMQ_USER=teaching
RABBITMQ_PASSWORD=asd123456
RABBITMQ_PORT=5672
```

Also, in the ```docker-compose.yml``` file, ensure that each AI-toolkit service has the RabbitMQ broker listed under the ```depends_on``` statement.

### **Federated Learning**
The FL process involves multiple AI-Toolkit models deployed across the cloud-edge continuum. Each participant interacts with the others via [Kafka](https://kafka.apache.org/). Thanks to the abstraction of the underlying communication protocol provided by the decorating function ```FederatedNode(producer: bool, consumer: bool)``` on both clients and server, the interaction can happen in two main ways:
1. _Direct_: each participant exchanges the messages directly via Kafka;
2. _Indirect via File System_: each participant "produces and consumes" (i.e., saves and loads) the messages on two predefined directories, allowing third-party modules to process the messages before exchanging them via Kafka.

The protocol can be defined with the environment variable ```FED_BACKEND``` and setup with the protocol-specific parameters. For the _**direct**_ mode, the parameters are
```bash
FED_BACKEND=kafka
KAFKA_HOST=node249-hpc.isti.cnr.it
KAFKA_PORT=9092
GROUPID=xxxx # Must be **UNIQUE** for each process exchanging messages with Kafka
```

For the  _**indirect**_ mode, the parameters are
```bash
FED_BACKEND=fs
PRODUCE_DIR=/app/storage/federated/local/<model_name>/outgoing
CONSUME_DIR=/app/storage/federated/local/<model_name>/incoming
```
The specific behaviours of clients and server are defined below.

#### **Client**
For participating to the FL process, each client (i.e., the instance of an AI-toolkit module) requires only to set the following environment variable:
```bash
MODE=FEDERATED
```
The topics which are specific for the model involved are inferred by the class attribute FED_TOPIC.\
**IMPORTANT**: the parameter ```SERVICE_NAME``` parameter must be **UNIQUE** for each client, since it is used as an identifier of the client itself.

#### **Server**
Since the behaviour of the server is not dependent on the model involved, it requires the ```MODEL_TOPIC``` to be specified and coherent with the one defined on the service class, e.g.,
```bash
MODEL_TOPIC=rlmodule
```