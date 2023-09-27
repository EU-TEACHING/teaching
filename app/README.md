<img src="https://teaching-h2020.eu/sites/default/files/teaching55.png" height="80">

# :car: TEACHING App :airplane:

TEACHING: A computing Toolkit for building Efficient Autonomous appliCations leveraging Humanistic INtelliGence is an EU-funded project that designs a computing platform and the associated software toolkit supporting the development and deployment of autonomous, adaptive and dependable CPSoS applications, allowing them to exploit a sustainable human feedback to drive, optimize and personalize the provisioning of their services.

This repository can be taken as the **main access point** for the **TEACHING platform** (a collection of repositories) and for the **design and deployment** of new applications within the EU TEACHING 2020 project.

In particular, in this repository you will find a set of TEACHING applications defined as *docker-compose* configuration files following a parallel and distributed micro-services architecture. *Five different applications* showcasing the main features of the platform can be found in [scenarios](https://github.com/EU-TEACHING/teaching-app/tree/main/scenarios).

## Main Application Logic

The application logic is defined at the docker compose level with a graph connecting several nodes (docker images) and arcs (communications link).
Nodes can be *producers*, *consumers* or *both* and communicate via RabbitMQ. Following the INPUT_TOPIC(s) and OUTPUT_TOPIC(s) variables (often hardcoded within each node) is possible to understand the graph composition.

Nevertheless, every node is agnostic with respect to the graph composition and technology used in other nodes. Every node only implements a single function and process each *DataPacket* (a JSON based object) made available trought the INPUT_TOPIC.

Data flow and processing is mostly abstracted to the end TEACHING app designer and implemented in the main repository:

- [teaching-base](https://github.com/EU-TEACHING/teaching-base): Project for the base image(s) and classes.

## Implement your own TEACHING app

In order to implement a new TEACHING application, first it is important to understand if the nodes available are enough to fit your needs. 
In particular you may want to check the following repos:

- [teaching-base](https://github.com/EU-TEACHING/teaching-base): Repository for the base image(s) and classes of the TEACHING platform.
- [teaching-sensors](https://github.com/EU-TEACHING/teaching-sensors): Project for all the sensors that can be instantiated, from "file" sensors to cameras and wearables.
- [teaching-data](https://github.com/EU-TEACHING/teaching-data): Project for persistent storage, e.g., the influxdb instance.
- [teaching-ai-toolkit](https://github.com/EU-TEACHING/teaching-ai-toolkit): AI-Toolkit collecting and implementing the AI modules for a TEACHING application.
- [model-transfer-service](model-transfer-service): services to encrypt and transfer parametric models.

We also offer an indipendend service (that can be run in parallel) that can be useful to model Federated Learning use-cases:

- [teaching-model-aggregator](https://github.com/EU-TEACHING/teaching-model-aggregator): Module devoted to federated model aggregation.

If these repos do not provide the producing or consuming nodes you need. Then you can implement your own following the guidelines provided within each repository documentation (mostly README.md).

## Repository Structure

The main content of this repo:

 * [defaults](defaults): defaults environment variables useful for the TEACHING platform.
 * [old_version](old_version): some reference scripts from the previous version of the platform.
 * [scenarios](scenarios): TEACHING app applications defined as docker-composed config files.

## Supported Platforms

Theoretically any Docker supported system may be run a TEACHING application. However, the reference platforms for the projects are:

* Linux :white_check_mark:
* Windows 10, 11 under WSL2 :white_check_mark:
* IMX8 :warning: (see additional instructions [here](https://github.com/EU-TEACHING/teaching-app/blob/main/imx8_setup.md))
* Other ARM64 boards :warning: 

:white_check_mark: **Working**
:warning:  **In progress**

## Pre-Defined Apps (Also Called Scenarios)

|Scenario|Description|x86/x64|IMX8|Other ARM|
|-|-|-|-|-|
|Scenario 1|Record measurements of a vehicle in a route.|:white_check_mark:|:white_check_mark:|:warning:|
|Scenario 2|Personalization of driving experience using a RL model.|:white_check_mark:|:white_check_mark:|:warning:|
|Scenario 3|Process real time video stream and draw the measurements on the image.|:white_check_mark:|:white_check_mark:|:warning:|
|Scenario 4|Record measurements of a driver using a shimmer device.|:warning:|:warning:|:warning:|
|Scenario 5|Integration between AI-Toolkit and TEACHING_Platform.|:white_check_mark:|:warning:|:warning:|

:white_check_mark: **Tested**
:warning:  **Untested**

## Run your First App!

First of all you need to setup your environment. If you are using Windows, you can install [WSL2](https://docs.microsoft.com/en-us/windows/wsl/install) and [Docker Desktop](https://docs.docker.com/desktop/windows/install/). Then within Docker settings make sure to check the WSL integration tab and connect it to your virtual image. Once this is done you can run docker-compose directly within your WSL image (e.g. Ubuntu based). Alternatively, you can install and run everything from Windows powershell directly. Before running the app, make sure you have docker installed properly in your system using the latest stable version. If you are using Windows you can follow [this guide](https://docs.docker.com/desktop/windows/wsl/). If you still encounter some issues try to run the simplest scenario like [scenario_1.yaml](https://github.com/EU-TEACHING/teaching-app/blob/main/scenarios/scenario_1.yaml) without modifying it. If this also fails it means the problem is mostly due to the installation of the TEACHING app dependencies (docker images, etc.) out of the scope of this repository. You can run the scenarios with the following commands:

```
# if you want to run it in the iMX8 remember to set the ARCH global variable
# to "arm64" otherwise "amd64" (for laptops, etc.) will be set by default
export ARCH=arm64
git clone --recurse-submodules https://github.com/EU-TEACHING/teaching-app
cd teaching-app
source setup.sh
docker-compose -f scenarios/scenario_1.yaml up
```


## Partners
<table border=0 >
  <tr >
    <td> <img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/University-of-Pisa.png"  height="100"></td>   
    <td><img src="https://lowinfood.eu/wp-content/uploads/2021/01/HUA-Logo-Blue-RGB-1-1024x427.jpg"  height="130"></td>
    <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/CNR.png" height="80"></td>
  </tr>
  <tr >
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/I%26M.png" height="80"> </td>
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/TUG.png" height="80"></td>
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/AVL-Logo.jpg" height="80"></td>

  </tr>
  <tr >
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2021-04/marelli-logo-history.png" height="80"></td>
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/Thales_logo.jpg" height="80"></td>
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2018-06/itml400.png" height="80"></td>

  </tr>
  <tr >
  <td><img src="https://teaching-h2020.eu/sites/default/files/styles/mt_brands/public/2020-02/infineon_logo_rgb.jpg" height="80"></td>
  <td></td>
  <td></td>

  </tr>
</table>

## Fundings
<img src="https://teaching-h2020.eu/sites/default/files/inline-images/eu.jpg" height="50">

This project has received funding from the European Union’s Horizon 2020 Research and Innovation program under grant agreement No 871385.
