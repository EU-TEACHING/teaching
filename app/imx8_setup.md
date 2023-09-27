## IMX8 Additional Instructions :vertical_traffic_light:

For deployment on the IMX8 
Moving the docker directory to the external storage (you can skip this if the internal memory is large enough). 

```bash
 systemctl docker stop
 fdisk /dev/mmcblk1
 resize2fs /dev/mmcblk1p2
 cp -r /var/lib/docker /run/media/mmcblk1p2/var/lib/docker/
 rm -rf /var/lib/docker/
 ln -s /run/media/mmcblk1p2/var/lib/docker/ /var/lib/docker
 systemctl docker start
```
Install pip and update the docker-compose 

```bash
 python3 -m ensurepip
 python3 -m pip install --upgrade pip
 python3 -m pip uninstall docker-compose
 rm /usr/bin/docker-compose
 python3 -m pip install docker-compose
```
Clone the repo to the /run/media/mmcblk1p2 directory
  ```bash
  git clone https://github.com/EU-TEACHING/TEACHING_Platform.git
  ```
For updates use git pull inside the TEACHING_Platform directory
  ```bash
  git pull
  ```
