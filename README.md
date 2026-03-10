# Autoware CARLA Leaderboard (AWCL)
This project enables the execution and evaluation of CARLA Leaderboard challenge scenarios using the modular Autoware autonomous driving software stack. By automating scenario deployment and bridging it with Autoware Universe, the framework provides a seamless environment for robust, scenario-based testing.

### Key Feature
- Fully compatible with **CARLA 0.9.16** and **Autoware Universe 1.7.1** (February 2026 release).

- High-Performance Data Pipeline: Optimized sensor data transport and processing utilizing **CARLA 0.9.16 native DDS** for transmission and a **C++ backend** for low-latency translation.
 
- Precise Reproducibility: Ensures high scenario consistency through **internal time tracking** and **synchronization** between the CARLA simulator and the Autoware software stack.

- Native support for CARLA Town01, Town07, and Town10.

- Includes a Digital Twin of the CARLA Audi e-tron.

- Integrated Traffic Light Detection within the Autoware pipeline.

- Enables a privileged mode that provides Autoware with ground truth localization and perception data for training data generation.

### Contact & Support
For questions, bug reports, or feature requests, please use the [GitHub Issues](https://github.com/TUMFTM/autoware_carla_leaderboard/issues) page.

**Project Maintainer:**
* **Name:** Gemb Kaljavesi
* **Email:** [gemb.kaljavesi@tum.de](mailto:gemb.kaljavesi@tum.de)

### Paper
If you use this or the other associated repos, please cite our preprint:
**Empirical mapping of technical bottlenecks in autonomous driving: A cross-platform evaluation of simulated and real-world performance**<br>Gemb Kaljavesi, Domagoj Majstorovic, Maxime Clement, Frank Diermeyer 

```
@inproceedings{awcl_26,
  title = {Empirical mapping of technical bottlenecks in autonomous driving: A cross-platform evaluation of simulated and real-world performance},
  author = {Kaljavesi, Gemb and Majstorovic, Domagoj and Clement, Maxime and Diermeyer, Frank},
  year = {2026},
  howpublished = {Preprint}
}
```

### Youtube Demo Video + Quickstart
| Demo                                            | Quickstart                                         |
| ----------------------------------------------------- | ---------------------------------------------- |
| [![DEMO](https://img.youtube.com/vi/0cwbvZAWZuw/0.jpg)](https://youtu.be/0cwbvZAWZuw)                                       | [![Quickstart](https://img.youtube.com/vi/_P4Ctp4JUhg/0.jpg)](https://youtu.be/_P4Ctp4JUhg)                             |


## DEV-Workflow
For simplicity, we use Rocker, which streamlines Docker commands.
To better understand the framework, look here first: [Click here to see the Desin Infos](doc/Design.md)
### Build
1) Build or pull the  autoware_carla_leaderboard docker:
```
./docker/docker_setup.sh 

docker pull tumgeka/autoware_carla_leaderboard:0.9.16
```

2) Start autoware_carla_leaderboard docker and mount the repo directory
```
rocker --nvidia --x11 --privileged --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --volume <autoware_carla_leaderboard> --name aw_carla_leaderboard_0916 -- tumgeka/autoware_carla_leaderboard:0.9.16
```

Example launch command using your own cyclone_localhost.xml file.
```
rocker --nvidia --x11 --privileged --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --volume /home/tofadmin/gemb/cyclone_localhost.xml -e CYCLONEDDS_URI=/home/tofadmin/gemb/cyclone_localhost.xml --volume /home/tofadmin/gemb/awcl --name aw_carla_leaderboard_0916 -- tumgeka/autoware_carla_leaderboard:0.9.16
```
3) Navigate to autoware_carla_leaderboard and import repos files necessary
```
cd <autoware_carla_leaderboard>
mkdir -p src/external \
	&& vcs import --recursive src/external < docker/carla.repos \
	&& vcs import --recursive src/external < docker/ros2.repos
```

4) Build Code
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run
0) Download CARLA here: [CARLA_0916](https://autoware-files.s3.us-west-2.amazonaws.com/carla/CARLA_294096eb1-dirty.tar.gz)
and Launch Carla with native DDS
```
./CarlaUE4.sh --ros2
```
1) Launch Docker: Start autoware_carla_leaderboard and mount the repo. Same as above!
```
rocker --nvidia --x11 --privileged --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --volume <autoware_carla_leaderboard> --name aw_carla_leaderboard_0916 -- tumgeka/autoware_carla_leaderboard:0.9.16
```

Example launch command using your own cyclone_localhost.xml file.
```
rocker --nvidia --x11 --privileged --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --volume /home/tofadmin/gemb/cyclone_localhost.xml -e CYCLONEDDS_URI=/home/tofadmin/gemb/cyclone_localhost.xml --volume /home/tofadmin/gemb/autoware_carla_leaderboard --name aw_carla_leaderboard_0916 -- tumgeka/autoware_carla_leaderboard:0.9.16
```
2) Open a second terminal for the C++ bridge.
```
docker exec -it aw_carla_leaderboard_0916 bash
```
3) In both terminals, navigate to your autoware_carla_leaderboard directory. Source the built packages and CARLA environments (Note: carla_envs is optional for the C++ bridge but won't cause issues).
```
cd <autoware_carla_leaderboard>
source install/setup.bash
source carla_envs.sh # ./carla_env.sh will not work
```
4) Run Bridge: Start the C++ bridge (it persists across scenario restarts).
```
ros2 launch autoware_carla_cpp_bridge aw_carla_cpp_bridge.launch.py
```
5) Set the right config in the config/config.yaml
```
# Choose your agent and also choose your scenario
evaluation:
  agent_setup:
    # Path to Agent's .py file to evaluate
    agent: src/autoware_agent/aw_priviliged.py # For priviliged Mode
    agent: src/autoware_agent/aw_e2e.py # For e2e Mode
  simulation_setup:
    routes: resources/routes/town10_parking_crossing.xml
```
6) Only start the scenario after Autoware has fully launched.
```
python3 src/external/leaderboard/leaderboard/leaderboard_evaluator.py --conf_file_path=config/config.yaml
```

## Autoware universe

### General
The current recommended Autoware Version is: **1.7.1**
> 1) Pull the autoware docker ghcr.io/autowarefoundation/autoware:universe-devel-cuda-1.7.1
> 2) Clone the 1.7.1 autoware repo
> 3) vcs import all autoware.repos
> 4) Clone the audi etron vehicle model: https://github.com/TUMFTM/carla_audi_etron
> 5) Mount the repos into the autoware docker and build everything 

You can find Town01, Town07 and Town10 here:
[Map_Assets](https://github.com/TUMFTM/autoware_carla_leaderboard/releases/tag/v1.0.0-maps)


### Optional
Quick and easy ways to increase the vehicle's speed in Autoware Universe, although other parameters also affect the driving profile.
**/autoware_launch/config/planning/scenario_planning/common/common.param.yaml**
```diff
-    max_vel: 4.17           # max velocity limit [m/s]
+    max_vel: 20.0           # max velocity limit [m/s]
 
     # constraints param for normal driving
     normal:
-      min_acc: -1.0         # min deceleration [m/ss]
-      max_acc: 1.0          # max acceleration [m/ss]
-      min_jerk: -1.0        # min jerk [m/sss]
-      max_jerk: 1.0         # max jerk [m/sss]
+      min_acc: -2.0         # min deceleration [m/ss]
+      max_acc: 2.0          # max acceleration [m/ss]
+      min_jerk: -2.0        # min jerk [m/sss]
+      max_jerk: 2.0         # max jerk [m/sss]
 
     # constraints to be observed
     limit:
       min_acc: -2.5         # min deceleration limit [m/ss]
-      max_acc: 1.0          # max acceleration limit [m/ss]
-      min_jerk: -1.5        # min jerk limit [m/sss]
-      max_jerk: 1.5         # max jerk limit [m/sss]
+      max_acc: 2.5          # max acceleration limit [m/ss]
+      min_jerk: -2.5        # min jerk limit [m/sss]
+      max_jerk: 2.5         # max jerk limit [m/sss]

```

**/autoware_launch/config/system/component_state_monitor/topics.yaml**
Comment out most of it, since aw in priviliged mode will miss a lot of the intermediate topics. 

## E2E vs Priviliged Mode
There are two modes available.


### E2E
This will send sensor data raw data to autoware.
1) In the config.yaml set the agento to aw_e2e.py
2) Launch autoware e2e
```
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=carla_audi_etron_vehicle sensor_model:=carla_audi_etron_sensor_kit map_path:=<map_path>
```

### Priviliged
This will send sensor data raw data to autoware.
1) In the config.yaml set the agento to aw_priviliged.py
2) Modify the planning_simulator.launch.xml 
```diff
-  <arg name="use_sim_time" default="$(var scenario_simulation)"/>
+  <arg name="use_sim_time" default="true"/>

-<arg
-    name="localization_sim_mode"
-    default="api"

+<arg
+    name="localization_sim_mode"
+    default="none"
```
3) Launch autoware priviliged
```
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=carla_audi_etron_vehicle sensor_model:=carla_audi_etron_sensor_kit map_path:=<map_path>
```


# Cyclone Settings
Its recommended to update your cyclone settings. Here you can find more: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/#cyclonedds-configuration

TUM Docker Workflow for NO Broadcasting (At least at the autoware side)
### No Broadcasting Multidocker
```sudo vim /usr/bin/enable_multicast_lo```
```
// filename: enable_multicast_lo
#!/bin/bash
 
ip l set lo multicast on
```
```sudo vim /lib/systemd/system/enable_multicast_lo.service```
```
// filename: enable_multicast_lo.service
[Unit]
Description=Localhost Multicasting
After=network.target
 
[Service]
ExecStart=/usr/bin/enable_multicast_lo
 
[Install]
WantedBy=multi-user.target
```
```
sudo chmod +x /usr/bin/enable_multicast_lo

sudo systemctl daemon-reload

sudo systemctl enable enable_multicast_lo.service

sudo systemctl start enable_multicast_lo.service
```
### AW Sensor Settings
```sudo vim /etc/sysctl.d/10-cyclone-max.conf```
```
// filename: 10-cyclone-max.conf
# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```
```sudo sysctl --system```
### cyclone_localhost.xml
```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="docker0" priority="0" multicast="default" />
                <NetworkInterface name="lo" priority="1" multicast="default" />
            </Interfaces>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
       	<Internal>
          <SocketReceiveBufferSize min="10MB"/>
          <Watermarks>
	    <WhcHigh>500kB</WhcHigh>
          </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
```
