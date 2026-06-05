# Gauge autoreader

The purpose of this project is the creation of a dataset designed to train robots to automatically read gauges.

See individual package `README`s for further descriptions.

## Deployment using VS Code and Docker

The easiest way to get started is to run the project in the container defined by `.devcontainer/Dockerfile`. For instructions on installing Docker, take a look at [this guide](https://docs.docker.com/engine/install).

It is recommended to use VS Code with the `Dev Containers` extension for development. Terminal profiles and Python paths are preconfigured in `.vscode`. 

### Running graphical packages

In order to be able to launch graphical packages (`rqt_graph`, `rviz`, etc.) from within the container, you must provide access to the host's X server (or its Wayland compositor if XWayland is available). To do this, run the following command from a terminal launched on the host.
```bash
xhost +local:$(docker ps -q --filter "label=devcontainer.local_folder")
```
> [!NOTE]
> This will give graphics access to all containers marked as VS Code Dev Containers. Keep security implications in mind and exercise caution running it.


## Using multiple computers over LAN

`ROS_DOMAIN_ID` is set to `88` in `.devcontainer/devcontainer.json`. Make sure to set it up in each terminal not launched from the project's Docker container by running
```bash
export ROS_DOMAIN_ID=88
echo $ROS_DOMAIN_ID  # For checking
```

or set it up permanently by adding it to the terminal profile:
```bash
echo 'export ROS_DOMAIN_ID=88' >> ~/.bashrc &&  source ~/.bashrc
echo $ROS_DOMAIN_ID
```

## Compiling packages for ARM

You may want to run some of these packages from low-power ARM-based machines communicating with the others using ROS2-over-WiFi. These ARM machines may lack the performance to compile the necessary packages. Since ROS-native cross-compilation is deprecated, it is set up using ARM emulation via QEMU.

### Preparing the ARM machine

Make sure ROS2 Jazzy is installed, along with any package dependencies.

### Install QEMU

Install the virtualization packages from your distro's package repositories. Afterwards, restart your computer.

#### Ubuntu
```bash
sudo apt install qemu-user-static
```
#### Fedora
```bash
sudo dnf install qemu-user-static
```
#### Arch
```bash
sudo pacman -S qemu-user-static
```

### Select packages for cross-compilation

In `.arm64/Dockerfile.arm64`, modify the line
```dockerfile
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select <YOUR PACKAGES HERE>
```
to contain your desired packages and any dependencies. Also make sure to install any package dependencies in the `builder` container using `apt-get` and/or `pip` to ensure successful linking.

### Run the compilation script

The build process can be launched automatically by running
```bash
.arm64/build-arm64.sh
```
If running for the first time, you may need to make it executable using 
```bash
chmod +x .arm64/build-arm64.sh
```

This script will create a `build-arm64` folder if it does not exist and export the ROS2 binary executables there as a tarball. Running it for the first time may take over 10 minutes.

### Copy to ARM machine

Copy the files to the target machine using SCP by running
```bash
scp install-<DATETIME>.tar.gz user@<ARM_IP>:/home/user/ws/
```

You can find out the IP of the target machine using
```bash
hostname -I
```

### Running the node on ARM

Extract the folder, source the executables and the node should run without issue.

```bash
tar -xzf archive.tar.gz
source install/setup.bash
```

## CAD models used

TBA
