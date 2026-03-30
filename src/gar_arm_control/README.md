# GAR Arm Control

This package is responsible for the control of robot arms in all relevant workflows.

## UFactory Lite 6

A pair of UFactory Lite 6 arms were used in the production of the gauge dataset. The following section details their configuration method.

### Subnet configuration

First, connect the arms and the PC using an RJ45 cable. For the arms to be reachable, the controlling PC needs to have an address on the same subnet as the arms. In the case of our Lite6es, the subnet was `192.168.1.xxx`. To configure it on GNOME, go to `Settings -> Network -> Wired`, turn it on using the switch if it is not active, and after clicking the gear button, navigate to the `IPv4` tab, and set an arbitrary unoccupied address on the appropriate subnet and set the mask to `255.255.255.0`. See the example below.

![Setting up the right subnet in GNOME](/.docs/network-config.png)

### Controlling multiple arms from the same PC

Since most PCs only have one Ethernet port, the arms can either be connected to a router on your network or connected to your PC via a network switch hub.

### Testing connectivity

To test your PS's connection to the arms, download [UFactory Studio](https://www.ufactory.us/ufactory-studio) from the manufacturer's website. Make it executable by running 
`chmod +x ufactory-studio-client-linux-VERSION.AppImage`, enter your arm's IP address, and twist the emergency stop button to release the joint servos. Upon enabling the arm in the GUI, a series of clicks should be heard, meaning that the arm was successfully unlocked.
