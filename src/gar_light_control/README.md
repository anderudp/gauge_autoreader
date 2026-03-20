# GAR Light Control
Automatic adjustment of lights for Gauge Autoreader.

## Using LIFX bulbs
Different LIFX bulbs have a range of parameters adjustable over WiFi. The models used for our purposes were a pair of SuperColors, which have RGB and white temperature adjustment, but not infrared capabilities.

### Onboarding
For the bulbs to be reachable over the network, they need to be *onboarded*. The officially supported method for this is via the usage of some proprietary application or another. Alternatively, [this script](https://github.com/tserong/lifx-hacks/blob/master/onboard.py) allows for app-free onboarding by performing the following steps.

#### Download the script
```bash
git clone https://github.com/tserong/lifx-hacks.git
```

#### Power on the bulb and connect to its WiFi Access Point
![Connecting to the bulb's AP](/.docs/lifx-wifi-ap.png)

#### Run the script with your network's credentials
```bash
chmod +x ./lifx-hacks/onboard.py && ./lifx-hacks/onboard.py YOUR_NETWORK_SSID YOUR_WIFI_PASSWORD
```

Enter `y` prompted to continue. After a few seconds, the bulb should momentarily blink once.

#### Reconnect to your network
The access point should disappear and the bulb should now be reachable over WiFi.

### Testing the bulb
After compiling the project ans sourcing the setup script, run the following command:
```bash
ros2 launch gar_light_control lifx_test_launch.py 
```
The node should be able to recognize the device as a light and display its data.

In another terminal, test the node's functionality by running:
```bash
ros2 topic pub --once /set_power gar_interfaces/msg/SetPower "{id: 0, power: true}"
```

### If onboarding failed
#### Reset the bulb
Turn the bulb on and off at least 5 times in roughly 1 second intervals. After a few seconds of stopping, the bulb should flash and the WiFi access point should reappear. 

#### Repeat execution of the onboarding script
Sometimes it fails the first time around. Make sure the SSID and Password are correct.

#### Ensure the bulb is on the expected IP address
```bash
ping 172.16.0.1
```
should give something akin to
```
64 bytes from 172.16.0.1: icmp_seq=1 ttl=255 time=4.56 ms
```
Otherwise, find the correct address and change the IP at the end of the script accordingly.

#### Ensure the correct wireless security protocol is selected
Find the name of your computer's wireless interface by running 
```bash
iw dev | awk '$1=="Interface"{print $2}'
```
This should give something like `wlan0`, `wlo1`, `wlp2s0`, etc.

Then, find the pairwise cipher for your network by running
```bash
sudo iw dev YOUR_WL_INTERFACE scan | grep -A10 -i "SSID: YOUR_NETWORK_SSID" | grep -A2 RSN
```
If the Pairwise ciphers field reads reads `CCMP`, the network uses AES encryption. Otherwise, consult the comments in the downloaded script and modify the final octet of the binary message output accordingly.

### LIFX-LAN API
The package uses a [third-party high-level API](https://github.com/mclarkk/lifxlan) to control LIFX bulbs. If not running the node in a Docker container, make sure to install it on the host by running
```bash
pip install --break-system-packages lifxlan
```
or using any virtual environment manager.