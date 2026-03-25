# GAR Gauge Control

The purpose of this package is to aid the generation of a dataset of gauge images. It is responsible for adjusting a servo driving the needle of a gauge.

### Testing the servo control node

After compiling the project and sourcing the setup script, run the following command:
```bash
ros2 launch gar_gauge_control servo_test_launch.py 
```

In another terminal, run:
```bash
ros2 topic pub --once /set_extended_position gar_interfaces/msg/SetExtendedPosition "{id: 1, position: 0}"
```
This should zero out the servo. Alter the position if no movement occurs.