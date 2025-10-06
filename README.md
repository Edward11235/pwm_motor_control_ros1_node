# pwm_motor_control_ros1_node
### Udev Rule for Arduino Nano

To ensure consistent naming of your Arduino Nano device, you can create a udev rule. This will allow the device to always appear at `/dev/arduino`.

1. **Identify Your Arduino Nano**

    ```sh
    lsusb
    ```

2. **Create the Udev Rule**

    ```sh
    sudo nano /etc/udev/rules.d/99-arduino-nano_motor.rules
    # or
    sudo nano /etc/udev/rules.d/99-arduino-mega_motor.rules
    # or
    sudo nano /etc/udev/rules.d/99-arduino-uno_motor.rules
    ```

3. **Add the Following Rule**

    ```sh
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="arduino_motor"
    # or
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino_motor"
    # or
    SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino_motor"
    # or (orignal arduino uno)
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2a03", ATTRS{idProduct}=="0043", SYMLINK+="arduino_motor"
    ```

4. **Reload the Udev Rules**

    ```sh
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
5. **TO run it**
