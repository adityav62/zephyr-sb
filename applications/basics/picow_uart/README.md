# Raspberry Pico W UART Demo for Zephyr RTOS - inspired by the Pico Serial Demo by Alex Bucknall (see LICENSE)

Example project for a simple UART using the Raspberry Pi Pico W, using Zephyr RTOS.

This is a simple demo for how get a Serial Console from the RPi Pico W over UART pins.

## Explanation

The following files are required to enable a USB Serial Console on the RPi Pico.

### prj.conf

The `*.conf` file is used by Zephyr as a `Kconfig` file, which specifies which kernel configuration options should be used for this application.
These settings are used along with the board/device specific settings to generate a specific Zephyr kernel configuration.

In our application we have the following configs:

```bash
CONFIG_SERIAL=y # Enable options for serial drivers
CONFIG_UART_CONSOLE=y # Sets the default for printk and printf to output to the UART serial console
CONFIG_UART_LINE_CTRL=y # This enables the API for apps to control the serial line, such as baud rate, CTS and RTS.
```

If you're unsure of what a specific option does, the Zephyr Docs helpfully provide a [nice tool](https://docs.zephyrproject.org/latest/kconfig.html) for searching for each mainline config does.

### app.overlay

Device Tree Overlays can be used by a Zephyr application to specify new hardware or re-assign existing hardware.
In our application, we'll use the `app.overlay` to inform Zephyr that we want to have a uart device.

Breaking this down even further and quoting the Linux documentation:

> The `chosen` node does not represent a real device, but serves as a place for passing data between firmware and the operating system, like boot arguments.
Data in the `chosen` node does not represent the hardware.
Typically the `chosen` node is left empty in .dts source files and populated at boot time.

The following is an excerpt from the original USB CDC ACM example used by Alex Bucknall - see the original project [here](https://github.com/Bucknalla/pico-zephyr-serial)

```dts
/ {
    chosen {
        zephyr,console = &cdc_acm_uart0;
    };
};
```

For our use case, we modify it as follows:

```dts
/ {
    chosen {
        zephyr,console = &uart0;
    };
};
```
The `zephyr,console` is a property within the chosen node, which indicates the device that should be used for console output, which is typically where system messages are sent. `&uart0` is a reference to another node in the device tree, defined elsewhere, that represents the actual device. The & symbol denotes that this is a reference or pointer to the node named uart0. The `zephyr,console` property within chosen is set to point to `&uart0`, which means the system should use the device referred to as `uart0` for console output.


Again quoting the Linux Kernel docs:

> A Devicetree's overlay purpose is to modify the kernel’s live tree, and have the modification affecting the state of the kernel in a way that is reflecting the changes.
Since the kernel mainly deals with devices, any new device node that result in an active device should have it created while if the device node is either disabled or removed all together, the affected device should be deregistered.

The following is an excerpt from the original USB CDC ACM example used by Alex Bucknall - see the original project [here](https://github.com/Bucknalla/pico-zephyr-serial)

```dts
&zephyr_udc0 {
    cdc_acm_uart0: cdc_acm_uart0 {
        compatible = "zephyr,cdc-acm-uart";
        label = "CDC_ACM_0";
    };
};
```
In the USB CDC ACM case discussed by Alex Bucknall, the RPi Pico board has a `zephyr_udc0` (USB device controller) node, which must be updated to specify that we want this to be a CDC ACM type device, specifically compatible with `"zephyr,cdc-acm-uart"`.

For our simple UART use case, we modify it as follows:

```dts
&uart0 {
    status = "okay";
    pinctrl-0 = <&uart0_default>;
    pinctrl-names = "default";
    current-speed = <115200>;
};
```
which is essentially already present in the Raspberry Pi Pico's device tree source include under 'rpi_pico-common.dtsi'. However, we place this here just so you know where certain properties can be changed. Say, you want to change the baud rate. You wouldn't change the dtsi file of your board since that is the default behaviour. Instead, you would change the baud rate in the app.overlay file.

### .vscode/c_cpp_properties.json

This file, while not specific to Zephyr, is helpful when developing Zephyr Applications using VSCode as it allows the C/CPlusPlus [IntelliSense](https://code.visualstudio.com/docs/editor/intellisense) tools to resolve the missing header/includes for your Zephyr libraries.
Without this `.vscode/c_cpp_properties.json` file in your project (and then ), you won't get any of the helpful tooltips that instruct you about specific functions or data structures.

## Glossary

- CDC -  Communications Device Class (USB)
- ACM -  Abstract Control Model (USB)
- UDC -  USB Device Controller
- UART - Universal Asynchronous Receiver-Transmitter
