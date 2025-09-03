# BBSHD/BBS02 Super-charged Firmware

⚠️ This project is very much alpha. I run this firmware on my daily driver ebike, but this is only one possible combination of configuration options. Use at your own risk, expect bugs, and please report them.

This firmware is intended to replace the original Bafang firmware on the BBSHD/BBS02 motor controller. It takes a different direction and approach compared to the original firmware, providing new features and changing the behaviour of the motor to run more predictably and smoothly.

This firmware is mostly compatible with all displays & accessories that works with the original Bafang firmware. It will not work with anything that attempts to modify the configuration of the controller.

bbsxtra is based on [bbs-fw](https://github.com/danielnilsson9/bbs-fw). If you would like a more "traditional" configuration tool and functionality that more closely matches the stock Bafang controller, go check out @danielnilsson9's excellent work. I wouldn't know where to begin on reverse engineering the controller, so this project would not be possible without them.

## Major features of bbsxtra

- **Define power in watts instead of amps:** provide the same level of power no matter the battery level by compensating for the voltage drop as the battery depletes.
- **PID controller speed limit** to make the motor more smoothly and accurately hit the target max speed for throttle & PAS
- **Per assist level speed limit in km/h:** more accurately set the speed limit in absolute units instead of percentage
- **Partial Eggrider support:** supports the road/offroad switch in Eggrider, using the integrated standard/sport mode of bbsxtra.
- **More accurate speed sensing:** specify wheel circumference in mm along with the diameter in inches set on the display. The firmware compensates for the difference and shows the more accurate speed.

But wait, there's more! Here's what bbs-fw already provides:

- A bit more power without hardware modifications! (max 33A). 
- No upper voltage limit in software, can by default run up to 63V (maximum rating of components).
- Support lower voltage cutoff for use with e.g. 36V battery.
- Smooth Throttle/PAS override.
- Optional separate set of street legal & offroad assist levels which can be toggled by a key combination.
- Support setting road speed limit per assist level.
- Support setting cadence limit per assist level.
- Support cruise assist levels (i.e. motor power without pedal or throttle input).
- Thermal limiting gradual ramp down.
- Low voltage gradual ramp down.
- Voltage calibration for accurate LVC and low voltage ramp down.
- Display motor/controller temperature on standard display.
- Use of speed sensor is optional.

### Planned, but not yet added

- Cascading PID that sets the speed of the bike to hit a certain pedal cadence RPM, to keep your cadence in the "sweet spot"
- Speed-adjustable cruise control, press a button to make the bike continue at the current speed
- A configuration CLI tool to configure, compile and flash firmware
- Support mph and farenheit through said configuration tool (currently everything is in km/h and celsius)
- An arbitrary number of assist profiles that can be activated using different key combinations, not just standard/sport mode

### Not planned

- Windows support. This is built and tested on Linux, and should also work on MacOS. I'm open to PRs after I've built the configuration tool, but I won't make (or support) this myself.
- Other motor controllers. While bbs-fw branched out to support the TSDZ2, I do not own this and want to keep the scope of the project on Bafang. Feel free to fork.
- A GUI-based configuration/flashing tool. I'd love to do this but I don't have unlimited time and I'm a terrible UI designer, please help with PRs!

## How to install

This project takes inspiration from [ESPHome](https://esphome.io/), where you provide all the configuration up-front, and a custom firmware is compiled for you with your configuration baked in. This has a number of benefits, the main one being able to fit just the functionality you want to include inside the very space-constrained microcontroller, while also not needing to include code for a configuration interface.

As such, no pre-compiled binaries are provided. A command-line tool is WIP and will be provided in the future that will assist in compiling and flashing the controller based on a YAML configuration file. For now though, no documentation is yet provided - provide all configuration in `src/firmware/fwconfig.h`.

Please note that **once flashed, there's no way back**. It's not possible to read the firmware from the controller to back it up, and the original firmware is not freely available on the internet. If you're lucky your supplier might be able to help you out, or try asking around on the Endless Sphere forums. Hopefully though, you won't ever want to go back :)

⚠️ Warning: The firmware should NOT be flashed or configured while the eBike battery is charging!

## Supported Hardware

bbs-fw has been tested on the BBSHD, and I own a (newer) BBS02B 750W. The controllers are similar so it should work great on both, however bbsxtra is only tested on the BBS02B.

This firmware specifically supports the UART type of Bafang motor. Bafang is now only producing these motors with CAN-bus controllers. These controllers are completely different, and are also completely locked down. If you're looking to build a new bike with this firmware in 2025, you have 3 options:

1. ($) These motors have been produced since 2017 so check the second-hand market, they're reliable and easy to service,
2. ($$) Find an old stock UART motor still being sold,
3. ($$$) Purchase a CAN-bus motor, then source a UART controller and all accessories (the screen etc is not compatible). Replace the controller on the motor. *I've not tried this, it's at your own risk.*

### BBSHD

Revision | MCU          | Released    | Comment
-------- | ------------ | ----------- | --------------------
V1.4     | STC15W4K56S4 | ~2017       | V1.3 printed on PCB, sticker with 1.4.
V1.5     | IAP15W4K61S4 | ~2019       | V1.4 printed on PCB, sticker with 1.5.

### BBS02B
This firmware is suspected to be incompatible with older BBS02 controllers.
Newer BBS02Bs work great, but if you have an older controller it might not be a good idea to flash this firmware.

Revision | MCU          | Released    | Comment
-------- | ------------ | ----------- | --------------------
V1.?     | STC15F2K60S2 |             | Supported from BBS-FW version 1.1
V1.?     | IAP15F2K61S2 |             | Supported from BBS-FW version 1.1

BBS02A - No idea, not tested, not recommended to try unless you have an already bricked controller.

### Displays and Controller 

Only displays with the Bafang display protocol can work. 

Also the controllers need to be those, that are officially designed by Bafang, respectively Tongshen. 

Some shops sell kits with their own controller.

## Legal
* Installing this firmware will void your warranty.
* I cannot be held responsible for any injuries caused by the use of this firmware, use at your own risk.
