# Stall Guard Extras
A simple klipper plugin that queries for stall guard warnings

# Testing imitations
This has only been tested on TMC2209 drivers with stallguard2 on a Voron2.4. I plan on getting the newer tmc2209's with stallguard4 but thats probably all im going to be able to test with.

## Config
```
[stallguardextras]
update_time: 0.01 # time in seconds to query the drivers
# sgthrs: 250
# sgt: 0 #
```

## Moonraker Auto Update
```
[update_manager stallguardextras]
type: git_repo
path: ~/klipper/klippy/extras
origin: https://github.com/isivisi/stallguardextras.git
managed_services: klipper
```