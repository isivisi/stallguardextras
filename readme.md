# Stall Guard Extras
A simple klipper plugin that queries for stall guard warnings

# WIP!

# Testing limitations
This has only been tested on TMC2209 drivers

## Config
```
# load stallguardextras.py from ~/klipper/klippy/extras
[stallguardextras]

[collision_detection]
update_time: 0.01       # time in seconds to query the drivers
deviation_tolerance: 50
# sgthrs: 250
# sgt: 0 #

[extruder_detection]
#WIP
```

## Moonraker Auto Update
```
[update_manager stallguardextras]
type: git_repo
path: ~/klipper/klippy/extras
origin: https://github.com/isivisi/stallguardextras.git
managed_services: klipper
```