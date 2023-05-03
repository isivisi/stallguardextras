# Stall Guard Extras
A simple klipper plugin that queries for stall guard warnings

# Testing imitations
This has only been tested on TMC2209 drivers

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