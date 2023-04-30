# Stall Guard Extras
A simple klipper plugin that queries for stall guard warnings

## Config
```
[stallguardextras]
update_time: 1.0 # time in seconds to query the drivers
crash_threshold: 30
```

## Moonraker Auto Update
```
[update_manager stallguardextras]
type: git_repo
path: ~/klipper/klippy/extras
origin: https://github.com/isivisi/stallguardextras.git
managed_services: klipper
```