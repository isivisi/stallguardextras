# Utilizing stallguard
#
# Copyright (C) 2023 John Iannandrea    <jiannandrea@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class StallGuardExtras:
    supportedDrivers = [
        'TMC2130',
        'TMC2209',
        'TMC2660',
        'TMC5160',
    ]
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.updateTime = float(config.get("update_time", 0.1))
        self.crashThreshold = int(config.get("crash_threshold", 25))
        self.sgthrs = config.get("sgthrs", None)
        self.sgt = config.get("sgt", None)
        self.loop = None

        self.drivers = {}
        self.extruders = []

        #self.csv = []

        logging.info("--- Stall guard extras loaded ---")

        for name, obj in self.printer.lookup_objects():
            if any(driver.lower() in name.lower() for driver in self.supportedDrivers):
                driverDetails = {
                    'driver': None,
                    'history': None,
                    'expectedRange': 25,
                    'triggers': 0,
                    'expectedPos': 0,
                    'type': name.split(" ")[0]
                }
                if ('extruder' in name): self.extruders.append(obj)
                else: 
                    driverDetails["driver"] = obj
                    self.drivers[name.split(" ")[1]] = driverDetails
                logging.info("found stallguard compatible driver " + str(obj))

        #self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        self.printer.register_event_handler("stepper_enable:motor_off", self.onMotorOff)
        self.printer.register_event_handler("homing:homing_move_begin", self.onMotorOn)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.enableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("DEBUG_STALLGUARD", self.debug, desc="")
        gcode.register_command("DEBUG_QUERY_OBJECTS", self.queryObjects, desc="")
        gcode.register_command("TUNE_STALLGUARD", self.tune, desc="")

    def onKlippyConnect(self):
        #self.setupDrivers()
        self.enableChecks()

    def onMotorOff(self, eventtime):
        self.disableChecks()

        #with open("stallout.csv", "w") as txt_file:
        #    for line in self.csv:
        #        txt_file.write(line + "\n")
        
        #self.csv = []

    def onMotorOn(self, eventtime):
        self.setupDrivers()
        self.enableChecks()
    
    def setupDrivers(self):
        for d in self.drivers:
            driver = self.drivers[d]["driver"];
            # set drivers stallguard strength if specified
            if (self.sgthrs != None and self.driverHasField(driver, 'sgthrs')):
                driver.fields.set_field("sgthrs", int(self.sgthrs))
            elif (self.sgt != None and self.driverHasField(driver, 'sgt')):
                driver.fields.set_field("sgt", int(self.sgt))
    
    def driverHasField(self, driver, field):
        return field in driver.fields.field_to_register.keys()
    
    # move the toolhead safely within printing speeds to finetune the stallguard settings
    def tune(self, gcmd):
        self.disableChecks()
        self.setupDrivers()

        toolhead = self.printer.lookup_object('toolhead')
        # X, Y, Z, E = toolhead.get_position()
        # toolhead.move([x,y,z,e], max_v)
        #todo loop through different speeds moving along axis
        # slowly lower threshold and transient settings until movement no longer considered stalling

        # output settings

        # SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        configfile.set('stallguardextras', 'sgthrs', '0')

    def lerp(self, start, end, delta):
        return (start + (end - start) * delta)

    def queryObjects(self, gcmd):
        gcmd.respond_info(str(self.printer.lookup_objects()))

    def debug(self, gcmd):
        #gcmd.respond_info(str(self.printer.lookup_objects()))
        #gcmd.respond_info(str(self.drivers))
        responses = {}
        for name in self.drivers:
            responses[name] = int(self.drivers[name]["driver"].mcu_tmc.get_register('SG_RESULT'))
        gcmd.respond_info(str(responses))

    def enableChecks(self):
        if (self.loop == None):
            reactor = self.printer.get_reactor()
            curTime = reactor.monotonic()
            self.loop = reactor.register_timer(self.doChecks, curTime + self.updateTime)

    def disableChecks(self):
        if (self.loop != None):
            self.printer.get_reactor().unregister_timer(self.loop)
            self.loop = None
    
    lastVelocity = 0
    def doChecks(self, eventtime):
        # diag_pin

        # SGTHRS // set threshold
        # SG_RESULT // get result 0 - 510

        velocity = self.printer.objects["motion_report"].get_status(eventtime)["live_velocity"]
        
        # driver
        for d in self.drivers:
            driverInfo = self.drivers[d]

            status = driverInfo["driver"].get_status()
            
            # driver.get_status mcu_phase_offset, phase_offset_position, run_current, hold_current

            result = int(driverInfo["driver"].mcu_tmc.get_register('SG_RESULT'))
            standStillIndicator = True
            if (status['drv_status']): standStillIndicator = status['drv_status'].get('stst', False)
            #actualMotorCurrent = status['drv_status']['cs_actual']

            if (driverInfo["history"] == None): driverInfo["history"] = result

            if (velocity != self.lastVelocity): driverInfo["expectedPos"] = result

            difference = driverInfo["expectedPos"] - result
            
            expectedDropRange = 50

            expectedDropRange = self.lerp(driverInfo["expectedRange"], 50, self.updateTime * 0.5)
            if (self.lastVelocity != velocity): expectedDropRange = 100
            
            if (difference > expectedDropRange and not standStillIndicator):
                driverInfo["triggers"] += 1
                if (driverInfo["triggers"] > 10):
                    self.printer.invoke_shutdown("Detecting motor slip on motor %s" % (d,))
            else:
                driverInfo["triggers"] = max(0, driverInfo["triggers"] - 1)
            driverInfo["history"] = result
            driverInfo["expectedRange"] = expectedDropRange

        self.lastVelocity = velocity
        
        return eventtime + self.updateTime

    def get_status(self, eventtime):
        data = {}
        for d in self.drivers:
            data[d] = {
                "sg_result": self.drivers[d]["history"],
                "sg_expected": self.drivers[d]["expectedPos"],
                "sg_triggers": self.drivers[d]["triggers"],
            }
        return data

def load_config(config):
    return StallGuardExtras(config)