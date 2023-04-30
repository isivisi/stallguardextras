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
        self.crashThreshold = int(config.get("crash_threshold", 600))
        self.loop = None

        self.drivers = {}
        self.extruders = []

        logging.info("--- Stall guard extras loaded ---")

        for name, obj in self.printer.lookup_objects():
            if any(driver.lower() in name.lower() for driver in self.supportedDrivers):
                driverDetails = {
                    'driver': None,
                    'history': None
                }
                if ('extruder' in name): self.extruders.append(obj)
                else: 
                    driverDetails["driver"] = obj
                    self.drivers[name.split(" ")[1]] = driverDetails
                logging.info("found stallguard compatible driver " + str(obj))

        self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.enableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("DEBUG_STALLGUARD", self.debug, desc="")

    def onKlippyConnect(self):
        self.enableChecks()

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
    
    def doChecks(self, eventtime):
        # diag_pin

        # SGTHRS // set threshold
        # SG_RESULT // get result
        
        # driver
        for d in self.drivers:
            driverInfo = self.drivers[d]

            result = int(driverInfo["driver"].mcu_tmc.get_register('SG_RESULT'))

            if (driverInfo["history"] == None): driverInfo["history"] = result

            if (result > 400): 
                logging.info("driver %s value %s, current %s" % (d, str(result), 0))

            # raise self.printer.command_error()

            difference = result - driverInfo["history"]

            # todo: use (current / expected current) to influence the crashThreshold

            if (difference > self.crashThreshold):
                logging.info("Detecting motor slip, transient peaked %s for motor %s, %s more than previous value" % (str(result), d, str(difference)))
                self.printer.invoke_shutdown("Detecting motor slip, transient peaked %s for motor %s, %s more than previous value" % (str(result), d, str(difference)))
                #self.printer.invoke_shutdown("Shutdown due to stallguard on driver %s." % (d,str(result)))

            driverInfo["history"] = result
        
        return eventtime + self.updateTime

def load_config(config):
    return StallGuardExtras(config)