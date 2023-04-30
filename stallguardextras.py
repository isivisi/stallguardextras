import logging

class StallGuardExtras:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.updateTime = float(config.get("update_time", 1.0))
        self.loop = None
        #self.kin = self.printer.lookup_object('toolhead').get_kinematics()
        #self.kinSteppers = self.kin.get_steppers() # grab all steppers defined by the kinamatics

        self.drivers = self.printer.lookup_objects(module="TMC2209")

        #self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.enableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("DEBUG_STALLGUARD", self.debug, desc="")

    def onKlippyConnect(self):
        self.enableChecks()

    def debug(self, gcmd):
        for driver in self.drivers:
            gcmd.respond_info(vars(driver))
            #gcmd.respond_info("driver %s value %s" % (driver.name, driver.mcu_tmc.get_register('SG_RESULT')))

    def enableChecks(self):
        if (self.loop == None):
            reactor = self.printer.get_reactor()
            curTime = reactor.monotonic()
            self.loop = reactor.register_timer(self.doChecks, curTime + self.updateTime)

    def disableChecks(self):
        if (self.loop != None):
            self.printer.get_reactor().unregister_timer(self.loop)
            self.loop = None
    
    def doChecks(self):
        # diag_pin

        # SGTHRS // set threshold
        # SG_RESULT // get result
        
        # driver
        for driver in self.drivers:
            driver.mcu_tmc.get_register('SG_RESULT')

        return self.printer.get_reactor().NEVER

def load_config(config):
    return StallGuardExtras(config)