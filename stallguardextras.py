import logging
from . import print_stats

class StallGuardExtras:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stallGuard = self.printer.load_object(config, 'stallguard')
        self.loop  = None
        self.kin = self.printer.lookup_object('toolhead').get_kinematics()
        self.kinSteppers = self.kin.get_steppers() # grab all steppers defined by the kinamatics

        self.drivers = self.printer.lookup_objects(module="tmc")

        self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")

    def onKlippyConnect():
        enableChecks()

    def enableChecks():
        if (self.loop == None):
            reactor = self.printer.get_reactor()
            curTime = reactor.monotonic()
            self.loop = reactor.register_timer(self.doChecks, curTime + 1)

    def disableChecks():
        if (self.loop != None):
            self.printer.get_reactor().unregister_timer(self.loop)
            self.loop = None
    
    def doChecks(self):
        # diag_pin

        # SGTHRS
        # SG_RESULT

        self.mcu_tmc.get_register('SG_RESULT')

        return self.printer.get_reactor().NEVER