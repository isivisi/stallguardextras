# Utilizing stallguard
#
# Copyright (C) 2023 John Iannandrea    <jiannandrea@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, chelper

def lerp(start, end, delta):
    return (start + (end - start) * delta)

class DriverHelper:
    def __init__(self, sg, name, driver, stepper):
        self.name = name
        self.sg = sg
        self.printer = sg.printer
        self.driver = driver
        self.stepper = stepper
        self.history = None
        self.expectedRange = 25
        self.triggers = 0
        self.expectedPos = 0
        self.trapq = stepper.get_trapq() if stepper else None # trapezoidal velocity queue for stepper
        self.lastVelocity = 0
        self.lastStepPos = 0
        self.smoothedResult = 0

        self.deviationTolerance = sg.deviationTolerance

    def check(self, eventtime, updateTime):
        status = self.driver.get_status()
        standStillIndicator = False
        if (status['drv_status']): standStillIndicator = status['drv_status'].get('stst', False)

        last_move = self.getLastMove(eventtime)
        velocity = last_move.start_v + last_move.accel if last_move else 0
        steppos = self.stepper.get_commanded_position()

        result = int(self.driver.mcu_tmc.get_register('SG_RESULT'))
        self.smoothedResult = lerp(self.smoothedResult, result, updateTime * 0.5)

        if (standStillIndicator):
            self.triggers = 0
            self.expectedPos = result
            return

        if (self.history == None): self.history = result

        difference = self.expectedPos - result

        expectedDropRange = lerp(self.expectedRange, self.deviationTolerance, updateTime * 0.5)

        if (self.lastVelocity != velocity or steppos != self.lastStepPos):
            self.expectedPos = result
            #expectedDropRange = self.deviationTolerance * 2
            
        if (difference > expectedDropRange):
            #logging.warning("detecting slip %s/5" % (str(self.triggers,)))
            self.triggers += 1
            if (self.triggers <= 2):
                logging.warning("detecting slip, adjusting expected pos from %s to %s incase anomaly" % (str(self.expectedPos),str(lerp(self.expectedPos, result, 0.5))))
                self.expectedPos = self.expectedPos - expectedDropRange #lerp(self.expectedPos, result, 0.5) # give it a chance to readjust incase of drastic change duing normal ops
            if (self.triggers > 2):
                #if self.sg.testMode: logging.warning("Detecting motor slip on motor %s. %s value deviated by %s from previous. maximum %s deviation" % (self.name,str(result),str(difference), str(expectedDropRange)))
                self.printer.invoke_shutdown("Detecting motor slip on motor %s. %s value deviated by %s from previous. maximum %s deviation" % (self.name,str(result),str(difference), str(expectedDropRange)))
        else:
            self.triggers = max(0, self.triggers - 1)
        self.history = result
        self.expectedRange = expectedDropRange

        self.lastVelocity = velocity
        self.lastStepPos = steppos

    def getStatus(self):
        return {
            "sg_result": self.smoothedResult,
            "sg_expected": self.expectedPos,
            "sg_triggers": self.triggers,
        }

    def getLastMove(self, eventtime):
        print_time = self.printer.lookup_object('mcu').estimated_print_time(eventtime)
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new('struct pull_move[1]') # make a new move struct
        count = ffi_lib.trapq_extract_old(self.trapq, data, 1, 0., print_time) # fill move struct with last value in steppers movement queue
        if not count:
            return None
        # start_v, move_t, print_time, accel, start_x, start_y, start_z x_r, y_r, z_r
        return data[0]

    def driverHasField(self, field):
        return field in self.driver.fields.field_to_register.keys()

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
        self.sgthrs = config.get("sgthrs", None)
        self.sgt = config.get("sgt", None)
        self.testMode = bool(config.get("test_mode", False))
        self.deviationTolerance = int(config.get("deviation_tolerance", 100))
        self.loop = None

        self.drivers = {}
        self.extruders = []            

        self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        self.printer.register_event_handler("stepper_enable:motor_off", self.onMotorOff)
        self.printer.register_event_handler("homing:homing_move_begin", self.onMotorOn)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.enableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("TUNE_STALLGUARD", self.tune, desc="")
        gcode.register_command("DEBUG_STALLGUARD", self.debug, desc="")

    def onKlippyConnect(self):
        #self.setupDrivers()
        #self.enableChecks()

        toolhead = self.printer.lookup_object("toolhead")
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        steppers = kin.get_steppers()

        #logging.info(str([s.get_name() for s in steppers]))

        # find all compatable drivers
        for name, obj in self.printer.lookup_objects():
            if any(driver.lower() in name.lower() for driver in self.supportedDrivers):
                stepper = [s for s in steppers if s.get_name() == name.split(" ")[1]]
                logging.info("%s found %s" % (name, str(stepper)))
                if ('extruder' in name):
                    self.extruders.append(obj)
                else:
                    self.drivers[name.split(" ")[1]] = DriverHelper(
                        self,
                        name, 
                        obj, 
                        stepper[0] if len(stepper) else None, 
                    )
                logging.info("found stallguard compatible driver " + str(obj))

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
        for name, driverDef in self.drivers.items():
            driver = driverDef.driver
            # set drivers stallguard strength if specified
            if (self.sgthrs != None and driverDef.driverHasField('sgthrs')):
                driver.fields.set_field("sgthrs", int(self.sgthrs))
            elif (self.sgt != None and driverDef.driverHasField('sgt')):
                driver.fields.set_field("sgt", int(self.sgt))
    
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

    def queryObjects(self, gcmd):
        gcmd.respond_info(str(self.printer.lookup_objects()))

    def debug(self, gcmd):
        #gcmd.respond_info(str(self.printer.lookup_objects()))
        #gcmd.respond_info(str(self.drivers))
        responses = {}
        for name in self.drivers:
            last_move = self.drivers[name].getLastMove(self.printer.get_reactor().monotonic())
            responses[name] = {
                "result": int(self.drivers[name].driver.mcu_tmc.get_register('SG_RESULT')),
                "velocity": last_move.start_v + last_move.accel if last_move else 0,
                "mcu_pos": self.drivers[name].stepper.get_commanded_position()
            }
        gcmd.respond_info(str(responses))

    def enableChecks(self, gcmd=None):
        if (self.loop == None):
            reactor = self.printer.get_reactor()
            curTime = reactor.monotonic()
            self.loop = reactor.register_timer(self.doChecks, curTime + self.updateTime)

    def disableChecks(self, gcmd=None):
        if (self.loop != None):
            self.printer.get_reactor().unregister_timer(self.loop)
            self.loop = None

    def debugMove(self, gcmd):
        move = self.getLastMove(self.printer.get_reactor().monotonic())
        gcmd.respond_info("start_v:%s, move_t:%s, accel:%s\nx_r:%s x_y:%s x_z:%s" % (str(move.start_v), str(move.move_t), str(move.accel), str(move.x_r), str(move.y_r), str(move.z_r)))
    
    lastVelocity = 0
    def doChecks(self, eventtime):

        # TODO: cleanup, force rehome on x and y when slipping, cant do anything with z though :(
        
        # driver
        for name, driver in self.drivers.items():
            driver.check(eventtime, self.updateTime)

        return eventtime + self.updateTime

    def get_status(self, eventtime):
        data = {}
        for d in self.drivers:
            data[d] = self.drivers[d].getStatus()
        return data

def load_config(config):
    return StallGuardExtras(config)