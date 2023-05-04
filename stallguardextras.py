# Utilizing stallguard
#
# Copyright (C) 2023 John Iannandrea    <jiannandrea@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, chelper

supportedDrivers = [
    'TMC2130',
    'TMC2209',
    'TMC2660',
    'TMC5160',
]

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
        self.expectedRange = sg.deviationTolerance
        self.triggers = 0
        self.expectedPos = 0
        self.trapq = stepper.get_trapq() if stepper else None # trapezoidal velocity queue for stepper
        self.lastVelocity = 0
        self.lastStepPos = 0
        self.smoothedResult = 0

        self.moving = False
        self.lastMicroStep = 0
        self.lastMove = None

        self.hasChanged = True

        self.deviationTolerance = sg.deviationTolerance

    # Perform some checks using the stallguard result to determine if the motor is 
    # slipping / about to slip.
    #
    # The default method is defining a threshold for the stallguard value to go below,
    # and checking fi it does. This only seems to work at one set speed.
    #
    # I try to dynamically set the threshold for different movements by determing when
    # the motor will change momentum / change to a different move. Then grab the sg value,
    # wait to see if it deviates and assume stalled if it does.
    #
    # works great except I need to find a better way to determine when the specifc motor is
    # to change momentum. sometimes the motor changes to a different, but consistent
    # value and its not picking up on that.
    def check(self, eventtime, updateTime):
        status = self.driver.get_status()
        standStillIndicator = False
        if (status['drv_status']): standStillIndicator = status['drv_status'].get('stst', False)
        result = int(self.driver.mcu_tmc.get_register('SG_RESULT'))

        if (standStillIndicator):
            self.expectedPos = result
            self.triggers = 0

        if (self.history == None): self.history = result

        #if (movingChangedThisTick):
        #    self.triggers = 0
        #    self.expectedPos = result
            #self.expectedRange = self.deviationTolerance * 2

        self.hasChanged = True if self.hasMovementChanged(eventtime) else self.hasChanged

        difference = self.expectedPos - result
        expectedDropRange = lerp(self.expectedRange, self.deviationTolerance, updateTime * 0.5)
            
        if (abs(difference) > expectedDropRange):
            self.triggers += updateTime

            #if (self.hasChanged):
                #self.expectedPos = result
                #self.triggers = 0
                #self.hasChanged = False
                
            #if (self.triggers <= 1):
            #    logging.warning("detecting slip, adjusting expected pos from %s to %s incase anomaly" % (str(self.expectedPos),str(lerp(self.expectedPos, result, 0.5))))
            #    self.expectedPos = self.expectedPos - difference #lerp(self.expectedPos, result, 0.5) # give it a chance to readjust incase of drastic change duing normal ops
            if (self.triggers > 0.1):
                if (not self.hasChanged):
                    self.printer.invoke_shutdown("Detecting motor slip on motor %s. %s value deviated by %s from previous. maximum %s deviation" % (self.name,str(result),str(difference), str(expectedDropRange)))
                else:
                    logging.warning("%s slip ignored, intended change" % (self.name,))
                    self.expectedPos = result
                    self.triggers = 0
                    self.hasChanged = False
        else:
            self.triggers = max(0, self.triggers - updateTime)
        self.history = result
        self.expectedRange = expectedDropRange

    def hasMovementChanged(self, eventtime):
        movingChangedThisTick = False

        #last_move = self.getLastMove(eventtime)
        #velocity = last_move.start_v + last_move.accel if last_move else 0
        #movingChangedThisTick = self.lastVelocity != velocity

        #steppos = self.lastStepPos
        #if (self.getStartPosOfLastMove(eventtime)):
        #steppos = self.stepper.get_commanded_position()
        #movingChangedThisTick = steppos != self.lastStepPos

        #microstepcounter = int(self.driver.mcu_tmc.get_register('MSCNT')) # 0 to 1023

        # testing new thing
        move = self.getMove(eventtime)
        if (move and self.lastMove):
            if (move.first_clock >= self.lastMove.last_clock):
                movingChangedThisTick = True

        #if (microstepcounter != self.lastMicroStep and not self.moving):
        #    movingChangedThisTick = True
        #    self.moving = True
        #elif (microstepcounter == self.lastMicroStep and self.moving):
        #    movingChangedThisTick = True
        #    self.moving = False

        #self.lastVelocity = velocity
        #self.lastStepPos = steppos
        #self.lastMicroStep = microstepcounter
        self.lastMove = move

        return movingChangedThisTick

    def getStatus(self):
        return {
            "sg_result": self.history,
            "sg_expected": self.expectedPos,
            "sg_triggers": self.triggers,
        }

    # grab specifically just the start_position of the most recent move we are sending to the mcu
    def getMove(self, eventtime):
        if (not self.stepper): return None
        clock = self.stepper.get_mcu().print_time_to_clock(eventtime)
        # first_clock, last_clock, start_position, step_count, interval, add
        steps, count = self.stepper.dump_steps(1, 0, 1<<63)
        if not steps: return None
        return steps[0]

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
        
        #extruderSensing = ExtruderDetection(config.getsection('extruder_detection'))

    def onKlippyConnect(self):
        #self.setupDrivers()
        #self.enableChecks()

        toolhead = self.printer.lookup_object("toolhead")
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        steppers = kin.get_steppers()

        #logging.info(str([s.get_name() for s in steppers]))

        # find all compatable drivers
        for name, obj in self.printer.lookup_objects():
            if any(driver.lower() in name.lower() for driver in supportedDrivers):
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
                "microstepcounter": int(self.drivers[name].driver.mcu_tmc.get_register('MSCNT'))
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

class ExtruderDetection:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.updateTime = float(config.get("update_time", 0.1))
        self.jamDetectEnabled = bool(config.get("jam_detect", False))
        self.runoutDetect = bool(config.get("runout_detect", False))

def load_config(config):
    return StallGuardExtras(config)