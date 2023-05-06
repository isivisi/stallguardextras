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

# copy of c struct data
class MoveHelper:
    def __init__(self, rawMove):
        self.first_clock = rawMove.first_clock
        self.last_clock = rawMove.last_clock
        self.start_position = rawMove.start_position
        self.step_count = rawMove.step_count
        self.interval = rawMove.interval
        self.add = rawMove.add

    def getStatus(self):
        return {
            "first_clock": self.first_clock,
            "last_clock": self.last_clock,
            "start_position": self.start_position,
            "step_count": self.step_count,
            "interval": self.interval,
            "add": self.add,
        }

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

        #self.enableChecks = bool(sg.config.getsection(self.name).get('check_collisions', True))

        self.moving = False
        self.stepsMoving = False
        self.lastMicroStep = 0
        self.lastMove = None
        self.lastKnownPrintTime = 0

        self.hasChanged = True

        self.deviationTolerance = sg.deviationTolerance

    
    def onEnabled(self):
        self.moving = False
        self.stepsMoving = False
        self.lastMove = None
        self.lastMicroStep = -1
        self.expectedPos = 0
        self.hasChanged = True
        self.history = None

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
    # Then we wait for a driver command to be picked up to make sure deviation was expected
    # or not.
    #                                  |----| 0.250 second max
    #   sg_value_deviated: - - - - - - X--OK - - -
    #   move_sent_to_drv : - - - - - - - -X - - -
    #
    # works great except I need to find a better way to determine when the specifc motor is
    # to change momentum. sometimes the motor changes to a different, but consistent
    # value and its not picking up on that.
    def check(self, eventtime, updateTime, onDetect):
        status = self.driver.get_status()
        standStillIndicator = False
        if (status['drv_status']): standStillIndicator = status['drv_status'].get('stst', False)
        result = int(self.driver.mcu_tmc.get_register('SG_RESULT'))

        if (standStillIndicator):
            self.expectedPos = result
            self.triggers = 0
            self.hasChanged = False

        if (self.history == None): self.history = result

        # movement changed
        changedThisTick = self.hasMovementChanged(eventtime)
        self.hasChanged = True if changedThisTick else self.hasChanged

        difference = self.expectedPos - result
        expectedDropRange = lerp(self.expectedRange, self.deviationTolerance, updateTime * 0.5)
            
        if (abs(difference) > expectedDropRange):
            self.triggers += updateTime

            #if (self.triggers <= 1):
            #    logging.warning("detecting slip, adjusting expected pos from %s to %s incase anomaly" % (str(self.expectedPos),str(lerp(self.expectedPos, result, 0.5))))
            #    self.expectedPos = self.expectedPos - difference #lerp(self.expectedPos, result, 0.5) # give it a chance to readjust incase of drastic change duing normal ops
            if (self.triggers > 0.250):
                if (not self.hasChanged):
                    onDetect(result, difference)
                else:
                    logging.warning("%s slip ignored, intended change" % (self.name,))
                    self.expectedPos = result
                    self.triggers = 0
                    self.hasChanged = False
        else:
            self.triggers = max(-0.250 if self.hasChanged else 0, self.triggers - updateTime)

        if (self.triggers <= -0.250):
            logging.warning("%s expected change detected" % (self.name,))
            self.triggers = 0
            self.hasChanged = False

        self.history = result
        self.expectedRange = expectedDropRange

    def hasMovementChanged(self, eventtime):
        movingChangedThisTick = False

        steppos = self.lastStepPos
        steppos = self.stepper.get_commanded_position()
        #movingChangedThisTick = steppos != self.lastStepPos

        microstepcounter = int(self.driver.mcu_tmc.get_register('MSCNT')) # 0 to 1023

        # testing new thing
        move = self.getMove(eventtime)
        clock = self.stepper.get_mcu().print_time_to_clock(self.stepper.get_mcu().estimated_print_time(eventtime))
        if (move and self.lastMove):
            if (move.first_clock != self.lastMove.first_clock and not self.moving):
                movingChangedThisTick = True
                self.moving = True
                self.lastMove = move
                logging.warning("move started")
            elif (clock > self.lastMove.first_clock and self.moving):
                movingChangedThisTick = True
                self.moving = False
                logging.warning("move ended")
                if (move.first_clock > self.lastMove.first_clock):
                    self.lastMove = move
                    self.moving = True
                    logging.warning("move continued")
        else:
            self.lastMove = move

        if (microstepcounter != self.lastMicroStep and not self.stepsMoving):
            logging.warning("movement started")
            movingChangedThisTick = True
            self.stepsMoving = True
        elif (microstepcounter == self.lastMicroStep and self.stepsMoving):
            logging.warning("movement stopped")
            movingChangedThisTick = True
            self.stepsMoving = False
            
        #self.lastVelocity = velocity
        #self.lastStepPos = steppos
        self.lastMicroStep = microstepcounter
        self.lastKnownPrintTime = clock
        #self.lastMove = move

        return movingChangedThisTick

    def getStatus(self):
        return {
            "sg_result": self.history,
            "sg_expected": self.expectedPos,
            "sg_triggers": self.triggers,
            "latest_move": self.lastMove.getStatus() if self.lastMove else None
        }

    # grab specifically just the start_position of the most recent move we are sending to the mcu
    def getMove(self, eventtime):
        if (not self.stepper): return None
        clock = self.stepper.get_mcu().print_time_to_clock(eventtime)
        # first_clock, last_clock, start_position, step_count, interval, add
        steps, count = self.stepper.dump_steps(1, 0, clock)
        if not steps: return None
        return MoveHelper(steps[0])

    def driverHasField(self, field):
        return field in self.driver.fields.field_to_register.keys()

class CollisionDetection:
    def __init__(self, config):
        if (not config):
            return
        self.config = config
        self.printer = config.get_printer()
        self.updateTime = float(config.get("update_time", 0.125))
        self.disableOnHome = bool(config.get("disable_on_home", True))
        self.sgthrs = config.get("sgthrs", None)
        self.sgt = config.get("sgt", None)
        self.testMode = bool(config.get("test_mode", False))
        self.deviationTolerance = int(config.get("deviation_tolerance", 50))
        self.loop = None

        self.drivers = {}
        self.extruders = []            

        self.faultOnDetection = True

        self.printer.register_event_handler("klippy:connect", self.onKlippyConnect)

        self.printer.register_event_handler("stepper_enable:motor_off", self.onMotorOff)
        
        if (self.disableOnHome):
            self.printer.register_event_handler("homing:homing_move_begin", self.onHomingOn)
            self.printer.register_event_handler("homing:homing_move_end", self.onHomingOff)
            self.printer.register_event_handler("homing:home_rails_begin", self.onHomingOn)
            self.printer.register_event_handler("homing:home_rails_end", self.onHomingOff)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ENABLE_STALLGUARD_CHECKS", self.enableChecks, desc="")
        gcode.register_command("DISABLE_STALLGUARD_CHECKS", self.disableChecks, desc="")
        gcode.register_command("TUNE_STALLGUARD", self.tune, desc="")
        gcode.register_command("DEBUG_STALLGUARD", self.debug, desc="")

        self.printer.add_object("collision_detection", self)

    def onKlippyConnect(self):
        #self.setupDrivers()
        self.enableChecks()

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

    def onHomingOn(self, *args):
        #self.disableChecks()
        self.faultOnDetection = False
    
    def onHomingOff(self, *args):
        #self.setupDrivers()
        #self.enableChecks()
        self.faultOnDetection = True
    
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
        configfile.set('collision_detection', 'sgthrs', '0')

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
            #for name, driver in self.drivers.items():
                #driver.onEnabled()
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
    
    def doChecks(self, eventtime):

        # TODO: cleanup, force rehome on x and y when slipping, cant do anything with z though :(
        
        # driver
        for name, driver in self.drivers.items():
            driver.check(eventtime, self.updateTime, self.onDetect)

        return eventtime + self.updateTime

    # event when deviation is detected and no move command is detected
    def onDetect(self, value, diff):
        if (faultOnDetection):
            self.printer.invoke_shutdown("Detecting motor slip on motor %s. %s value deviated by %s from previous. maximum %s deviation" % (self.name,str(value),str(diff), str(self.deviationTolerance)))

    def get_status(self, eventtime):
        data = {}
        for d in self.drivers:
            data[d] = self.drivers[d].getStatus()
        return data

class ExtruderDetection:
    def __init__(self, config):
        if (not config):
            return
        self.printer = config.get_printer()
        self.updateTime = float(config.get("update_time", 0.1))
        self.jamDetectEnabled = bool(config.get("jam_detect", False))
        self.runoutDetect = bool(config.get("runout_detect", False))

class StallGuardExtras:
    def __init__(self, config):
        self.collisionDection = CollisionDetection(config.getsection('collision_detection'))
        self.extruderSensing = ExtruderDetection(config.getsection('extruder_detection'))

def load_config(config):
    return StallGuardExtras(config)