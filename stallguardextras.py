import logging
from . import print_stats

class StallGuardExtras:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stallguard = self.printer.load_object(config, 'stallguard')

        self.printer.register_event_handler("toolhead:set_position". self.movement)
        self.printer.register_event_handler("toolhead:manual_move", self.movement)
    
    def movement(self):
        