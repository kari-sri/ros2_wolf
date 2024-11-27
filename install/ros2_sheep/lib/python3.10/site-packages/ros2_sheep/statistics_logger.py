import logging

class StatisticsLogger:
    def __init__(self):
        self.logger = logging.getLogger('StatisticsLogger')
        self.logger.setLevel(logging.INFO)
        handler = logging.FileHandler('statistics.log')
        handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        
    def log_event(self, event_name, count):
        self.logger.info(f"{event_name}: {count} times")

