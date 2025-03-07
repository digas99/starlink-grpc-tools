import logging

class LoggerFormater(logging.Formatter):
    RED = '\033[91m'
    RESET = '\033[0m'

    def formatTime(self, record, datefmt=None):
        return f"{record.created:.9f}"

    def format(self, record):
        log_str = super().format(record)
        if record.levelno == logging.ERROR:
            log_str = f"{self.RED}{log_str}{self.RESET}"
        return log_str

class RosLogger():
	def __init__(self, name):
		self.logger = logging.getLogger(name)
		self.logger.setLevel(logging.INFO)

		self.formatter = LoggerFormater('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
		self.handler = logging.StreamHandler()
		self.handler.setFormatter(self.formatter)
		self.logger.addHandler(self.handler)
                  
	def info(self, message):
		self.logger.info(message)
        
	def error(self, message):
		self.logger.error(message)