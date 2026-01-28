import logging
import os

class CSVLogger:
    def __init__(self, name="logger", log_dir="logs", level=logging.INFO):
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, f"{name}_logs.csv")

        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        if not self.logger.handlers:
            handler = logging.FileHandler(self.log_path, mode="w")

            formatter = logging.Formatter(
                "%(asctime)s.%(msecs)03d,%(relativeCreated)d,%(module)s,%(lineno)d,%(levelname)s,%(message)s",
                datefmt="%H:%M:%S"
            )

            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        if os.path.getsize(self.log_path) == 0:
            with open(self.log_path, "w") as f:
                f.write("timestamp,ms_since_start,module,line,level,message\n")

    def set_level(self, level):
        self.logger.setLevel(level)

    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)
