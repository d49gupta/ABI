import logging
import os

class CSVLogger:
    class CounterFilter(logging.Filter):
        """Internal class to inject the counter into each log record."""
        def __init__(self, parent):
            super().__init__()
            self.parent = parent

        def filter(self, record):
            # Attach the current counter value to the record
            record.msg_cnt = self.parent.counter
            return True

    def __init__(self, name="logger", log_dir="logs", level=logging.INFO):
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, f"{name}_logs.csv")
        self.counter = 0

        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Clear existing handlers/filters if re-initializing (important for Pi stability)
        if self.logger.handlers:
            self.logger.handlers.clear()
        if self.logger.filters:
            self.logger.filters.clear()

        # 1. Create and add the Filter
        self.logger.addFilter(self.CounterFilter(self))

        # 2. Setup Handler and Formatter
        handler = logging.FileHandler(self.log_path, mode="w")
        formatter = logging.Formatter(
            "%(asctime)s.%(msecs)03d,%(relativeCreated)d,%(module)s,%(lineno)d,%(levelname)s,%(msg_cnt)d,%(message)s",
            datefmt="%H:%M:%S"
        )
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

        # 3. Write CSV Header
        with open(self.log_path, "w") as f:
            f.write("timestamp,ms_since_start,module,line,level,msg_count,message\n")

    def info(self, msg, *args, **kwargs):
        self.counter += 1
        self.logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.counter += 1
        self.logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.counter += 1
        self.logger.error(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self.counter += 1
        self.logger.debug(msg, *args, **kwargs)