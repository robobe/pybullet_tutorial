import logging
import structlog
import random
import time

# Basic logging setup
logging.basicConfig(
    level=logging.INFO,
)

# Structlog configuration
structlog.configure(
    processors=[
        structlog.processors.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.JSONRenderer(),  # <-- JSON output
    ],
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

log = structlog.get_logger()

# Simulate some robot control logs
for i in range(10):
    joint = random.choice(["shoulder", "elbow"])
    q = random.uniform(-1.5, 1.5)
    tau = random.uniform(0, 10)

    log.info(
        "pid_step",
        robot="arm1",
        joint=joint,
        q=round(q, 3),
        tau=round(tau, 3),
    )

    time.sleep(0.1)
