import logging
import structlog

logging.basicConfig(level=logging.INFO, format="%(message)s")

structlog.configure(
    processors=[
        structlog.processors.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.dev.ConsoleRenderer(),  # <-- colored output
    ],
    logger_factory=structlog.stdlib.LoggerFactory(),
)

log = structlog.get_logger()

log.info("arm_move", joint="shoulder", target=1.0)
log.warning("joint_limit", lower=-1.57, upper=1.57)
log.error("motor_failure", joint="shoulder")