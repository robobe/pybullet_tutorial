# install

- create virtual environment

```bash
python3 -m venv venv
```

```bash
source venv/bin/activate
```

```bash
pip install pybullet
```



> [!NOTE] 
> ### Exercise 1
> Write **pybullet** hello world
> Run the simulation and create empty simulation loop


---

## Add more util libraries
### logging using structlog

```bash
pip install structlog
```

```python title="code_example/logging_basic.py"
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

```

- Structlog is a **pipeline system**
- every log line create **event dictionary**
- that pass via the processor chain that modify the **event dictionary**

---


