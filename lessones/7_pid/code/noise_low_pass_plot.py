import math
import random
import time
from pathlib import Path

from plotjuggler_udp import PlotJugglerUdpClient, TimeBase

DT = 1 / 100
SIGNAL_DURATION = 10.0
SIGNAL_STEPS = int(SIGNAL_DURATION / DT)
SIGNAL_FREQUENCY_HZ = 0.5
NOISE_STD = 0.35
FILTER_CUTOFF_HZ = 1.5
NOISE_SEED = 5


class LowPassFilter:
    def __init__(self, cutoff_hz, dt, initial_value=0.0):
        self.dt = dt
        self.value = initial_value
        self.set_cutoff(cutoff_hz)

    def set_cutoff(self, cutoff_hz):
        self.cutoff_hz = max(cutoff_hz, 0.001)
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        self.alpha = self.dt / (tau + self.dt)

    def update(self, measurement):
        self.value += self.alpha * (measurement - self.value)
        return self.value


random.seed(NOISE_SEED)

low_pass_filter = LowPassFilter(
    cutoff_hz=FILTER_CUTOFF_HZ,
    dt=DT,
)

SCRIPT_DIR = Path(__file__).resolve().parent.as_posix()
plotjuggler = PlotJugglerUdpClient(
    save=False,
    dump_duration=SIGNAL_DURATION,
    output_dir=SCRIPT_DIR,
    time_base=TimeBase.RELATIVE,
    extra_fieldnames=[
        "signal/noisy",
        "signal/filtered",
        "noise/value",
        "filter/cutoff_hz",
    ],
)

for step in range(SIGNAL_STEPS):
    signal_time = step * DT
    clean_signal = math.sin(2.0 * math.pi * SIGNAL_FREQUENCY_HZ * signal_time)
    noise = random.gauss(0.0, NOISE_STD)
    noisy_signal = clean_signal + noise
    filtered_signal = low_pass_filter.update(noisy_signal)

    plotjuggler.send(
        clean_signal,
        filtered_signal,
        extra_data={
            "signal/noisy": noisy_signal,
            "signal/filtered": filtered_signal,
            "noise/value": noise,
            "filter/cutoff_hz": FILTER_CUTOFF_HZ,
        },
    )

    time.sleep(DT)

