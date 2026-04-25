# PID

## Antiwindup
[video:  Anti-windup for PID control | Understanding PID Control, Part 2 ](https://youtu.be/NVLXCwc8HzM?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)
[code](code/pid_windup.py)

![alt text](images/anti_windup.png)

## Noise and low-pass filtering

[code](code/pid_noise_filter.py)

This simulation adds random noise to the measured box position before the PID controller sees it. The script also runs the noisy measurement through a first-order low-pass filter.

Use the sliders to change:

- `Noise std`: amount of sensor noise
- `Filter cutoff Hz`: how strongly the low-pass filter smooths the signal
- `Use filter`: switch the PID feedback between noisy position and filtered position
- `Kp`, `Ki`, `Kd`: PID gains

The simulation sends PlotJuggler JSON/CSV data for:

- `position/setpoint`
- `position/feedback`
- `position/true`
- `position/noisy`
- `position/filtered`
- `control/force`

Compare `position/noisy` with `position/filtered` to see the filter effect. A lower cutoff gives a smoother signal, but adds more delay.

For a simple signal-only view without PyBullet, run [noise_low_pass_plot.py](code/noise_low_pass_plot.py). It sends a clean sine wave, noisy signal, and filtered signal to PlotJuggler.
