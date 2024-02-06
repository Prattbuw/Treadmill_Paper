import nidaqmx
from nidaqmx.constants import AcquisitionType
import numpy as np

# DAC setup
fps = 200
trial_duration =30 # was 25 seconds
dac_rate = 10000.0
trigger_duration = trial_duration + 1
n_samp = int(trigger_duration * dac_rate)
sig = np.zeros(n_samp)
interval_size = int(dac_rate * (1/fps))
samples_high  = round(interval_size/2) # duty cycle of 0.5
start_r = samples_high
while start_r < n_samp:
    sig[start_r:start_r+samples_high] = 5
    start_r += interval_size

out = np.array(sig)

with nidaqmx.Task() as task:
    task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
    task.timing.cfg_samp_clk_timing(dac_rate, sample_mode=AcquisitionType.FINITE, samps_per_chan=len(out))
    task.start()
    task.write(out, auto_start=False, timeout=trigger_duration+1)
    task.stop()
    # task.close()
