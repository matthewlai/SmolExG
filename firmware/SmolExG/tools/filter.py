import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Filter parameters
fs = 2000  # Sampling rate in Hz
pass_low, pass_high = 20, 500
trans_width = 15
numtaps = 401
window = 'hamming'
fixed_point_denom = 16777216  # 24 bit seems to give us indistinguishable response.

coeffs_per_line = 8

edges = [0, pass_low - trans_width, pass_low, pass_high, pass_high + trans_width, 0.5 * fs]

# Method 1: Direct high-pass (pass_zero=False)
h = signal.remez(numtaps=numtaps, bands=edges, desired=[0, 1, 0], weight=[3, 0.1, 1], fs=fs)

h = (h * fixed_point_denom).round().astype(np.int32)

print(np.sum(h))

print(f'constexpr int32_t FILTER_{pass_low}_{pass_high}[{h.shape[0]}] = ''{')
for i in range(h.shape[0]):
  if i % coeffs_per_line == 0:
    print('  ', end='')
  print(f'{h[i]}', end=', ')
  if i % coeffs_per_line == (coeffs_per_line - 1):
    print('')
print('')
print('};')


# Compute frequency responses
h = h.astype(np.float32) / fixed_point_denom
w, h = signal.freqz(h, 1, worN=8192, fs=fs)

mag = 20 * np.log10(np.abs(h) + 1e-10)
phase = np.angle(h)

fig, axes = plt.subplots(1, 1, figsize=(12, 10))

axes.plot(w, mag, label='Magnitude (db)', linewidth=2)
#axes.plot(w, phase, label='Phase', linewidth=2)
axes.axhline(-3, color='r', linestyle='--', alpha=0.5, label='-3dB')
axes.axvline(pass_low, color='g', linestyle='--', alpha=0.5, label=f'{pass_low} Hz cutoff')
axes.axvline(pass_high, color='b', linestyle='--', alpha=0.5, label=f'{pass_high} Hz cutoff')
axes.grid(True, alpha=0.3)
axes.set_xlabel('Frequency (Hz)')
axes.set_ylabel('Magnitude (dB)')
axes.set_title('FIR Filter Frequency Response')
axes.legend()
axes.set_xlim([0, 1000])
axes.set_ylim([-80, 5])

plt.show()
