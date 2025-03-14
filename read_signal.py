import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
import re

# Define the sampling rate
sampling_rate = 10000  # 10 kHz

# Initialize lists to hold signals and peak frequencies
signals = []
peak_frequencies = []
signal_labels = []

# Read and parse the data
with open('signals.txt', 'r') as file:
    content = file.read()

# Split the content into blocks separated by 'Sampled Signal'
blocks = content.split('Sampled Signal')

for idx, block in enumerate(blocks[1:], start=1):  # Skip the first split as it will be before the first 'Sampled Signal'
    # Split the block into signal data and peak frequency
    parts = block.strip().split('Peak Frequency1:')
    if len(parts) != 2:
        continue  # Skip if the block doesn't have both signal and frequency
    
    signal_str, freq_str = parts
    # Extract numbers from the signal
    signal_numbers = re.findall(r'\d+\.\d+|\d+', signal_str)
    signal = [float(num) for num in signal_numbers]
    if not any(signal):  # Skip all-zero signals
        continue
    signals.append(signal)
    signal_labels.append(f'Signal {idx}')
    
    # Extract the peak frequency
    try:
        peak_freq = float(freq_str.strip())
    except ValueError:
        peak_freq = np.nan  # Handle cases where frequency is 'nan'
    peak_frequencies.append(peak_freq)

# Check if there are any signals to plot
if not signals:
    print("No valid signals found to plot.")
    exit()

# Create a figure for Time-Domain Signals
plt.figure(figsize=(15, 6))
plt.title('All Sampled Signals - Time Domain')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')

# Plot each time-domain signal
for signal, label in zip(signals, signal_labels):
    signal = np.array(signal)
    N = len(signal)
    T = 1.0 / sampling_rate  # Sampling interval
    time = np.linspace(0.0, N*T, N)
    plt.plot(time, signal, alpha=0.7, label=label)

plt.legend(loc='upper right', fontsize='small', ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()

# Create a figure for Frequency-Domain Signals
plt.figure(figsize=(15, 6))
plt.title('All Sampled Signals - Frequency Domain')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude')

# Define color map
cmap = plt.get_cmap('tab10')
color_idx = 0

# Plot each frequency-domain signal
for idx, (signal, peak_freq, label) in enumerate(zip(signals, peak_frequencies, signal_labels)):
    signal = np.array(signal)
    N = len(signal)
    T = 1.0 / sampling_rate  # Sampling interval
    
    # Perform FFT
    yf = fft(signal)
    xf = fftfreq(N, T)[:N//2]
    
    # Compute magnitude
    magnitude = 2.0/N * np.abs(yf[0:N//2])
    
    # Choose color
    color = cmap(color_idx % 10)
    color_idx += 1
    
    plt.plot(xf, magnitude, alpha=0.7, label=label, color=color)
    
    # Mark the peak frequency if it's a number
    if not np.isnan(peak_freq):
        plt.axvline(x=peak_freq, color=color, linestyle='--', alpha=0.7)

plt.legend(loc='upper right', fontsize='small', ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()
