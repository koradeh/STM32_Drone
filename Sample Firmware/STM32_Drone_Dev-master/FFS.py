# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt



import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

choice = 1


if choice == 1 :
    # Load your CSV file
    data = pd.read_csv(r'c:\users\Vincent\log\Dterm.csv')  # Using raw string literal

    # Extract the 'x' (time) and 'y' (gyro value) columns
    time = data['time'].values
    gyro_values = data['Roll_Rate_PID.Dterm'].values

    # Sampling period (assuming evenly spaced time)
    sampling_period = np.mean(np.diff(time))
    sampling_frequency = 1 / sampling_period  # Sampling frequency in Hz

    # Plot the spectrogram
    plt.figure(figsize=(10, 6))
    plt.specgram(gyro_values, Fs=sampling_frequency, NFFT=256, noverlap=128, cmap='plasma')
    plt.title('Spectrogram of Gyro Data (Roll Rate PID D-Term)')
    plt.xlabel('Time (s)')
    plt.ylabel('Frequency (Hz)')
    plt.colorbar(label='Intensity (dB)')
    plt.grid(True)
    plt.show()


if choice == 2:
    # Load your CSV file
    data = pd.read_csv(r'c:\users\Vincent\log\Dterm.csv')  # Using raw string literal

    # Extract the 'x' (time) and 'y' (gyro value) columns
    time = data['time'].values
    gyro_values = data['Roll_Rate_PID.Dterm'].values

    # Calculate the sampling period (assuming evenly spaced time)
    sampling_period = np.mean(np.diff(time))
    sampling_frequency = 1 / sampling_period  # Sampling frequency in Hz

    # Perform the FFT on the gyro data
    fft_result = np.fft.fft(gyro_values)
    fft_freqs = np.fft.fftfreq(len(gyro_values), d=sampling_period)

    # Plot the frequency spectrum (we plot only positive frequencies)
    plt.figure(figsize=(10, 6))
    plt.plot(fft_freqs[:len(fft_freqs)//2], np.abs(fft_result[:len(fft_result)//2]))
    plt.title('Frequency Spectrum of Gyro Data')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.show()