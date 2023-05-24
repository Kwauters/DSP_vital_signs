import math
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt


def phase_unwrap(phase: float) -> float:
    """
    Takes in a phase sample and makes sure it is between π and -π by adding 2π while the phase is smaller or subtract
    2π while the phase is larger.
    :param phase: A phase bin of the rangebin which is currently being processed
    :return: The unwrapped phase
    """
    result = phase
    if result > np.pi:
        fit_times = math.floor(result / (2 * np.pi))
        result -= (fit_times + 1) * 2 * np.pi
    if result < -np.pi:
        fit_times = math.floor(-result / (2 * np.pi))
        result += (fit_times + 1) * 2 * np.pi
    return result


def process_files():
    """
    Processes the files for each rangebin one by one and prints the breathing rate and heart rate to the console.
    """
    dirname_project = os.path.dirname(__file__)
    dirname_recordings = os.path.join(dirname_project, "recordings")
    for filename in os.listdir(dirname_recordings):
        with open(os.path.join(dirname_recordings, filename)) as f:
            values = np.array([complex(i) for i in f.readlines()])
            breathing_rate, heart_rate = process_rangebin(values)
            print(f"\nCurrent file: {filename}")
            print("================================")
            print(f"Breathing rate: {breathing_rate} per minute")
            print(f"Heart rate: {heart_rate} per minute")


def process_rangebin(rangebin: np.ndarray) -> tuple[int, int]:
    """
    Processes the given rangebin and returns the breathing rate and heart rate
    :param rangebin: The numpy array with the complex numbers
    :return: A tuple with the breathing rate and heart rate
    :rtype: tuple
    """
    phases = np.array(
        [phase_unwrap(p) for p in np.angle(rangebin)])  # Begin with getting the phase of the complex numbers

    phase_differences = np.subtract(  # Take the difference between φ[n] and φ[n-1] for all samples
        phases[0:len(phases) - 1],
        phases[1:]
    )

    # Define the bandpass filter coefficients:
    # For the breathing filter the cutoff frequencies are 0.1 and 0.6 Hz
    # For the heart rate filter the cutoff frequencies are 0.8 and 4 Hz
    breathing_filter = scipy.signal.butter(5, [0.1, 0.6], "bp", fs=20, analog=False, output="sos")
    heart_filter = scipy.signal.butter(5, [0.8, 4], "bp", fs=20, analog=False, output="sos")

    # Use the filter coefficients to filter the signal to find the phases for the breathing and heart rate
    filtered_breathing = scipy.signal.sosfilt(breathing_filter, phase_differences)
    filtered_heart = scipy.signal.sosfilt(heart_filter, phase_differences)

    # Take the FFT and different plots
    fft_breathing = np.absolute(scipy.fft.fft(filtered_breathing))
    fft_heart = np.absolute(scipy.fft.fft(filtered_heart))

    plt.figure(figsize=(16, 8), layout="constrained")
    plt.subplot(231)
    plt.plot(range(len(phase_differences)), phase_differences)
    plt.title("Phase differences")

    plt.subplot(232)
    plt.plot(range(len(filtered_breathing)), filtered_breathing)
    plt.title("Filtered phase differences for the breathing rate")

    plt.subplot(233)
    plt.plot(range(len(filtered_heart)), filtered_heart)
    plt.title("Filtered phase differences for the heart rate")

    plt.subplot(235)
    plt.plot(range(len(fft_breathing)), fft_breathing)
    plt.title("Frequency spectrum for the breathing rate")

    plt.subplot(236)
    plt.plot(range(len(fft_heart)), fft_heart)
    plt.title("Frequency spectrum for the heart rate")
    plt.show()

    # Find the highest peak of the breathing and heart rate frequency bins, this should the breathing are heart rate.
    peaks_breathing, peak_height_breathing = scipy.signal.find_peaks(fft_breathing, height=0.2)
    peaks_heart, peak_height_heart = scipy.signal.find_peaks(fft_heart, 40)

    breathing_rate, heart_rate = (0, 0)

    if len(peak_height_breathing["peak_heights"]) != 0:
        breathing_rate = peaks_breathing[np.argmax(peak_height_breathing["peak_heights"])]

    if len(peak_height_heart["peak_heights"]) != 0:
        heart_rate = peaks_heart[np.argmax(peak_height_heart["peak_heights"])]

    return breathing_rate, heart_rate


process_files()
