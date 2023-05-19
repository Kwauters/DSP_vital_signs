import math
import os
import numpy as np
import scipy
import matplotlib.pyplot as plt


def process_files():
    for file in os.listdir("C:/dsp/Python Code vital signs/recordings"):
        with open("C:/dsp/Python Code vital signs/recordings/" + file) as f:
            values = np.array([complex(i) for i in f.readlines()])
            breathing_rate, heart_rate = process_rangebin(values)
            print(f"\nCurrent file: {file}")
            print("================================")
            print(f"Breathing rate: {breathing_rate}")
            print(f"Heart rate: {heart_rate}")


def process_rangebin(rangebin: np.ndarray):
    """
    Processes the given rangebin and returns the breathing rate and heart rate
    :param rangebin: The numpy array with the complex numbers
    :return: A tuple with the breathing rate and heart rate
    :rtype: tuple
    """
    phases = np.angle(rangebin)

    for phase in phases:
        if phase > np.pi:
            fit_times = math.floor(phase / (2 * np.pi))
            phase -= (fit_times + 1) * 2 * np.pi

        if phase < -np.pi:
            fit_times = math.floor(-phase / (2 * np.pi))
            phase += (fit_times + 1) * 2 * np.pi

    phase_differences = np.subtract(
        phases[0:len(phases) - 1],
        phases[1:]
    )

    breathing_filter = scipy.signal.butter(5, [0.005, 0.03], "bp", fs=20, analog=False, output="sos")
    heart_filter = scipy.signal.butter(5, [0.04, 5], "bp", fs=20, analog=False, output="sos")

    plt.plot(range(len(phase_differences)), phase_differences)
    plt.title("The phase differences")
    plt.show()

    filtered_breathing = scipy.signal.sosfilt(breathing_filter, phase_differences)
    filtered_heart = scipy.signal.sosfilt(heart_filter, phase_differences)

    plt.plot(range(len(filtered_breathing)), filtered_breathing)
    plt.title("The filtered phase differences for the breathing rate")
    plt.show()

    plt.plot(range(len(filtered_heart)), filtered_heart)
    plt.title("The filtered phase differences for the heart rate")
    plt.show()

    fft_breathing = np.absolute(scipy.fft.fft(filtered_breathing))
    fft_heart = np.absolute(scipy.fft.fft(filtered_heart))

    plt.plot(range(len(fft_breathing)), fft_breathing)
    plt.title("The frequency spectrum for the breathing rate")
    plt.show()

    plt.plot(range(len(fft_heart)), fft_heart)
    plt.title("The frequency spectrum for the heart rate")
    plt.show()

    peaks_breathing, peak_height_breathing = scipy.signal.find_peaks(fft_breathing, 0.5)
    peaks_heart, peak_height_heart = scipy.signal.find_peaks(fft_heart, 40)

    breathing_rate, heart_rate = (0, 0)

    if len(peak_height_breathing["peak_heights"]) != 0:
        breathing_rate = peaks_breathing[np.argmax(peak_height_breathing["peak_heights"])]

    if len(peak_height_heart["peak_heights"]) != 0:
        heart_rate = peaks_heart[np.argmax(peak_height_heart["peak_heights"])]

    return breathing_rate, heart_rate


process_files()
