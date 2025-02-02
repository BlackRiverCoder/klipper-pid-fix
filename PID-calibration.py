import time
import shutil
from pathlib import Path
import csv
import subprocess
import pandas as pd
import numpy as np
import math
from scipy.signal import find_peaks
from datetime import datetime

home = Path.home()

# Function to read and update the PID calibration file
def update_pid_calibrate_file(old_delta, new_delta):
    with open(home / 'klipper/klippy/extras/pid_calibrate.py', 'r') as file:
        filedata = file.read()
    
    filedata = filedata.replace(f'TUNE_PID_DELTA = {old_delta}', f'TUNE_PID_DELTA = {new_delta}')
    
    with open(home / 'klipper/klippy/extras/pid_calibrate.py', 'w') as file:
        file.write(filedata)

# Function to restart Klipper using systemctl
def restart_klipper():
    subprocess.run(["sudo", "systemctl", "restart", "klipper"], check=True)

# Function to wait for the log entry to start with "Stats "
def wait_for_stats_entry():
    log_file_path = home / 'printer_data/logs/klippy.log'
    target_string = "Stats "

    print("Waiting for Klipper to finish restarting...")
    time.sleep(10)  # Add delay to wait for klippy to start logging
    while True:
        with open(log_file_path, 'r') as log_file:
            lines = log_file.readlines()
            if lines:
                # print(f"Current last line: {lines[-1].strip()}")  # Debug print statement
                if lines[-1].startswith(target_string):
                    break
        time.sleep(1)  # Wait a bit before checking again

# Function to append command to klippy.serial
def send_calibrate_command(delta, heater, target):
    with open(home / 'printer_data/comms/klippy.serial', 'a') as myFile:
        print(f"PID_CALIBRATE HEATER={heater} TARGET={target} WRITE_FILE=1", file=myFile)
    print(f"Sent calibration command for {heater} with target {target} and delta {delta}")

# Function to wait for the log entry
def wait_for_log_entry(heater):
    log_file_path = home / 'printer_data/logs/klippy.log'
    target_string = f"save_config: set [{heater}] pid_Kd ="

    print("Waiting for calibration to complete...")
    while True:
        with open(log_file_path, 'r') as log_file:
            lines = log_file.readlines()[-15:]  # Get the last 15 lines
            for line in reversed(lines):
                if line.startswith(target_string):
                    # Calibration completed, now find the last Autotune line
                    for line in reversed(lines):
                        if line.startswith("Autotune: raw"):
                            _, _, Ku_str, Tu_str, *rest = line.split()
                            Ku = float(Ku_str.split('=')[1])
                            Tu = float(Tu_str.split('=')[1])
                            return Ku, Tu
                    return None  # If no Autotune line found, return None
        time.sleep(1)  # Wait a short time before checking again

# Function to copy and parse the output file
def process_output_file(delta, heater, target):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    source_file = Path('/tmp/heattest.txt')
    destination_dir = home / 'PID' / 'Raw_Data'
    destination_file = destination_dir / f'heattest_{heater}_{target}.txt'

    print(f"Copying output file for delta {delta}...")
    destination_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(source_file, destination_file)

    # Parse the output file
    with open(destination_file, 'r') as file:
        lines = file.readlines()

    # Filter out lines that start with 'pwm:'
    filtered_lines = [line for line in lines if not line.startswith('pwm:')]

    # Write the filtered data to a new CSV file
    csv_file = destination_dir / f'parsed_heattest_{heater}_{target}_{delta}_{timestamp}.csv'
    print(f"Parsing output file for delta {delta}...")
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Temperature'])  # Write the header
        for line in filtered_lines:
            time, temp = line.strip().split()
            writer.writerow([time, temp])

    return csv_file  # Return CSV file path for further processing

# Function to find ultimate period Tu
def calculate_Tu(time, temp):
    peaks, _ = find_peaks(temp,prominence=1)
    peak_times = time[peaks[1:]]  # Ignore the first peak
    Tu = np.diff(peak_times).mean()  # Average period between peaks
    return Tu, temp[peaks[1:]]

# Function to calculate amplitude
def calculate_amplitude(temp):
    peaks, _ = find_peaks(temp,prominence=1)
    valleys, _ = find_peaks(-temp,prominence=1)

    # Ensure equal number of peaks and valleys
    num_cycles = min(len(peaks), len(valleys)) - 1
    peaks = peaks[1:]
    valleys = valleys[1:]

    # Create new lists with sequential indexing
    sequential_peaks = []
    sequential_valleys = []
    for i in range(num_cycles):
        sequential_peaks.append(temp[peaks[i]])
        sequential_valleys.append(temp[valleys[i]])

    amplitudes = []
    for i in range(num_cycles):
        amplitude = (sequential_peaks[i] - sequential_valleys[i]) / 2
        amplitudes.append(amplitude)

    average_amplitude = sum(amplitudes) / len(amplitudes)
    return average_amplitude

# Function to calculate ultimate gain Ku
def calculate_Ku(amplitude, max_power):
    return 4.0 * max_power / (math.pi * amplitude)

# Function to interpolate values
def interpolate(x1, y1, x2, y2, x):
    return y1 + (y2 - y1) / (x2 - x1) * (x - x1)

# Function to calculate PID parameters based on various tuning rules
def calculate_pid_parameters(Ku, Tu, rule):
    if rule == "Classic Ziegler-Nichols":
        Kp = 0.6 * Ku * 255
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
    elif rule == "Pessen Integral Rule":
        Kp = 0.7 * Ku * 255
        Ti = 0.4 * Tu
        Td = 0.15 * Tu
    elif rule == "Some Overshoot":
        Kp = 0.33 * Ku * 255
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
    elif rule == "No Overshoot":
        Kp = 0.2 * Ku * 255
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
    else:
        raise ValueError("Unknown rule")
    
    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd

print("\nPID Calibration Menu:")
print("1. Calibrate Extruder")
print("2. Calibrate Bed")
print("3. Exit")

while True:
    choice = input("Enter your choice (1/2/3, press Enter for Extruder): ")
    if choice == '1' or choice == '':
        heater = "extruder"
        while True:
            try:
                target_temp = int(input("Enter target temperature (default 250): ") or 250)
                break
            except ValueError:
                print("Invalid input. Please enter an integer value for the target temperature.")
        break
    elif choice == '2':
        heater = "heater_bed"
        while True:
            try:
                target_temp = int(input("Enter target temperature (default 100): ") or 100)
                break
            except ValueError:
                print("Invalid input. Please enter an integer value for the target temperature.")
        break
    elif choice == '3':
        print("Exiting...")
        exit()
    else:
        print("Invalid choice. Please try again.")

while True:
    try:
        show_self_calculated = (input("Do you want to see self-calculated values? (y/N): ") or "n")
        if show_self_calculated.lower() in ["yes", "y"]:
            show_self_calculated = True
            break
        elif show_self_calculated.lower() in ["no", "n"]:
            show_self_calculated = False
            break
        else:
            raise ValueError("Invalid input. Please enter 'y' or 'n'.")
    except ValueError as e:
        print(e)

while True:
    try:
        max_power = float(input("Enter maximum power (default 1.0): ") or 1.0)
        break
    except ValueError:
        print("Invalid input. Please enter a float value for maximum power.")

while True:
	try:
		first_delta = float(input("Enter first delta value (default 2.5): ") or 2.5)
		break
	except ValueError:
		print("Invalid input. Please enter a float value for the first delta.")

while True:
	try:
		second_delta = float(input("Enter second delta value (default 5.0, 0 to skip): ") or 5.0)
		break
	except ValueError:
		print("Invalid input. Please enter a float value for the second delta.")

# First run for first_delta
print(f"\nStarting first calibration run with TUNE_PID_DELTA = {first_delta}")
update_pid_calibrate_file(5.0, first_delta)
restart_klipper()
wait_for_stats_entry()
send_calibrate_command(first_delta, heater, target_temp)
Ku_log_first, Tu_log_first = wait_for_log_entry(heater)
csv_file_first = process_output_file(first_delta, heater, target_temp)

# Only run second calibration if second_delta is not 0
if second_delta != 0:
    print(f"\nStarting second calibration run with TUNE_PID_DELTA = {second_delta}")
    update_pid_calibrate_file(first_delta, second_delta)
    restart_klipper()
    wait_for_stats_entry()
    print("Waiting 1 minute for heater to cool down")
    time.sleep(60)
    send_calibrate_command(second_delta, heater, target_temp)
    Ku_log_second, Tu_log_second = wait_for_log_entry(heater)
    csv_file_second = process_output_file(second_delta, heater, target_temp)
else:
    csv_file_second = None

# Restore the PID calibration file to original value
update_pid_calibrate_file(second_delta if second_delta != 0 else first_delta, 5.0)
restart_klipper()

# Load the parsed CSV data
data_first = pd.read_csv(csv_file_first)

# Calculate Tu and amplitude for the first dataset
Tu_first, peaks_first = calculate_Tu(data_first['Time'], data_first['Temperature'])
amplitude_first = calculate_amplitude(data_first['Temperature'])
valleys_f, _ = find_peaks(-data_first['Temperature'], prominence=1)
valleys_first = data_first['Temperature'][valleys_f[1:]]

# Calculate ultimate gain Ku for the first dataset
Ku_first = calculate_Ku(amplitude_first, max_power)

# Define the tuning rules
rules = ["Classic Ziegler-Nichols", "Pessen Integral Rule", "Some Overshoot", "No Overshoot"]

# Prepare to write results to a text file
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
results_file = home / f'PID/results_{heater}_{target_temp}_{timestamp}.txt'


with open(results_file, 'w') as file:
    file.write(f'Target Temperature: {target_temp}\n')  # Record the target temperature

# Klipper based calculations for the first delta
if Ku_log_first is not None and Tu_log_first is not None:
    print("\n\033[91mKlipper Based Calculations:\033[0m")
    print(f'\n\033[91mTune PID Delta={first_delta}:\033[0m')
 
    with open(results_file, 'a') as file:
        file.write(f'\nTune PID Delta={first_delta}:\n')
        file.write(f'Klipper Period Tu={Tu_log_first}\n')
        file.write(f'Klipper Gain Ku={Ku_log_first}\n')
        for rule in rules:
            Kp, Ki, Kd = calculate_pid_parameters(Ku_log_first, Tu_log_first, rule)
            pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp:.3f}\n#*# pid_ki = {Ki:.3f}\n#*# pid_kd = {Kd:.3f}"
            print(f'\033[93m{rule}\033[0m PID parameters:')
            print(pid_output)
            file.write(f'{rule} PID parameters:\n')
            file.write(f'{pid_output}\n')

    # Only calculate and print interpolated PID parameters if second_delta is not 0
    if second_delta != 0 and Ku_log_second is not None and Tu_log_second is not None:
        print(f'\n\033[91mTune PID Delta={second_delta}:\033[0m')
       
        with open(results_file, 'a') as file:
            file.write(f'\nTune PID Delta={second_delta}:\n')
            file.write(f'Klipper Period Tu={Tu_log_second}\n')
            file.write(f'Klipper Gain Ku={Ku_log_second}\n')
            
            for rule in rules:
                Kp, Ki, Kd = calculate_pid_parameters(Ku_log_second, Tu_log_second, rule)
                pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp:.3f}\n#*# pid_ki = {Ki:.3f}\n#*# pid_kd = {Kd:.3f}"
                print(f'\033[93m{rule}\033[0m PID parameters:')
                print(pid_output)
                file.write(f'{rule} PID parameters:\n')
                file.write(f'{pid_output}\n')

        print(f'\n\033[91mInterpolated\033[0m PID parameters for \033[91mTUNE_PID_DELTA=0.0:\033[0m')
        with open(results_file, 'a') as file:
            file.write(f'\nInterpolated PID parameters for TUNE_PID_DELTA=0.0:\n')
            for rule in rules:
              # Interpolate Ku and Tu
              Ku_0 = interpolate(second_delta, Ku_log_second, first_delta, Ku_log_first, 0.0)
              Tu_0 = interpolate(second_delta, Tu_log_second, first_delta, Tu_log_first, 0.0)

              # Calculate final PID parameters using interpolated Ku and Tu
              Kp_0, Ki_0, Kd_0 = calculate_pid_parameters(Ku_0, Tu_0, rule)

              print(f'\033[93m{rule}\033[0m PID parameters:')
              pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp_0:.3f}\n#*# pid_ki = {Ki_0:.3f}\n#*# pid_kd = {Kd_0:.3f}"
              print(pid_output)
              file.write(f'{rule} PID parameters:\n')
              file.write(f'{pid_output}\n')

if show_self_calculated:
    # Calculate and print PID parameters for each tuning rule and the first dataset
    print('\n\033[91mSELF-CALCULATED VALUES:\033[0m')
    print(f'\n\033[91mTune PID Delta={first_delta}:\033[0m')
    print(f'Peak values: {peaks_first.tolist()}')
    print(f'Valley values: {valleys_first.tolist()}')
    print(f'Max peak value: {max(peaks_first)}')
    print(f'Min valley value: {min(valleys_first)}')
    print(f'Amplitude: {amplitude_first}')
    print(f'Ultimate Period Tu={Tu_first}')
    print(f'Ultimate Gain Ku={Ku_first}')

    with open(results_file, 'a') as file:
        file.write('\nSELF-CALCULATED VALUES:\n')
        file.write(f'\nTune PID Delta={first_delta}:\n')
        file.write(f'Peak values: {peaks_first.tolist()}\n')
        file.write(f'Valley values: {valleys_first.tolist()}\n')
        file.write(f'Max peak value: {max(peaks_first)}\n')
        file.write(f'Min valley value: {min(valleys_first)}\n')
        file.write(f'Amplitude: {amplitude_first}\n')
        file.write(f'Ultimate Period Tu={Tu_first}\n')
        file.write(f'Ultimate Gain Ku={Ku_first}\n')

        for rule in rules:
            Kp, Ki, Kd = calculate_pid_parameters(Ku_first, Tu_first, rule)
            pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp:.3f}\n#*# pid_ki = {Ki:.3f}\n#*# pid_kd = {Kd:.3f}"
            print(f'\033[93m{rule}\033[0m PID parameters:')
            print(pid_output)
            file.write(f'{rule} PID parameters:\n')
            file.write(f'{pid_output}\n')

    # Only calculate and print interpolated PID parameters if second_delta is not 0
    if second_delta != 0:
        # Load the second parsed CSV data
        data_second = pd.read_csv(csv_file_second)
        
        # Calculate Tu and amplitude for the second dataset
        Tu_second, peaks_second = calculate_Tu(data_second['Time'], data_second['Temperature'])
        amplitude_second = calculate_amplitude(data_second['Temperature'])
        
        # Calculate ultimate gain Ku for the second dataset
        Ku_second = calculate_Ku(amplitude_second, max_power)

        # Define valleys_second
        valleys_s, _ = find_peaks(-data_second['Temperature'], prominence=1)
        valleys_second = data_second['Temperature'][valleys_s[1:]]

        print(f'\n\033[91mTune PID Delta={second_delta}:\033[0m')
        print(f'Peak values: {peaks_second.tolist()}')
        print(f'Valley values: {valleys_second.tolist()}')
        print(f'Max peak value: {max(peaks_second)}')
        print(f'Min valley value: {min(valleys_second)}')
        print(f'Amplitude: {amplitude_second}')
        print(f'Ultimate Period Tu={Tu_second}')
        print(f'Ultimate Gain Ku={Ku_second}')
        
        with open(results_file, 'a') as file:
            file.write(f'\nTune PID Delta={second_delta}:\n')
            file.write(f'Peak values: {peaks_second.tolist()}\n')
            file.write(f'Valley values: {valleys_second.tolist()}\n')
            file.write(f'Max peak value: {max(peaks_second)}\n')
            file.write(f'Min valley value: {min(valleys_second)}\n')
            file.write(f'Amplitude: {amplitude_second}\n')
            file.write(f'Ultimate Period Tu={Tu_second}\n')
            file.write(f'Ultimate Gain Ku={Ku_second}\n')
            
            for rule in rules:
                Kp, Ki, Kd = calculate_pid_parameters(Ku_second, Tu_second, rule)
                pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp:.3f}\n#*# pid_ki = {Ki:.3f}\n#*# pid_kd = {Kd:.3f}"
                print(f'\033[93m{rule}\033[0m PID parameters:')
                print(pid_output)
                file.write(f'{rule} PID parameters:\n')
                file.write(f'{pid_output}\n')

        print(f'\n\033[91mInterpolated\033[0m PID parameters for \033[91mTUNE_PID_DELTA=0.0:\033[0m')
        with open(results_file, 'a') as file:
            file.write(f'\nInterpolated PID parameters for TUNE_PID_DELTA=0.0:\n')
            for rule in rules:
              # Interpolate Ku and Tu
              Ku_0 = interpolate(second_delta, Ku_second, first_delta, Ku_first, 0.0)
              Tu_0 = interpolate(second_delta, Tu_second, first_delta, Tu_first, 0.0)

              # Calculate final PID parameters using interpolated Ku and Tu
              Kp_0, Ki_0, Kd_0 = calculate_pid_parameters(Ku_0, Tu_0, rule)

              print(f'\033[93m{rule}\033[0m PID parameters:')
              pid_output = f"#*# [{heater}]\n#*# control = pid\n#*# pid_kp = {Kp_0:.3f}\n#*# pid_ki = {Ki_0:.3f}\n#*# pid_kd = {Kd_0:.3f}"
              print(pid_output)
              file.write(f'{rule} PID parameters:\n')
              file.write(f'{pid_output}\n')

print(f"Results written to {results_file}")