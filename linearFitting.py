import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import seaborn as sns

def parse_file(file_path):
    with open(file_path, 'r') as file:
        data = file.readlines()

    sizes = []
    rtts = []

    for line in data:
        match = re.search(r'size = (\d+) bytes, .* RTT = ([\d.]+) ms', line)
        if match:
            size = int(match.group(1))
            rtt = float(match.group(2))
            sizes.append(size)
            rtts.append(rtt)

    return sizes, rtts


def average_rtts(sizes, rtts):
    avg_sizes = []
    avg_rtts = []
    unique_sizes = sorted(set(sizes))

    for size in unique_sizes:
        rtt_values = [rtts[i] for i in range(len(sizes)) if sizes[i] == size]
        for i in range(0, len(rtt_values), 5):
            avg_sizes.append(size)
            avg_rtts.append(np.mean(rtt_values[i:i + 5]))

    return avg_sizes, avg_rtts


def plot_fitting(avg_sizes, avg_rtts):
    sns.set(style="whitegrid")

    slope, intercept, r_value, p_value, std_err = linregress(avg_sizes, avg_rtts)

    plt.scatter(avg_sizes, avg_rtts, color='blue', label='Average RTT', edgecolors='black', s=50)

    fit_line = [slope * size + intercept for size in avg_sizes]
    plt.plot(avg_sizes, fit_line, color='red', linestyle='--', linewidth=2,
             label=f'Fitted Line: y={slope:.4f}x+{intercept:.4f}')

    plt.xlabel('Message Size (bytes)', fontsize=12)
    plt.ylabel('RTT (ms)', fontsize=12)
    plt.title('Message Size vs RTT with Linear Fitting', fontsize=14, fontweight='bold')
    plt.legend(loc='upper left', fontsize=10)

    plt.grid(True)

    plt.tight_layout()
    plt.show()

    return slope, intercept


if __name__ == '__main__':
    file_path = "result.txt"
    sizes, rtts = parse_file(file_path)
    avg_sizes, avg_rtts = average_rtts(sizes, rtts)
    slope, intercept = plot_fitting(avg_sizes, avg_rtts)
    print(f"slope: {slope:.10f} bytes/ms, intercept: {intercept:.3f} ms")
