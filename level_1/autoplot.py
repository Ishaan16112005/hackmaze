import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Check if Verilog has generated the data file
data_file = 'histogram_data.txt'
if not os.path.exists(data_file):
    print(f"Error: {data_file} not found. Did you run 'vvp sim.vvp' first?")
    exit()

# 2. Read the data automatically!
with open(data_file, 'r') as f:
    lines = f.readlines()

# Convert the comma-separated text into lists of numbers
latency_counts = [int(x) for x in lines[0].strip().split(',')]
jitter_counts = [int(x) for x in lines[1].strip().split(',')]

# 3. Setup the Labels
latency_labels = ['0-10', '11-20', '21-50', '51-100', '101-250', '251-500', '501-1k', '>1000']
jitter_labels = ['0-5', '6-10', '11-20', '21-50', '51-100', '101-200', '201-500', '>500']

plt.style.use('dark_background')

# ======================================================================
# 4. Plot P99 Latency Histogram
# ======================================================================
plt.figure(figsize=(10, 5))
plt.bar(latency_labels, latency_counts, color='cyan', edgecolor='white', zorder=2)
plt.title('AutoTimeMon-TSN: P99 Latency Distribution', fontsize=16, fontweight='bold')
plt.xlabel('Latency (Clock Cycles)', fontsize=12)
plt.ylabel('Number of Frames', fontsize=12)

max_lat = max(latency_counts) if max(latency_counts) > 0 else 1
plt.yticks(np.arange(0, max_lat + 2, max(1, max_lat//5))) 
plt.grid(axis='y', linestyle='--', alpha=0.5, zorder=1)
plt.savefig('latency_histogram.png', bbox_inches='tight', dpi=150)
print("SUCCESS: Saved latency_histogram.png")
plt.close()

# ======================================================================
# 5. Plot Jitter Histogram
# ======================================================================
plt.figure(figsize=(10, 5))
plt.bar(jitter_labels, jitter_counts, color='magenta', edgecolor='white', zorder=2)
plt.title('AutoTimeMon-TSN: Jitter Distribution', fontsize=16, fontweight='bold')
plt.xlabel('Jitter (Clock Cycles)', fontsize=12)
plt.ylabel('Number of Frames', fontsize=12)

max_jit = max(jitter_counts) if max(jitter_counts) > 0 else 1
plt.yticks(np.arange(0, max_jit + 2, max(1, max_jit//5)))
plt.grid(axis='y', linestyle='--', alpha=0.5, zorder=1)
plt.savefig('jitter_histogram.png', bbox_inches='tight', dpi=150)
print("SUCCESS: Saved jitter_histogram.png")
plt.close()
