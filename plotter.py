import pandas as pd
import matplotlib.pyplot as plt

# Load CSV (tab-separated)
df = pd.read_csv("Test_Log_CX_XXXX.csv", sep=",", skiprows=1, header=None)

# Extract fields
cycle   = df[1]
max_f   = df[2]
targetf = df[3]

# Plot
plt.plot(cycle, max_f, label="Max Force per Cycle")
plt.plot(cycle, targetf, label="Target Force")

plt.xlabel("Cycle Count")
plt.ylabel("Force")
plt.legend()
plt.show()