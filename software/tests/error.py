import numpy as np
from scipy import optimize
import matplotlib.pyplot as plt

# Measured data points (voltage, dBm)
data_points = [
    (0.358, -56.00), (0.361, -53.92), (0.364, -52.20), (0.369, -50.10),
    (0.376, -48.21), (0.388, -46.23), (0.403, -44.23), (0.421, -42.28),
    (0.443, -40.32), (0.469, -38.29), (0.505, -36.32), (0.555, -34.32),
    (0.610, -32.36), (0.665, -30.46), (0.732, -28.59), (0.792, -26.74),
    (0.848, -24.89), (0.902, -23.69), (0.933, -22.69), (0.999, -20.60),
    (1.074, -18.54), (1.139, -16.53), (1.210, -14.50), (1.287, -12.48),
    (1.359, -10.48), (1.438, -8.47), (1.506, -6.46), (1.578, -4.47),
    (1.636, -2.48), (1.688, -0.50), (1.741, 1.44), (1.782, 3.33),
    (1.816, 5.17), (1.872, 7.02), (1.909, 8.54)
]

voltage = np.array([x[0] for x in data_points])
dbm_actual = np.array([x[1] for x in data_points])

# Optimized 7-region breakpoints (voltage thresholds)
breakpoints = [0.358, 0.40, 0.47, 0.60, 0.90, 1.20, 1.60, 1.909]

def piecewise_linear(x, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6, a7, b7):
    return np.piecewise(x, [
        x <= breakpoints[1],
        (breakpoints[1] < x) & (x <= breakpoints[2]),
        (breakpoints[2] < x) & (x <= breakpoints[3]),
        (breakpoints[3] < x) & (x <= breakpoints[4]),
        (breakpoints[4] < x) & (x <= breakpoints[5]),
        (breakpoints[5] < x) & (x <= breakpoints[6]),
        x > breakpoints[6]
    ], [
        lambda x: a1*x + b1,
        lambda x: a2*x + b2,
        lambda x: a3*x + b3,
        lambda x: a4*x + b4,
        lambda x: a5*x + b5,
        lambda x: a6*x + b6,
        lambda x: a7*x + b7
    ])

# Initial parameter guesses based on data trends
initial_guess = [
    160, -114,  # Region 1
    100, -80,   # Region 2
    50, -60,    # Region 3
    30, -50,    # Region 4
    25, -40,    # Region 5
    35, -55,    # Region 6
    38, -58     # Region 7
]

# Constrained optimization to keep errors < 0.5 dBm
bounds = ([120, -120, 80, -90, 40, -70, 20, -60, 20, -50, 30, -60, 35, -65],
          [200, -100, 120, -70, 60, -50, 40, -40, 30, -30, 40, -50, 40, -50])

popt, _ = optimize.curve_fit(piecewise_linear, voltage, dbm_actual, 
                            p0=initial_guess, bounds=bounds, maxfev=10000)

def voltage_to_dbm(v):
    if v <= breakpoints[1]: return popt[0]*v + popt[1]
    elif v <= breakpoints[2]: return popt[2]*v + popt[3]
    elif v <= breakpoints[3]: return popt[4]*v + popt[5]
    elif v <= breakpoints[4]: return popt[6]*v + popt[7]
    elif v <= breakpoints[5]: return popt[8]*v + popt[9]
    elif v <= breakpoints[6]: return popt[10]*v + popt[11]
    else: return popt[12]*v + popt[13]

# Verification
dbm_calc = np.array([voltage_to_dbm(v) for v in voltage])
errors = dbm_calc - dbm_actual
max_error = np.max(np.abs(errors))

if max_error > 0.5:
    print("Warning: Maximum error exceeds 0.5 dBm threshold!")
else:
    print(f"Success! Maximum error = {max_error:.3f} dBm")

# Plotting
plt.figure(figsize=(14, 7))
plt.plot(voltage, dbm_actual, 'ko', label='Actual Data', markersize=6)
voltage_fine = np.linspace(voltage.min(), voltage.max(), 500)
plt.plot(voltage_fine, piecewise_linear(voltage_fine, *popt), 'r-', label='7-Region Fit')
plt.xlabel('Voltage (V)')
plt.ylabel('dBm')
plt.title(f'7-Region Piecewise Linear Fit (Max Error = {max_error:.3f} dBm)')
plt.grid(True)

# Add error bars
plt.errorbar(voltage, dbm_calc, yerr=np.abs(errors), fmt='none', 
             ecolor='blue', capsize=3, label='Error')

# Mark region boundaries
for bp in breakpoints[1:-1]:
    plt.axvline(x=bp, color='gray', linestyle='--', alpha=0.3)

plt.legend()
plt.show()

# Print coefficients for C implementation
print("\nC Implementation Coefficients:")
print("float voltage_to_dbm(float voltage) {")
print(f"    if (voltage <= {breakpoints[1]:.3f}f) return {popt[0]:.5f}f * voltage + {popt[1]:.5f}f;")
print(f"    else if (voltage <= {breakpoints[2]:.3f}f) return {popt[2]:.5f}f * voltage + {popt[3]:.5f}f;")
print(f"    else if (voltage <= {breakpoints[3]:.3f}f) return {popt[4]:.5f}f * voltage + {popt[5]:.5f}f;")
print(f"    else if (voltage <= {breakpoints[4]:.3f}f) return {popt[6]:.5f}f * voltage + {popt[7]:.5f}f;")
print(f"    else if (voltage <= {breakpoints[5]:.3f}f) return {popt[8]:.5f}f * voltage + {popt[9]:.5f}f;")
print(f"    else if (voltage <= {breakpoints[6]:.3f}f) return {popt[10]:.5f}f * voltage + {popt[11]:.5f}f;")
print(f"    else return {popt[12]:.5f}f * voltage + {popt[13]:.5f}f;")
print("}")
