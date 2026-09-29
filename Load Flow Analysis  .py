# Load Flow Analysis using Gauss-Seidel Method
# Simple educational simulation

import cmath

# -----------------------------
# System Data
# -----------------------------

# Bus 1 = Slack bus
# Bus 2 = PQ bus
# Bus 3 = PQ bus

V1 = 1.05 + 0j

# Load at buses in per unit
P2 = -0.50
Q2 = -0.20

P3 = -0.60
Q3 = -0.25

# Bus admittance matrix (Y-bus)
Y = [
    [10 - 20j, -5 + 10j, -5 + 10j],
    [-5 + 10j, 10 - 20j, -5 + 10j],
    [-5 + 10j, -5 + 10j, 10 - 20j]
]

# Initial voltages
V = [
    V1,
    1 + 0j,
    1 + 0j
]

# -----------------------------
# Gauss-Seidel Calculation
# -----------------------------

max_iterations = 20
tolerance = 0.00001

for iteration in range(max_iterations):

    old_V2 = V[1]
    old_V3 = V[2]

    # Update Bus 2
    sum_yv = Y[1][0] * V[0] + Y[1][2] * V[2]

    V[1] = (
        (P2 - 1j * Q2) / V[1].conjugate()
        - sum_yv
    ) / Y[1][1]

    # Update Bus 3
    sum_yv = Y[2][0] * V[0] + Y[2][1] * V[1]

    V[2] = (
        (P3 - 1j * Q3) / V[2].conjugate()
        - sum_yv
    ) / Y[2][2]

    # Check convergence
    error = max(
        abs(V[1] - old_V2),
        abs(V[2] - old_V3)
    )

    if error < tolerance:
        break


# -----------------------------
# Display Results
# -----------------------------

print("======================================")
print("        LOAD FLOW ANALYSIS")
print("======================================")

print("Method       : Gauss-Seidel")
print("Iterations   :", iteration + 1)

print("\nBus Voltage Results")
print("--------------------------------------")

for i, voltage in enumerate(V):
    magnitude = abs(voltage)
    angle = cmath.phase(voltage) * 180 / 3.141592653589793

    print(
        f"Bus {i + 1}: "
        f"Voltage = {magnitude:.4f} pu, "
        f"Angle = {angle:.2f}°"
    )

print("\nLoad Data")
print("--------------------------------------")
print(f"Bus 2: P = {-P2:.2f} pu, Q = {-Q2:.2f} pu")
print(f"Bus 3: P = {-P3:.2f} pu, Q = {-Q3:.2f} pu")

print("\nLoad Flow Status")
print("--------------------------------------")

if error < tolerance:
    print("Status: CONVERGED")
    print("Power system load flow solved successfully.")
else:
    print("Status: NOT CONVERGED")
