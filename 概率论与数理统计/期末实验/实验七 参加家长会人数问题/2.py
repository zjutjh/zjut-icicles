import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

#np.random.seed(42)

n = 400
p = 0.8
num_simulations = 100000


Y_trials = np.random.binomial(1, p, size=(num_simulations, n))

Y_sum = Y_trials.sum(axis=1)

count_Y_ge_340 = np.sum(Y_sum >= 340)
prob_Y_ge_340_simulation = count_Y_ge_340 / num_simulations

print(f"Simulated P(Y >= 340): {prob_Y_ge_340_simulation * 100:.4f}%")
E_Y = n * p
Var_Y = n * p * (1 - p)
std_Y = np.sqrt(Var_Y)

threshold = 340
Z = (threshold - E_Y) / std_Y

prob_Y_ge_340_theoretical = 1 - norm.cdf(Z)

print(f"Theoretical P(Y >= 340) using normal approximation: {prob_Y_ge_340_theoretical * 100:.4f}%")

plt.figure(figsize=(12, 7))

count, bins, ignored = plt.hist(Y_sum, bins=200, density=True, alpha=0.6, color='skyblue', edgecolor='black', label='Simulation')

x = np.linspace(Y_sum.min(), Y_sum.max(), 1000)
pdf = norm.pdf(x, E_Y, std_Y)
plt.plot(x, pdf, 'r-', lw=2, label='Normal Approximation')

plt.axvline(x=threshold, color='green', linestyle='--', linewidth=2, label='Y = 340')

x_shade = np.linspace(threshold, Y_sum.max(), 1000)
plt.fill_between(x_shade, norm.pdf(x_shade, E_Y, std_Y), color='orange', alpha=0.5, label='P(Y ≥ 340) Area')

plt.text(threshold + 10, max(pdf)*0.6, f"Simulated P(Y ≥ 340): {prob_Y_ge_340_simulation*100:.2f}%",
         color='blue', fontsize=12, bbox=dict(facecolor='white', alpha=0.6))
plt.text(threshold + 10, max(pdf)*0.5, f"Theoretical P(Y ≥ 340): {prob_Y_ge_340_theoretical*100:.2f}%",
         color='red', fontsize=12, bbox=dict(facecolor='white', alpha=0.6))

plt.xlabel('Total Number of Students with Exactly One Parent Attending (Y)', fontsize=14)
plt.ylabel('Probability Density', fontsize=14)
plt.title('Distribution of Y and Normal Approximation', fontsize=16)
plt.legend(fontsize=12)
plt.grid(True)
plt.tight_layout()
plt.show()
