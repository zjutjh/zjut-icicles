import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as st

#np.random.seed(42)


n = 400
p0, p1, p2 = 0.05, 0.8, 0.15
num_simulations = 100000

E_Xk = 1.1
Var_Xk = 0.19
E_X = n * E_Xk
Var_X = n * Var_Xk
Std_X = np.sqrt(Var_X)

X_samples = np.random.choice([0, 1, 2], size=(num_simulations, n), p=[p0, p1, p2])
X_sums = X_samples.sum(axis=1)

empirical_prob = np.mean(X_sums > 450)

Z_cut = (450 - E_X)/Std_X
theoretical_prob = 1 - st.norm.cdf(Z_cut)

print("Empirical P(X > 450):", empirical_prob)
print("Theoretical (Normal Approx) P(X > 450):", theoretical_prob)

plt.figure(figsize=(10,6))

count, bins, patches = plt.hist(X_sums, bins=400, density=True, alpha=0.7, color='g', edgecolor='black')

x = np.linspace(min(X_sums), max(X_sums), 1000)
pdf = st.norm.pdf(x, loc=E_X, scale=Std_X)
plt.plot(x, pdf, 'r', linewidth=2, label='Normal approximation')


plt.axvline(x=450, color='blue', linestyle='--', linewidth=2, label='x=450')


plt.text(450+5, max(pdf)*0.8,
         f"Empirical P(X > 450) = {empirical_prob:.4f}\nTheoretical P(X > 450) = {theoretical_prob:.4f}",
         fontsize=12, color='blue')

plt.title('Distribution of X and Normal Approximation', fontsize=16)
plt.xlabel('X value (Total number of parents)', fontsize=14)
plt.ylabel('Density', fontsize=14)
plt.legend(fontsize=12)
plt.grid(True)
plt.tight_layout()
plt.show()
