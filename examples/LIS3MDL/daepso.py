import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import svd, eigh

import math
np.math = math

class DAEPSO:
    """
    Dynamic Adaptive Elite-guided PSO
    """

    def __init__(self,
                 dim,
                 n_particles=60,
                 max_iter=1000,
                 bounds=None,
                 w_max=4.0, w_min=0.4,
                 c1=1.5,
                 c2=1.5,
                 c3=0.5,
                 elite_count=5,
                 stagnation_threshold=30,
                 levy_prob=0.18,
                 levy_beta=2.0,
                 tol = 1e-4):

        self.dim = dim
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.bounds = bounds

        self.w_max = w_max
        self.w_min = w_min
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3

        self.elite_count = elite_count
        self.stagnation_threshold = stagnation_threshold
        self.levy_prob = levy_prob
        self.levy_beta = levy_beta
        self.tol = tol
        self.best_history = []
        self.avg_history = []

    # -------------------------------------------------------------

    def _levy(self, size):
        beta = self.levy_beta
        sigma = (np.math.gamma(1 + beta) * np.sin(np.pi * beta / 2) /
                 (np.math.gamma((1 + beta) / 2) * beta * 2 ** ((beta - 1) / 2))) ** (1 / beta)
        u = np.random.normal(0, sigma, size)
        v = np.random.normal(0, 1, size)
        return u / np.abs(v) ** (1 / beta)

    def _apply_bounds(self, x):
        if self.bounds is None:
            return x
        low = np.array([b[0] for b in self.bounds])
        high = np.array([b[1] for b in self.bounds])
        return np.clip(x, low, high)

    # -------------------------------------------------------------

    def optimize(self, objective, verbose=True, plot=True):

        # Initialization
        if self.bounds is None:
            X = np.random.uniform(-1, 1, (self.n_particles, self.dim))
            span = np.ones(self.dim) * 2.0
        else:
            low = np.array([b[0] for b in self.bounds])
            high = np.array([b[1] for b in self.bounds])
            span = high - low
            X = low + np.random.rand(self.n_particles, self.dim) * span

        V = np.random.uniform(-0.1, 0.1, X.shape)

        Pbest = X.copy()
        Pbest_f = np.array([objective(x) for x in X])

        g_idx = np.argmin(Pbest_f)
        Gbest = Pbest[g_idx].copy()
        Gbest_f = Pbest_f[g_idx]

        stagnation = 0

        # ---------------------------------------------------------
        for it in range(1, self.max_iter + 1):

            progress = it / self.max_iter
            w = self.w_max - (self.w_max - self.w_min) * progress

            # Adaptive velocity limit (based on swarm spread)
            Vmax = 0.5 * np.std(X, axis=0) + 1e-5

            elite_idx = np.argsort(Pbest_f)[:self.elite_count]
            elites = Pbest[elite_idx]

            improved = False

            for i in range(self.n_particles):

                r1, r2, r3 = np.random.rand(3)

                elite = elites[np.random.randint(len(elites))]
                elite_dir = elite - Gbest  # directional guidance

                V[i] = (
                    w * V[i]
                    + self.c1 * r1 * (Pbest[i] - X[i])
                    + self.c2 * r2 * (Gbest - X[i])
                    + self.c3 * r3 * elite_dir
                )

                V[i] = np.clip(V[i], -Vmax, Vmax)
                X_new = X[i] + V[i]

                # Lévy jump (absolute, scale-aware)
                if stagnation > self.stagnation_threshold and np.random.rand() < self.levy_prob:
                    X_new += 0.05 * self._levy(self.dim) * span

                X_new = self._apply_bounds(X_new)
                f_new = objective(X_new)

                if f_new < Pbest_f[i]:
                    Pbest[i] = X_new
                    Pbest_f[i] = f_new

                    if f_new < Gbest_f:
                        Gbest = X_new.copy()
                        Gbest_f = f_new
                        improved = True

                X[i] = X_new

            stagnation = 0 if improved else stagnation + 1

            self.best_history.append(Gbest_f)
            self.avg_history.append(np.mean(Pbest_f))
            
            if Gbest_f < self.tol:
                break

            if verbose and it % 50 == 0:
                print(f"DAEPSO iter {it:5d} | Best {Gbest_f:.3e} | Avg {self.avg_history[-1]:.3e}")

        if plot:
            self._plot()

        return Gbest, Gbest_f, {
            "best": self.best_history,
            "avg": self.avg_history
        }

    # -------------------------------------------------------------

    def _plot(self):
        plt.figure(figsize=(9, 5))
        plt.semilogy(self.best_history, label="Best")
        plt.semilogy(self.avg_history, "--", label="Average")
        plt.xlabel("Iteration")
        plt.ylabel("Fitness")
        plt.title("DAEPSO Convergence")
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()

"""
# ──────────────────────────────────────────────────────────────
# Generate noisy ellipsoid points
# ──────────────────────────────────────────────────────────────

def generate_noisy_points(a, b, c, x0, V, num_points=1200, noise_level=0.05):
    u = np.linspace(0, 2*np.pi, int(np.sqrt(num_points)))
    v = np.linspace(-np.pi/2, np.pi/2, int(np.sqrt(num_points)))
    u, v = np.meshgrid(u, v)

    x_sphere = np.cos(u) * np.cos(v)
    y_sphere = np.sin(u) * np.cos(v)
    z_sphere = np.sin(v)

    x_ell = a * x_sphere
    y_ell = b * y_sphere
    z_ell = c * z_sphere

    pts = np.vstack((x_ell.ravel(), y_ell.ravel(), z_ell.ravel()))
    rotated = V @ pts
    points = rotated + x0[:, np.newaxis]

    noise_scale = noise_level * min(a, b, c)
    points += noise_scale * np.random.randn(*points.shape)

    return points

# ──────────────────────────────────────────────────────────────
# Ellipsoid cost function
# ──────────────────────────────────────────────────────────────

def ellipsoid_cost(p, points):
    A, B, C, D, E, F, G, H, I, J = p
    X, Y, Z = points

    quad = (
        A*X**2 + B*Y**2 + C*Z**2 +
        D*X*Y + E*X*Z + F*Y*Z +
        G*X + H*Y + I*Z + J
    )

    Q = np.array([
        [A, D/2, E/2],
        [D/2, B, F/2],
        [E/2, F/2, C]
    ], dtype=np.float64)

    pts = np.vstack((X, Y, Z))
    Qp = Q @ pts
    norms_sq = np.sum(pts * Qp, axis=0)
    norms = np.sqrt(np.maximum(norms_sq, 1e-12))
    safe_div = np.where(norms > 1e-6, 1.0 / norms, 1.0)

    resid = quad * safe_div
    resid = np.clip(resid, -1e6, 1e6)

    return np.mean(resid**2) + 1e-6 * np.sum(p**2)

def ellipsoid_from_quadratic(coeffs):

    A, B, C, D, E, F, G, H, I, J = coeffs

    # Quadratic form matrix
    T = np.array([
        [2*A, D,   E],
        [D,   2*B, F],
        [E,   F,   2*C]
    ])

    # Ellipsoid center
    center = -np.linalg.solve(T, np.array([G, H, I]))

    # Normalize quadratic form
    Q = T / 2.0

    # Eigen-decomposition
    vals, vecs = eigh(Q)
    idx = np.argsort(vals)[::-1]
    eigenvalues = vals[idx]
    axes = vecs[:, idx]

    # Constant term after translation
    x0, y0, z0 = center
    J0 = (
        A*x0**2 + B*y0**2 + C*z0**2 +
        D*x0*y0 + E*x0*z0 + F*y0*z0 +
        G*x0 + H*y0 + I*z0 + J
    )

    # Ellipsoid validity check
    if np.all(eigenvalues > 0) and J0 < 0:
        radii = np.sqrt(-J0 / eigenvalues)
        valid = True
    else:
        radii = (np.nan, np.nan, np.nan)
        valid = False

    return {
        "center": center,
        "radii": radii,
        "axes": axes,
        "eigenvalues": eigenvalues,
        "J0": J0,
        "valid": valid
    }

# ──────────────────────────────────────────────────────────────
# Example usage
# ──────────────────────────────────────────────────────────────


if __name__ == "__main__":

    # Ground truth ellipsoid
    a, b, c = 4.00, 5.50, 7.00
    x0 = np.array([12.0, -8.0, 5.0])
    V = np.array([
        [0.36, -0.48, 0.80],
        [0.80,  0.60, 0.00],
        [-0.48, 0.64, 0.60]
    ])

    points = generate_noisy_points(a, b, c, x0, V, num_points=1600, noise_level=0.03)
    X, Y, Z = points

    # Objective function binding points
    def objective(p):
        return ellipsoid_cost(p, points)

    # Parameter bounds
    dim = 10
    bounds = [
        (-0.1, 0.1),  # A
        (-0.1, 0.1),  # B
        (-0.1, 0.1),  # C
        (-0.1, 0.1),  # D
        (-0.1, 0.1),  # E
        (-0.1, 0.1),  # F
        (-10, 10),      # G
        (-10, 10),      # H
        (-10, 10),      # I
        (-10, 10)       # J
    ]

    # Run DAEPSO
    optimizer = DAEPSO(
        dim=dim,
        n_particles=180,
        max_iter=3000,
        bounds=bounds,
        elite_count=6,
        stagnation_threshold=10
    )

    best_p, best_cost, history = optimizer.optimize(objective, verbose=True)


    print("\n" + "="*70)
    print("Ellipsoid fit complete")
    print(f"Final cost: {best_cost:.6e}")
    coeffs = []
    for name, val in zip("A B C D E F G H I J".split(), best_p):
        coeffs.append(val)
        print(f"  {name} = {val:+.6e}")
        
    vals = ellipsoid_from_quadratic(coeffs)
    print("="*70)
    
"""
#if __name__ == "__main__":
#    def rastrigin(x):
#        x = np.asarray(x)
#        return 10 * len(x) + np.sum(x**2 - 10 * np.cos(2 * np.pi * x))
#
#    def ackley(x):
#        x = np.asarray(x)
#        d = len(x)
#        return (
#            -20.0 * np.exp(-0.2 * np.sqrt(np.sum(x**2) / d))
#            - np.exp(np.sum(np.cos(2 * np.pi * x)) / d)
#            + 20 + np.e)
#    def rosenbrock(x):
#        x = np.asarray(x)
#        return np.sum(
#            100.0 * (x[1:] - x[:-1]**2)**2 +
#            (1.0 - x[:-1])**2)

    
#    dim = 10
#    bounds = [(-5.768, 5.768)] * dim


 #   optimizer = DAEPSO(
 #       dim=dim,
 #       n_particles=100,
 #       max_iter=5000,
 #       bounds=bounds,
 #       stagnation_threshold=5
 #   )
    
 #   best_solution, best_fitness, history = optimizer.optimize(ackley, verbose=True)
 #   
 #   print("\n" + "="*70)
 #   print(f"Best solution found: {best_solution.round(6)}")
 #   print(f"Best fitness: {best_fitness:.8f}")
 #   print("="*70)
