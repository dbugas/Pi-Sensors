import numpy as np
from scipy.linalg import svd, eigh
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import sys
import termios
import tty
import select

from pylisxmdl import pylisxmdl, FullScale, ODR
import daepso as daepso_module   

"""
R = 
-0.68512761   0.72569165   0.06302210
 0.72663274  -0.12893599   0.67432876
-0.05103928   0.98964828  -0.13413153

S = 
2.17548075   0.00000000   0.00000000
0.00000000   2.12745755   0.00000000
0.00000000   0.00000000   2.08222143

x0 = [0.042518, 0.016366, 0.156674]

"""
# =============================================
# Configuration
# =============================================
GENERATE_RANDOM_ELLIPSOID = False     # use real data. 
SAVE_DATA = True                      # save real data
LOAD_DATA = False                     # Load real data
filepath = ""                         # path for loading data
# =============================================
# Main execution
# =============================================
def main():
    if GENERATE_RANDOM_ELLIPSOID:
        try:
            # Step 1: Generate a valid ellipsoid
            params, x0, V, lambda_, a, b, c, J0 = generate_ellipsoid()
        except RuntimeError as e:
            print("Error generating ellipsoid:", e)
            return
         
        # Step 2: Generate noisy points
        points = generate_noisy_points(a, b, c, x0, V)

        # Step 3: Fit ellipsoid
        coeffs, coeffs_svd, x0_fit, lambda_fit, V_fit, a_fit, b_fit, c_fit, J0_fit = fit_ellipsoid(points)

        rms = compute_rms(coeffs, points)

        # Step 5: Print results
        print_results(params, coeffs, lambda_, lambda_fit,
                      a, b, c, a_fit, b_fit, c_fit, J0, J0_fit, rms, V, V_fit, x0, x0_fit, GENERATE_RANDOM_ELLIPSOID)

        # Step 6: Plot
        x0_plot = np.zeros(3)  # just for plotting consistency with original
        plot_ellipsoids(points, x0_plot, x0_fit, coeffs, a_fit, b_fit, c_fit, V_fit)

        # Step 7: Transform to sphere
        sphere_points = transform_to_sphere(points, x0_fit, V_fit, a_fit, b_fit, c_fit)
        plot_sphere_points(sphere_points)

    else:
        # === REAL DATA MODE ===

        if(LOAD_DATA):
            points = np.loadtxt(
                filepath,
                delimiter=',',
                skiprows=1,          
                dtype=np.float64)
        else:
            points = Get_data()
            if(SAVE_DATA):
                np.savetxt(
                    "mag_xyz.csv",
                    points,
                    delimiter=',',
                    header='X_gauss,Y_gauss,Z_gauss',
                    comments='',
                    fmt='%.10f')

        print("Fitting Data...")
        coeffs, coeffs_svd, x0_fit, lambda_fit, V_fit, a_fit, b_fit, c_fit, J0_fit = fit_ellipsoid(points)

        rms = compute_rms(coeffs, points)

        print_results(np.zeros(10), coeffs, np.zeros(3), lambda_fit,
                      0, 0, 0, a_fit, b_fit, c_fit, 0, J0_fit, rms, 
                      np.zeros([3,3]), V_fit, np.zeros([3]), x0_fit, GENERATE_RANDOM_ELLIPSOID)

        plot_ellipsoids(points, np.zeros(3), x0_fit, coeffs, a_fit, b_fit, c_fit, V_fit)

        sphere_points = transform_to_sphere(points, x0_fit, V_fit, a_fit, b_fit, c_fit)
        plot_sphere_points(sphere_points)


# =============================================
# Core functions
# =============================================

def generate_ellipsoid(max_attempts=1000):
    """
    More stable version - tries to avoid infs/NaNs and ill-conditioned cases
    """
    attempt = 0
    while attempt < max_attempts:
        attempt += 1
        
        # Reasonable positive definite tendency
        A = np.random.uniform(-2.0, 2.0)
        B = np.random.uniform(-2.0, 2.0)
        C = np.random.uniform(-2.0, 2.0)
        
        # Controlled cross terms
        scale_cross = 0.35 * min(A, B, C)
        D = np.random.uniform(-scale_cross, scale_cross)
        E = np.random.uniform(-scale_cross, scale_cross)
        F = np.random.uniform(-scale_cross, scale_cross)
        
        # Moderate offsets
        G = np.random.uniform(-1.2, 1.2)
        H = np.random.uniform(-1.2, 1.2)
        I = np.random.uniform(-1.2, 1.2)
        J = np.random.uniform(-2.5, 2.5)
        
        T = np.array([
            [2*A, D,   E],
            [D,   2*B, F],
            [E,   F,   2*C]
        ])
        
        # Skip very ill-conditioned matrices
        if np.linalg.cond(T) > 400:
            continue
            
        linear = np.array([G, H, I])
        
        try:
            x0 = -np.linalg.solve(T, linear)
        except np.linalg.LinAlgError:
            continue
            
        Q = T / 2.0
        vals, vecs = eigh(Q)  # more stable for symmetric matrices
        lambda_ = np.sort(vals)[::-1]
        
        if not np.all(lambda_ > 1e-8):
            continue
            
        J0 = (A*x0[0]**2 + B*x0[1]**2 + C*x0[2]**2 +
              D*x0[0]*x0[1] + E*x0[0]*x0[2] + F*x0[1]*x0[2] +
              G*x0[0] + H*x0[1] + I*x0[2] + J)
        
        if J0 >= -1e-5:
            continue
            
        try:
            a = np.sqrt(-J0 / lambda_[0])
            b = np.sqrt(-J0 / lambda_[1])
            c = np.sqrt(-J0 / lambda_[2])
            
            if not all(np.isfinite([a, b, c])) or max(a, b, c) > 40:
                continue
                
        except (ZeroDivisionError, ValueError, FloatingPointError):
            continue
            
        # Success!
        print(f"Generated valid ellipsoid after {attempt} attempts")
        params = np.array([A, B, C, D, E, F, G, H, I, J])
        idx = np.argsort(vals)[::-1]
        V = vecs[:, idx]
        
        return params, x0, V, lambda_, a, b, c, J0
    
    raise RuntimeError(f"Failed to generate valid ellipsoid after {max_attempts} attempts")


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


def fit_ellipsoid(points):
    X, Y, Z = points
    n = len(X)

    D = np.c_[X**2, Y**2, Z**2, X*Y, X*Z, Y*Z, X, Y, Z, np.ones(n)]

    # SVD initial guess
    _, _, Vt = svd(D, full_matrices=False)
    coeffs = Vt[-1, :]
    coeffs /= np.linalg.norm(coeffs[:6])

    # Sign check
    T = np.array([[2*coeffs[0], coeffs[3], coeffs[4]],
                  [coeffs[3], 2*coeffs[1], coeffs[5]],
                  [coeffs[4], coeffs[5], 2*coeffs[2]]])

    if np.all(eigh(T/2)[0] < 0):
        coeffs = -coeffs

    coeffs_svd = coeffs.copy()
    
    # Scale so that the quadratic part has reasonable magnitude
    quad_norm = np.linalg.norm(coeffs[:6])
    if quad_norm > 1e-8:
        coeffs /= quad_norm
    else:
        # Very degenerate case - add small perturbation
        coeffs[:6] += 1e-6 * np.random.randn(6)
        coeffs /= np.linalg.norm(coeffs[:6])
    

    def ellipsoid_cost(p, points):
        A, B, C, D, E, F, G, H, I, J = p
        X, Y, Z = points

        # compute f(x)
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

        # r = f(x)/sqrt(x^TQx)
        resid = quad * safe_div
        resid = np.clip(resid, -1e6, 1e6)

        return np.mean(resid**2) + 1e-5 * np.sum(p**2)
    
    # Objective function binding points
    def objective(p):
        return ellipsoid_cost(p, points)
    dim = 10
    bounds = [
        (-5, 5),  # A
        (-5, 5),  # B
        (-5, 5),  # C
        (-5, 5),  # D
        (-5, 5),  # E
        (-5, 5),  # F
        (-30, 30),      # G
        (-30, 30),      # H
        (-30, 30),      # I
        (-40, 40)       # J
    ]
    
    # Run DAEPSO
    optimizer = daepso_module.DAEPSO(
        dim=dim,
        n_particles=100,
        max_iter=3000,
        bounds=bounds,
        elite_count=30,
        stagnation_threshold=20
    )

    best_p, best_cost, history = optimizer.optimize(objective, verbose=True)
    res = []
    for name, val in zip("A B C D E F G H I J".split(), best_p):
        res.append(val)
    coeffs = np.array(res)
    coeffs /= np.linalg.norm(coeffs[:6])

    # Final sign check
    T = np.array([[2*coeffs[0], coeffs[3], coeffs[4]],
                  [coeffs[3], 2*coeffs[1], coeffs[5]],
                  [coeffs[4], coeffs[5], 2*coeffs[2]]])
    if np.all(eigh(T/2)[0] < 0):
        coeffs = -coeffs

    # Choose better solution
    rms_svd = compute_rms(coeffs_svd, points)
    rms_ls  = compute_rms(coeffs, points)
    print(f"rms SVD {rms_svd}")
    print(f"rms DAEPSO {rms_ls}")
    if rms_svd < rms_ls or math.isnan(rms_ls):
        coeffs = coeffs_svd

    A,B,C,D,E,F,G,H,I,J = coeffs

    T = np.array([[2*A, D, E], [D, 2*B, F], [E, F, 2*C]])
    x0_fit = -np.linalg.solve(T, np.array([G, H, I]))

    Q = T / 2
    vals, vecs = eigh(Q)
    lambda_fit = np.sort(vals)[::-1]
    idx = np.argsort(vals)[::-1]
    V_fit = vecs[:, idx]

    J0_fit = (A*x0_fit[0]**2 + B*x0_fit[1]**2 + C*x0_fit[2]**2 +
              D*x0_fit[0]*x0_fit[1] + E*x0_fit[0]*x0_fit[2] + F*x0_fit[1]*x0_fit[2] +
              G*x0_fit[0] + H*x0_fit[1] + I*x0_fit[2] + J)

    if np.all(lambda_fit > 0) and J0_fit < 0:
        a_fit = np.sqrt(-J0_fit / lambda_fit[0])
        b_fit = np.sqrt(-J0_fit / lambda_fit[1])
        c_fit = np.sqrt(-J0_fit / lambda_fit[2])
    else:
        print("Warning: Fitted quadric is not a valid ellipsoid!")
        a_fit = b_fit = c_fit = np.nan

    return coeffs, coeffs_svd, x0_fit, lambda_fit, V_fit, a_fit, b_fit, c_fit, J0_fit


def compute_rms(coeffs, points):
    A,B,C,D,E,F,G,H,I,J = coeffs
    X,Y,Z = points

    quad = A*X**2 + B*Y**2 + C*Z**2 + D*X*Y + E*X*Z + F*Y*Z + G*X + H*Y + I*Z + J

    Q = np.array([[A, D/2, E/2],
                  [D/2, B, F/2],
                  [E/2, F/2, C]])

    norm = np.sqrt(np.sum(points * (Q @ points), axis=0) + 1e-20)
    dist = np.abs(quad) / norm

    return np.sqrt(np.mean(dist**2))


def print_results(params, coeffs, lambda_orig, lambda_fit,
                  a, b, c, a_fit, b_fit, c_fit, J0, J0_fit, rms, 
                  V, V_fit, x0, x0_fit, is_random):
    print("\n" + "═"*30)
    print('f(x) = Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Iz + J')
    if is_random:
        print("FITTED ELLIPSOID PARAMETERS:\n\n")
        print("  ORIGINAL  |  FITTED")
        print("─"*30)
        names = "ABCDEFGHIJ"
        for n, po, pf in zip(names, params, coeffs):
            print(f"{n} = {po:12.5f}  |  {n}_fit = {pf:12.5f}")
        print(f"J0   = {J0:12.5f}  |  J0_fit   = {J0_fit:12.5f}")
        print("\nCenter (x0):")
        print(f"  Original: {x0.round(5)}")
        print(f"  Fitted:   {x0_fit.round(5)}")
        print("\nSemi-axes:")
        print(f"  Original: a={a:.4f}  b={b:.4f}  c={c:.4f}")
        print(f"  Fitted:   a={a_fit:.4f}  b={b_fit:.4f}  c={c_fit:.4f}\n")
        print("\n" + "═"*30)
        print("\nUnit transformation: S*R^T*([points] - x0)\n")
        print("═"*30)
        print("Rotation original R = ")
        for row in V:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Inverse semi-major axis original S = ")
        S = [[1/a,0,0],[0,1/b,0],[0,0,1/c]]
        for row in S:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Ellipse center original x0 = ")
        for x in x0:
            print(f"{x:8.3f}")
        print("\n" + "═"*30)
        print("\nRotation fitted R =")
        for row in V_fit:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Inverse semi-major axis fitted S = ")
        S = [[1/a_fit,0,0],[0,1/b_fit,0],[0,0,1/c_fit]]
        for row in S:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Ellipse center fitted x0 = ")
        for x in x0_fit:
            print(f"{x:8.3f}")
        
    else:
        print("FITTED ELLIPSOID PARAMETERS:")
        print("\n" + "═"*30)
        for n, val in zip("ABCDEFGHIJ", coeffs):
            print(f"{n} = {val:.7f}")
        print(f"J0_fit = {J0_fit:.7f}")
        print(f"Center: {x0_fit.round(6)}")
        print(f"Semi-axes: a={a_fit:.4f}  b={b_fit:.4f}  c={c_fit:.4f}")

        print("\n" + "═"*30)
        print("\nUnit transformation: S*R^T*([points] - x0)\n")
        print("═"*30)
        print("\nRotation fitted R =")
        for row in V_fit:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Inverse semi-major axis fitted S = ")
        S = [[1/a_fit,0,0],[0,1/b_fit,0],[0,0,1/c_fit]]
        for row in S:
            print(" ".join(f"{v:8.8f}" for v in row))
        print("Ellipse center fitted x0 = ")
        for x in x0_fit:
            print(f"{x:8.3f}")

    print(f"\nRMS geometric error = {rms:.6f}")
    print("═"*50 + "\n")


def plot_ellipsoids(points, x0_orig, x0_fit, coeffs, a_fit, b_fit, c_fit, V_fit):
    """
    Plot measured points + parametric surface of the fitted ellipsoid
    using matplotlib's plot_surface
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot measured/noisy points
    ax.scatter(points[0], points[1], points[2],
               c='black', s=4, alpha=0.45, label='Measured points')
    
    # Plot centers
    ax.scatter(*x0_orig, c='blue', s=140, marker='*',
               label='Reference center (dummy)', zorder=10)
    ax.scatter(*x0_fit, c='red', s=180, marker='*',
               label='Fitted center', zorder=10)
    
    # ──────────────────────────────────────────────────────────────
    # Parametric surface of ellipsoid
    # ──────────────────────────────────────────────────────────────
    if any(np.isnan([a_fit, b_fit, c_fit])):
        print("Cannot plot parametric surface - invalid fitted semi-axes")
    else:
        # Create parametric angles
        u = np.linspace(0, 2 * np.pi, 60)     # azimuthal
        v = np.linspace(0, np.pi, 40)         # polar
        u, v = np.meshgrid(u, v)
        
        # Ellipsoid in its principal frame (aligned with axes a,b,c)
        x_ell = a_fit * np.sin(v) * np.cos(u)
        y_ell = b_fit * np.sin(v) * np.sin(u)
        z_ell = c_fit * np.cos(v)
        
        # Stack into 3×N array and reshape to match grid
        principal_pts = np.stack([x_ell, y_ell, z_ell], axis=0)  # shape (3, n_v, n_u)
        
        # Apply rotation (V_fit is the rotation matrix from principal → world frame)
        rotated = np.einsum('ij,jkl->ikl', V_fit, principal_pts)  # shape (3, n_v, n_u)
        
        # Translate to fitted center
        x_surf = rotated[0] + x0_fit[0]
        y_surf = rotated[1] + x0_fit[1]
        z_surf = rotated[2] + x0_fit[2]
        
        # Plot the surface
        surf = ax.plot_surface(x_surf, y_surf, z_surf,
                              color='red', alpha=0.22,
                              linewidth=0.3, antialiased=True,
                              rstride=2, cstride=2,
                              shade=True)
        
        # Optional: add some lighting/shading effect
        surf.set_facecolor((1, 0.3, 0.3, 0.22))  # reddish tint
        surf.set_edgecolor('none')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Measured Points + Parametric Fitted Ellipsoid Surface')
    ax.legend()
    ax.set_box_aspect([1,1,1])
    
    plt.show()


def transform_to_sphere(points, x0, V, a, b, c):
    R_inv = V.T
    S_inv = np.diag([1/a, 1/b, 1/c])

    sphere_pts = np.zeros_like(points)
    for i in range(points.shape[1]):
        q = points[:, i] - x0
        r = R_inv @ q
        sphere_pts[:, i] = S_inv @ r

    return sphere_pts


def plot_sphere_points(sphere_points):
    """
    Plot the transformed (calibrated) points together with a reference unit sphere
    """
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the calibrated/transformed points
    ax.scatter(sphere_points[0], sphere_points[1], sphere_points[2],
               c='darkblue', s=6, alpha=0.7, label='Calibrated points',
               edgecolor='none', zorder=5)
    
    # --- Reference unit sphere ---
    # Create parametric unit sphere
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 40)
    u, v = np.meshgrid(u, v)
    
    x_sphere = np.sin(v) * np.cos(u)
    y_sphere = np.sin(v) * np.sin(u)
    z_sphere = np.cos(v)
    
    # Plot surface
    ax.plot_surface(x_sphere, y_sphere, z_sphere,
                    color='lightgray', alpha=0.18,
                    linewidth=0, antialiased=True,
                    rstride=2, cstride=2,
                    shade=True, zorder=1)
    
    # Optional: add wireframe for better shape perception
    ax.plot_wireframe(x_sphere, y_sphere, z_sphere,
                      color='gray', alpha=0.3, linewidth=0.6,
                      rstride=4, cstride=4, zorder=2)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Calibrated Points\n'
                 '(ideal result: points lie on the surface)')
    
    # Make axes limits symmetric around origin for fair comparison
    max_range = np.max(np.abs(sphere_points)) * 1.15
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    
    ax.legend()
    ax.set_box_aspect([1,1,1])
    
    plt.show()
# =============================================
# Real Time Data
# =============================================
def Get_data():
    print("LIS3MDL Magnetometer - Press SPACE to stop")
    print("-----------------------------------------\n")

    # Make stdin non-blocking and disable echo
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        # Initialize sensor - feel free to change parameters
        try:
            mag = pylisxmdl(FullScale.Gauss_4, ODR.Hz_80)
            print("Sensor initialized successfully")
        except Exception as e:
            print("Failed to initialize sensor:", e)
            print("(make sure pigpiod is running → sudo pigpiod)")
            return

        print("      X [G]        Y [G]        Z [G]     ")
        print("-------------------------------------------------------------------")

        running = True
        data = []
        while running:
            # Read data
            try:

                if(mag.data_ready()):
                    x, y, z = mag.read_gauss()
                    print(f"{x:12.5f}  {y:12.5f}  {z:12.5f} ",
                      end='\r', flush=True)
                    data.append([x,y,z])

            except RuntimeError as e:
                print(f"\nRead error: {e}")
                time.sleep(1)

            # Check for SPACE key (or Ctrl+C handled by except KeyboardInterrupt)
            key = get_key()
            if key == ' ':
                print("\n\nSPACE pressed → stopping...")
                running = False
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

            time.sleep(0.0025)  # ~12–13 updates/sec

    except KeyboardInterrupt:
        print("\n\nStopped by Ctrl+C")
    except Exception as e:
        print("\nError:", e)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal restored")
    return np.array(data).T

def key_pressed():
    i, _, _ = select.select([sys.stdin], [], [], 0.0)
    return i


def get_key():
    if key_pressed():
        return sys.stdin.read(1)
    return None

if __name__ == "__main__":
    main()