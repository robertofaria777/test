import numpy as np
import matplotlib.pyplot as plt
import math

# --- 1. Python "Translation" of the MATLAB Driver Model ---
def driver_model_paper(X, Y, psi, u, delta_prev, road, params, prev_idx):
    """
    Python implementation of the MATLAB driver_model_paper function.
    """
    Tp = params.get('Tp', 1.0)
    Klat = params.get('Klat', 0.1)
    L = params.get('L', 2.5)
    Kug = params.get('Kug', 0.0)
    
    # --- 1. Find Current Position S_G on Track (Update prev_idx) ---
    search_window = 50
    N = len(road)
    
    idx = prev_idx
    min_dist = float('inf')
    
    # Clamp search range
    start_search = prev_idx
    end_search = min(prev_idx + search_window, N - 1)
    
    for i in range(start_search, end_search):
        p1 = road[i]
        p2 = road[i+1]
        
        t_vec = p2 - p1
        seg_len = np.linalg.norm(t_vec)
        if seg_len < 1e-6:
            continue
        t_hat = t_vec / seg_len
        
        v_vec = np.array([X, Y]) - p1
        proj_len = np.dot(v_vec, t_hat)
        
        if 0 <= proj_len <= seg_len:
            d = np.linalg.norm(v_vec - proj_len * t_hat)
            if d < min_dist:
                min_dist = d
                idx = i

    next_idx = idx
    
    # --- 2. Calculate Predicted Point P ---
    # Steady State Handling Equation
    if abs(delta_prev) < 1e-4:
        R = float('inf')
    else:
        R = (L + Kug * u**2) / delta_prev
        
    tG = np.array([math.cos(psi), math.sin(psi)])
    nG = np.array([-math.sin(psi), math.cos(psi)])
    
    if math.isinf(R):
        P = np.array([X, Y]) + u * Tp * tG
    else:
        # Center of curvature
        O = np.array([X, Y]) + R * nG
        theta = (u * Tp) / R
        
        # Rotate vector G->O by theta? No, "P = G + R nG - R [Rot] nG" ?
        # Actually easier: Rotate vector (G - O) by theta around Z axis (2D rot)
        v = np.array([X, Y]) - O # This is -R * nG
        
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # 2D Rotation
        v_rot = np.array([
            v[0] * cos_t - v[1] * sin_t,
            v[0] * sin_t + v[1] * cos_t
        ])
        
        P = O + v_rot

    # --- 3. Find Track Segment for P ---
    best_d = float('inf')
    
    # Search ahead from current car position index
    for k in range(next_idx, min(next_idx + 100, N - 1)):
        curr_S = road[k]
        next_S = road[k+1]
        
        seg_vec = next_S - curr_S
        s_len = np.linalg.norm(seg_vec)
        if s_len < 1e-6: continue
            
        t_hat_L = seg_vec / s_len
        
        p_vec = P - curr_S
        proj = np.dot(p_vec, t_hat_L)
        
        if 0 <= proj <= s_len:
            # Valid segment!
            n_L_local = np.array([-t_hat_L[1], t_hat_L[0]])
            dist = np.dot(p_vec, n_L_local)
            
            if abs(dist) < abs(best_d):
                best_d = dist
                # Found a valid update
    
    if math.isinf(best_d):
        best_d = 0 # Fallback
        
    d_lat = best_d
    
    # --- 4. Update Steering ---
    delta_cmd = delta_prev - Klat * d_lat
    
    # Saturation
    max_steer = np.deg2rad(35)
    delta_cmd = max(-max_steer, min(delta_cmd, max_steer))
    
    return delta_cmd, next_idx, P

# --- 2. Track & Vehicle Setup ---

def generate_track():
    # Simple Oval Track
    t = np.linspace(0, 2*np.pi, 200)
    x = 100 * np.cos(t)
    y = 50 * np.sin(t)
    # Add a chicane or wiggle
    # y += 10 * np.sin(5*t) 
    return np.column_stack((x, y))

def vehicle_dynamics(x, y, psi, u, delta, dt, L=2.5):
    # Kinematic Bicycle Model
    # x_dot = u * cos(psi)
    # y_dot = u * sin(psi)
    # psi_dot = (u / L) * tan(delta)
    
    x_new = x + u * math.cos(psi) * dt
    y_new = y + u * math.sin(psi) * dt
    psi_new = psi + (u / L) * math.tan(delta) * dt
    
    return x_new, y_new, psi_new

def run_simulation():
    road = generate_track()
    
    # Initial State
    x, y = road[0]
    psi = np.pi/2 + 0.2 # Start with bad heading to test correction
    u = 20.0 # m/s
    delta = 0.0
    
    params = {'Tp': 0.8, 'Klat': 0.05, 'L': 2.5, 'Kug': 0.00}
    
    dt = 0.05
    steps = 1000
    
    # History
    path_x, path_y = [], []
    steer_hist = []
    
    prev_idx = 0
    
    plt.figure(figsize=(10,6))
    plt.plot(road[:,0], road[:,1], 'k--', label='Track')
    
    for i in range(steps):
        path_x.append(x)
        path_y.append(y)
        steer_hist.append(delta)
        
        # 1. Driver Model
        delta_cmd, prev_idx, P_preview = driver_model_paper(x, y, psi, u, delta, road, params, prev_idx)
        
        if i % 20 == 0:
             plt.plot([x, P_preview[0]], [y, P_preview[1]], 'g-', alpha=0.3)
             plt.plot(P_preview[0], P_preview[1], 'gx', markersize=2)

        # 2. Vehicle Update
        # Assume generic steering actuator lag
        # delta += (delta_cmd - delta) * 0.2 
        delta = delta_cmd # Instant
        
        x, y, psi = vehicle_dynamics(x, y, psi, u, delta, dt)
        
        # Stop if lap complete (rough check)
        if i > 100 and np.linalg.norm([x - road[0,0], y - road[0,1]]) < 5:
            print(f"Lap completed in {i} steps")
            break
            
    plt.plot(path_x, path_y, 'b-', label='Vehicle Path')
    plt.axis('equal')
    plt.legend()
    plt.title(f"Simulation Results (Tp={params['Tp']}, Klat={params['Klat']})")
    plt.grid(True)
    plt.savefig('simulation_result.png')
    print("Simulation complete. Image saved.")

if __name__ == "__main__":
    run_simulation()
