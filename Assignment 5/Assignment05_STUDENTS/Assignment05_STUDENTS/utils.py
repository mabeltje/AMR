import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import math

def draw_robot(x, b, color='gray', alpha=1, label='', ax=None):
    wheel_width = b/10.
    wheel_diameter = b/4.
    body = patches.Circle([0,0], b/2, color=color, alpha=alpha)
    wheel_l = patches.Rectangle([-wheel_diameter/2.0, -b/2],  wheel_diameter, wheel_width, color='black', alpha=alpha)
    wheel_r = patches.Rectangle([-wheel_diameter/2.0, +b/2-wheel_width],  wheel_diameter, wheel_width, color='black', alpha=alpha)
    
    elements = [body, wheel_l, wheel_r]
    
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
    
    for element in elements:
        element.set_transform(mpl.transforms.Affine2D().rotate(x[2])+ mpl.transforms.Affine2D().translate(x[0], x[1])+ax.transData)
        ax.add_patch(element)
    
    ax.annotate("", xy=(x[0]+b/1.9*np.cos(x[2]), x[1]+b/1.9*np.sin(x[2])), xytext=(x[0]+b/5*np.cos(x[2]), x[1]+b/5*np.sin(x[2])), arrowprops=dict(arrowstyle="->"))
        
    if len(label)>0 :
        ax.text(x[0],x[1],label, fontsize=10, horizontalalignment='center', verticalalignment='center')
        

def draw_arrow(x, alpha=1, label='', ax=None, b=0.5, color='black'):
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
 
    ax.annotate(label, xy=(x[0]+b/1.9*np.cos(x[2]), x[1]+b/1.9*np.sin(x[2])), xytext=(x[0], x[1]), 
                arrowprops=dict(arrowstyle="->", color=color), size=15 )
    
def draw_state_sequence(xs, b=1, color='black', ax=None, alpha=1, label=None):
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
    
    x_prev = None
    for x in xs:
        if not (x_prev is None):
            ax.plot([x_prev[0], x[0]], [x_prev[1], x[1]], '-o', color=color, alpha=alpha, label=label)
            label=None
#         draw_arrow(x, b, color=color, ax=ax)
        x_prev = x
        
from scipy.stats import multivariate_normal

def draw_state_distribution(x, P, alpha=0.9, ax=None):
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    
    x1, x2 = np.meshgrid(np.linspace(x_min, x_max, 100), np.linspace(y_min, y_max, 100))

    var = multivariate_normal(mean=x[:2], cov=P[:2,:2])

    X = np.concatenate([x1.reshape(-1,1), x2.reshape(-1,1)],1)
    mu = x[:2].reshape(-1,2)
    P_inv =np.linalg.inv(P[:2,:2])
    z = var.pdf(X).reshape(100,100)

    cs = ax.contour(x1, x2, z, alpha=alpha, zorder=-1)
    
def draw_map(M, ax=None, color='red', dashed=False, linewidth=1, alpha=1):
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
    if isinstance(M, np.ndarray):
        M = [M[:,i] for i in range(M.shape[1])]
    
    for alp, rho in M:
        x1, x2, y1, y2 = polar_to_cartesian(rho, alp)
        ax.plot([x1, x2], [y1, y2], color=color, ls='dashed' if dashed else '-', linewidth=linewidth, alpha=alpha)
        
def draw_associations(M, Z, a):
    cmap = plt.get_cmap("tab10")
    for i in range(len(M)):
        draw_map([M[i]], color=cmap(i), linewidth=2)
    for i in range(len(Z)):
        if a[i]>=0:
            draw_map([Z[i]], color=cmap(a[i]), dashed=True)
        else:
            draw_map([Z[i]], color='k', dashed=True, alpha=0.6)
            
def draw_simulation(x, P, u, k, b, ax=None, N=100, alpha=0.2):
    if ax is None:
        fig = plt.gcf()
        ax = fig.gca()
    
    for i in range(N):
        x_actual = np.random.multivariate_normal(mean=x, cov=P)
        x_final = execute_instruction(x, u, b, k=k)
        ax.plot(x_final[0],x_final[1],'.r', alpha=alpha)

def compute_Q(u, k):
    Q = np.eye(2)*k
    Q[0,0]*=np.abs(u[0])
    Q[1,1]*=np.abs(u[1])
    
    return Q

def execute_instruction(x, u, b, k=0.05):   
    Q= compute_Q(u, k)
    
    u_noisy = np.random.multivariate_normal(u, Q)
    
    delta_s = u_noisy.mean()
    delta_theta = (u_noisy[1]-u_noisy[0])/b
    
    delta_x = np.array([
        delta_s * np.cos(x[2] + delta_theta/2),
        delta_s * np.sin(x[2] + delta_theta/2),
        delta_theta
    ])
    
    return x + delta_x

def generate_instructions(n):
    return [np.random.multivariate_normal(np.ones(2)*0.5, np.eye(2)*0.1) for _ in range(n)]

def execute_all_instructions(x, U, b, k=0.05):
    for u in U:
        x = execute_instruction(x, u, b, k=k)
    return x



def find_intersection(rho_1, alpha_1, rho_2, alpha_2):
    """
    This function finds the intersection between to polar lines.
    input:
        - rho_1 : float
        - alpha_1 : float
        - rho_2 : float
        - alpha_2 : float
    output:
        - coords : (float, float)
    """

    if alpha_1 == alpha_2:
        return False
    
    rho_diff = rho_1 - rho_2
    
    x_part = np.cos(alpha_1) - np.cos(alpha_2)
    y_part = np.sin(alpha_1) - np.sin(alpha_2)
    
    
    if alpha_1 == 0:
        y = (rho_diff - (rho_1 * x_part))/(y_part)
        if y > 16 or y < -16:
            return False
        return (rho_1, y)

    if alpha_1 == math.pi/2:
        x = (rho_diff - (rho_1 * y_part))/(x_part)
        if x > 16 or x < -16:
            return False
        return (x, rho_1)

def polar_to_cartesian(rho, alpha):
    """
    This function convert a polar line to two cartesian points to make a line
    input:
        - rho : float
        - alpha : float
    output:
        - two cartesian points : floats
    """
    points = []
    
    p = find_intersection(-16, 0, rho, alpha)
    if p:
        points.append(p)

    p = find_intersection(16, 0, rho, alpha)
    if p:
        points.append(p)
    
    p = find_intersection(-16, math.pi/2, rho, alpha)
    if p:
        points.append(p)
    
    p = find_intersection(16, math.pi/2, rho, alpha)
    if p:
        points.append(p)
    
    if len(points) < 2:
        print("This line is out of bounds")
        return False
    return points[0][0], points[1][0], points[0][1], points[1][1]

def make_measurements(x, M, k1=0.009, k2=0.0003):
    x = np.array(x)
    
    Z = []
    R = []
    for m in M:
        m = np.array(m)

        dist = m[1]-(x[0]*np.cos(m[0])+x[1]*np.sin(m[0]))
        z = [
            m[0]-x[2], 
            dist
        ]
        
        n_samples = np.random.choice([1,2,3])
        for _ in range(n_samples):
            cov = np.eye(2)
            cov[0,0] *= k2
            cov[1,1] *= k1
            cov *= np.abs(dist)
            
            measurement = np.random.multivariate_normal(z, cov)
            Z.append(measurement)
            R.append(cov)
            
    perm = list(np.random.permutation(len(Z)))
                    
    return [Z[i] for i in perm], [R[i] for i in perm]

def get_default_map():
    return [
        [0, 7],
        [np.pi/2., 7],
        [np.pi/2., 1],
        [0, 2],
        [np.pi/4., 4]
    ]

