import numpy as np
import matplotlib.pyplot as plt


def calc_a(p0, pf, tf, v0=0, vf=0):
        a0 = p0
        a1 = v0
        a2 = (3/tf**2)*(pf - p0) - 2*v0/tf - vf/tf
        a3 = -(2/tf**3)*(pf - p0) + (v0 + vf)/tf**2
        return [a0, a1, a2, a3]
    

def create_qubic_trajectory(p_start, p_finish, tf, inc): # tf - duration of generated trajectory; inc - timestep duration
    t = 0
    p = []
    p_d = []
    a = calc_a(p_start, p_finish, tf, 0, 0)
    for i in range(int(tf/inc)):
        p.append(a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3)
        p_d.append(a[1] + 2*a[2]*t + 3*a[3]*t**2)
        t += inc
    
    return p, p_d


def create_multiple_trajectory(p_start, p_finish, tf, inc):
    refs = []
    d_refs = []
    for i in range(len(p_start)):
        p, p_d = create_qubic_trajectory(p_start[i], p_finish[i], tf, inc)
        refs.append(p)
        d_refs.append(p_d)
    return refs, d_refs


# Test these functions
def main():
    theta_start = [0.11028755456209183, 2.156809091567993, -3.0121941566467285, -0.13045088946819305, 2.512023687362671, -3.0190162658691406, -0.11402397602796555, 2.2038586139678955, -3.014986753463745, 0.1452120840549469, 2.286531925201416, -3.052239418029785]
    theta_finish = [0, -1.57, 3.14, 0, 1.57, 3.14, 0, -1.57, 3.14, 0, 1.57, 3.14]
    tf = 5.0
    inc = 1/240

    if len(theta_start) != len(theta_finish):
        print("Error: Lists theta_start and theta_finish have different lengths")
        return
    
    theta = create_multiple_trajectory(theta_start, theta_finish, tf, inc)
    time = np.arange(0, tf, inc, dtype=np.float32)
    
    print("Trajectory size: {0}".format(len(theta[0])))
    
    fig1 = plt.figure(figsize=(20, 20), facecolor="white")
    for i in range(3):#len(theta_start)):
        plt.plot(time, theta[i])
    
    plt.title("Angle Thrajectory")
    plt.ylabel("Theta [deg]")
    plt.xlabel("t [s]")
    plt.grid(True)
    plt.show()

    


if __name__ == '__main__':
    main()