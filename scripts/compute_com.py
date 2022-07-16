import numpy as np
import matplotlib
matplotlib.use('WebAgg')

import matplotlib.pyplot as plt

np.random.seed(19680801)


class ulink:
    def __init__(self, 
        name: str, 
        sister: int, 
        child: int, 
        mother: int, 
        m: int,
        a: np.ndarray, 
        b: np.ndarray):

        self.name = name
        self.sister = sister
        self.child = child
        self.mother = mother
        self.m = m # mass of link
        self.a = a # joint axis vector in parent frame
        self.b = b # position of origin in parent frame

        self.p = None # position in world coordinates
        self.R = None # orientation in world coordiantes
        self.q = None # joint angle 
        

links = [
    ulink(name = "BODY",
        sister = -1,
        child = 1,
        mother = -1, 
        m = 1,
        a = None,
        b = None),
    ulink(name = "RLEG_J0", # r_hip_yaw
        sister = 7,
        child = 2,
        mother = 0, 
        m = 1,
        a = np.array([0, 0, -1]).transpose(),
        b = np.array([0, -2, -1]).transpose()),
    ulink(name = "RLEG_J1", # r_hip_roll
        sister = -1,
        child = 3,
        mother = 1, 
        m = 1,
        a = np.array([-1, 0, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose()),
        
    ulink(name = "RLEG_J2", # r_hip_pitch
        sister = -1,
        child = 4,
        mother = 2, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose()),
    ulink(name = "RLEG_J3", # r_knee_pitch
        sister = -1,
        child = 5,
        mother = 3, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, -5]).transpose()),
    ulink(name = "RLEG_J4", # r_ankle_pitch
        sister = -1,
        child = 6,
        mother = 4, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, -5]).transpose()),
    ulink(name = "RLEG_J5", # r_ankle roll
        sister = -1,
        child = -1,
        mother = 5, 
        m = 1,
        a = np.array([1, 0, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose()),
    ulink(name = "LLEG_J0", # l_hip_yaw
        sister = -1,
        child = 8,
        mother = 0, 
        m = 1,
        a = np.array([0, 0, -1]).transpose(),
        b = np.array([0, 2, -1]).transpose()),
    ulink(name = "LLEG_J1", # l_hip_roll
        sister = -1,
        child = 9,
        mother = 7, 
        m = 1,
        a = np.array([-1, 0, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose()),
    ulink(name = "LLEG_J2", # l_hip_pitch
        sister = -1,
        child = 10,
        mother = 8, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose()),
    ulink(name = "LLEG_J3", # l_knee_pitch
        sister = -1,
        child = 11,
        mother = 9, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, -5]).transpose()),
    ulink(name = "LLEG_J4", # l_ankle_pitch
        sister = -1,
        child = 12,
        mother = 10, 
        m = 1,
        a = np.array([0, 1, 0]).transpose(),
        b = np.array([0, 0, -5]).transpose()),
    ulink(name = "LLEG_J5", # l_ankle_roll
        sister = -1,
        child = -1,
        mother = 11, 
        m = 1,
        a = np.array([1, 0, 0]).transpose(),
        b = np.array([0, 0, 0]).transpose())
]

links[0].p = np.array([0, 0, 0]).transpose()
links[0].R = np.identity(3)

# Given a list of links set their joint angles
def set_joint_angles(links: list[ulink], angles):
    for i in range(len(links)):
        links[i].q = angles[i]

# Print out link names given a list of links
def print_link_names(links: list[ulink], j: int):
    if j != -1:
        print(links[j].name)
        print_link_names(links, links[j].child)
        print_link_names(links, links[j].sister)

# Returns the total mass of the robot
def total_mass(links: list[ulink], j: int):
    if j == -1: return 0
    else: return links[j].m + total_mass(links, links[j].sister) + total_mass(links, links[j].child)

# Returns the cross product form of the vector
def cpm(w: np.ndarray):
    return np.matrix(
        [[0, -w[2], -w[1]],
         [w[2], 0, -w[0]],
         [-w[1], w[0], 0]]
    )

def Rodrigues(axis: np.ndarray, angle: int):
    return np.identity(3) + cpm(axis)*np.sin(angle) + np.matmul(cpm(axis), cpm(axis))*(1-np.cos(angle))

# Compute forward kinematics with the current set of joint angles for the links
def forward_kinematics(links: list[ulink], j: int):
    if j == -1: return
    if j != 0:
        i = links[j].mother
        links[j].p = np.squeeze(np.asarray(np.matmul(links[i].R, links[j].b))) + links[i].p # calculate position; rotate then translate
        # calculate new orientation; need to compute rotation matrix 
        # (of this link in parent) then multiply by parent RotM in world
        links[j].R = np.matmul(links[i].R, Rodrigues(links[j].a, links[j].q)) 
        
    forward_kinematics(links, links[j].sister) 
    forward_kinematics(links, links[j].child)


if __name__ == "__main__":
    set_joint_angles(links, [0, 0.1, 0.4, -0.1, 0.1, 0, 0.1, 0.3, 0.4, 0.2, 0.1, 0.2, 0])
    set_joint_angles(links, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    print_link_names(links, 0)
    print(total_mass(links, 0))
    forward_kinematics(links, 0)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for link in links:
        print(link.p)
        ax.scatter(link.p[0], link.p[1], link.p[2])
        if link.mother != -1:
            ax.plot([link.p[0], links[link.mother].p[0]], 
                    [link.p[1], links[link.mother].p[1]],
                    [link.p[2], links[link.mother].p[2]])
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
