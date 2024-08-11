import sys
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np

from cpa import compute_cpa_tcpa


def plot_positions_and_headings(situation, position1, course1, position2, course2, bearing1_to_2, bearing2_to_1):
    plt.figure()
    plt.arrow(*position1, 5*np.cos(np.radians(course1)), 5*np.sin(np.radians(course1)), head_width=2, head_length=2, fc='blue', ec='blue') # pyright: ignore
    plt.arrow(*position2, 5*np.cos(np.radians(course2)), 5*np.sin(np.radians(course2)), head_width=2, head_length=2, fc='red', ec='red') # pyright: ignore
    plt.plot(*zip(position1, position2), 'k--')
    plt.text(*position1, 'P1', fontsize=12, ha='right') # pyright: ignore
    plt.text(*position2, 'P2', fontsize=12, ha='left') # pyright: ignore
    plt.xlim(-60, 60)
    plt.ylim(-60, 60)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.title(f'{situation.name}\nBearing from P1 to P2: {bearing1_to_2:.2f}°\nBearing from P2 to P1: {bearing2_to_1:.2f}°')
    plt.show()

def compute_bearings(position1, course1, position2, course2):
    delta = position2 - position1
    angle_to_2_deg = np.degrees(np.arctan2(delta[1], delta[0])) % 360
    bearing_from_1_to_2 = -((angle_to_2_deg - course1 + 180) % 360 - 180)
    bearing_from_2_to_1 = -(((angle_to_2_deg + 180) % 360 - course2 + 180) % 360 - 180)
    return bearing_from_1_to_2, bearing_from_2_to_1

class Colreg(Enum):
    TAKEOVER = 0
    CROSSING = 1
    HEADON   = 2
    NONE     = 3


def colregs_scenario(B1, B2, cpa):
    cpa_safe_distance = 20
    
    if (abs(B1) <= 15) and abs(B2) >= (112.5):
        return Colreg.TAKEOVER

    if (15 < B1 <= 112.5) and cpa <= cpa_safe_distance:
        return Colreg.CROSSING

    elif abs(B1 + B2) <= 15:
        return Colreg.HEADON

    else:
        return Colreg.NONE

if __name__ == "__main__":
    scenarios = [
        {'pos1': np.array([0, 0]), 'pos2': np.array([50, 0]), 'course1': 6, 'course2': 186, 'speed1': 10, 'speed2': 10},
        {'pos1': np.array([-20, 30]), 'pos2': np.array([55, 30]), 'course1': 45, 'course2': 135, 'speed1': 10, 'speed2': 10},
        {'pos1': np.array([0, 0]), 'pos2': np.array([-20, 0]), 'course1': 0, 'course2': 90, 'speed1': 10, 'speed2': 10},
        {'pos1': np.array([5, 0]), 'pos2': np.array([0, 20]), 'course1': 90, 'course2': 90, 'speed1': 10, 'speed2': 10},
    ]

    noise_factor = 1
    for scenario in scenarios:
        pos1 = scenario['pos1'] + np.random.uniform(-noise_factor, noise_factor, 2)
        pos2 = scenario['pos2'] + np.random.uniform(-noise_factor, noise_factor, 2)
        course1 = scenario['course1'] + np.random.uniform(-noise_factor*2, noise_factor*2)
        course2 = scenario['course2'] + np.random.uniform(-noise_factor*2, noise_factor*2)
        speed1  = scenario['speed1']
        speed2  = scenario['speed2']
        bearing1_to_2, bearing2_to_1 = compute_bearings(pos1, course1, pos2, course2)
        cpa, tcpa = compute_cpa_tcpa(pos1, speed1, course1, pos2, speed2, course2)
        print(f"cpa: {cpa}")
        sys.stdout.flush()
        situation = colregs_scenario(bearing1_to_2, bearing2_to_1, cpa)
        plot_positions_and_headings(situation, pos1, course1, pos2, course2, bearing1_to_2, bearing2_to_1)


