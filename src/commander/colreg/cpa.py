import numpy as np

def compute_cpa_tcpa(pos1, speed1, heading1, pos2, speed2, heading2):
    def heading_to_velocity(speed, heading):
        rad = np.radians(heading)
        return np.array([speed * np.cos(rad), speed * np.sin(rad)])
    
    vel1 = heading_to_velocity(speed1, heading1)
    vel2 = heading_to_velocity(speed2, heading2)
    rel_pos = np.array(pos2) - np.array(pos1)
    rel_vel = vel2 - vel1
    
    tcpa = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
    cpa = np.linalg.norm(rel_pos + tcpa * rel_vel)
    
    return cpa, tcpa


if __name__ == "__main__":
    pos1 = [0, 0]
    heading1 = 0
    speed1 = 10

    pos2 = [10, 10]
    heading2 = 270
    speed2 = 8

    cpa, tcpa = compute_cpa_tcpa(pos1, speed1, heading1, pos2, speed2, heading2)
    print(f"CPA: {cpa:.2f}, TCPA: {tcpa:.2f}")
