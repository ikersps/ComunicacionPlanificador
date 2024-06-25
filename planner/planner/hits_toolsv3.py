#These are the hits tools used in the distributed solution

import numpy as np

class Hits_toolsv3():
    def __init__(self, initial_positions, final_positions, speeds, next_wp):
        # final_positions contains tuples that contain the initial and the final position defining the trajectory of each drone
        self.final_positions = final_positions
        self.initial_positions = initial_positions
        self.speeds = speeds
        self.next_wp = next_wp
            
    def nearest_positions(self):
        result = set()  # Use a set to avoid duplicates

        for i in range(len(self.final_positions)):
            for j in range(i+1, len(self.final_positions)):
                if (abs(self.final_positions[i][2] - self.final_positions[j][2]) <= 1.5):  #In very different heights it does not need hit verification
                    p1_final =  np.array(self.final_positions[i])
                    p2_final =  np.array(self.final_positions[j])
                    distance = np.linalg.norm(p1_final - p2_final)
                    if distance < 30:
                        p1_initial =  np.array(self.initial_positions[i])
                        p2_initial =  np.array(self.initial_positions[j])
                        initial_distance = np.linalg.norm(p1_initial - p2_initial)
                        if (initial_distance >= distance):  #They are getting closer
                            intersection = self.trajectory_intersection(p1_initial, p1_final, p2_initial, p2_final)
                            intersection = [intersection[0], intersection[1], p1_final[2]]
                            d1_to_wps = np.linalg.norm(self.next_wp[i] - p1_final)
                            d2_to_wps = np.linalg.norm(self.next_wp[j] - p2_final)
                            d1_to_int = np.linalg.norm(p1_final - intersection)
                            d2_to_int = np.linalg.norm(p2_final - intersection)
                            t1 = d1_to_int / self.speeds[i]
                            t2 = d2_to_int / self.speeds[j]
                            if (d1_to_wps - d1_to_int > 2 and  d2_to_wps - d2_to_int > 2 and abs(t2 - t1) <= 2):
                                result.add(i)
                                result.add(j)

        return result
    
    def trajectory_intersection(self, p1, p2, p3, p4):
        p1_2d = np.array([p1[0], p1[1]])
        p2_2d = np.array([p2[0], p2[1]])
        p3_2d = np.array([p3[0], p3[1]])
        p4_2d = np.array([p4[0], p4[1]])

        d1 = p2_2d - p1_2d
        d2 = p4_2d - p3_2d
        
        A = np.array([d1, -d2]).T
        b = p3_2d - p1_2d
        t = np.linalg.solve(A, b)
        
        intersection = p1_2d + t * d1
        return intersection
    
final_positions = [(1, 1, 1), (1, 0, 1)]
initial_positions = [(0, 0, 1), (0, 1, 1)]
speeds = [30, 0.5]
sizes = [1]

hits_tools = Hits_toolsv3(initial_positions, final_positions, speeds, sizes)

print(hits_tools.nearest_positions())
