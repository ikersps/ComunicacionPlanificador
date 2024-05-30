import numpy as np

class Hits_toolsv3():
    def __init__(self, initial_positions, final_positions, sizes):
        # final_positions contains tuples that contain the initial and the final position defining the trajectory of each drone
        self.final_positions = final_positions
        self.initial_positions = initial_positions
        self.sizeDrone = sizes[0]
        for sizeD in sizes:
            if (sizeD > self.sizeDrone):
                self.sizeDrone = sizeD
            
    def nearest_positions(self):
        result = set()  # Use a set to avoid duplicates

        for i in range(len(self.final_positions)):
            for j in range(i+1, len(self.final_positions)):
                if (abs(self.final_positions[i][2] - self.final_positions[j][2]) <= 1.5):  #In very different heights it does not need hit verification
                    distance = np.linalg.norm(np.array(self.final_positions[i]) - np.array(self.final_positions[j]))
                    if distance < 4 * self.sizeDrone:
                        initial_distance = np.linalg.norm(np.array(self.initial_positions[i]) - np.array(self.initial_positions[j]))
                        if (initial_distance >= distance):  #They are getting closer
                            result.add(i)
                            result.add(j)

        return result


# Create some sample positions and sizes
final_positions = [(4, 4, 1), (1, 1, 1), (2, 2, 1), (4, 4, 1), (200, 200, 1)]
sizes = [1]

# Create an instance of Hits_toolsv2
hits_tool = Hits_toolsv3(final_positions, final_positions, sizes)

# Call the hit function and print the result
print(hits_tool.nearest_positions())