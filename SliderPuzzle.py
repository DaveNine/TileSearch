import numpy as np
import time

class TileSearch(object):
    def __init__(self, size):
        self.final_state = np.append(np.arange(1, size**2), np.zeros(1))

    def arrtotuple(self, array):
        return hash(tuple(array))


    def flattenBoard(self, state):
        return np.reshape(state, (np.size(state)))


    def squareBoard(self, state):
        return np.reshape(state, (np.sqrt(len(state)), np.sqrt(len(state))))


    # Gives a list of possible neighboring states
    def neighborStates(self, board):
        directions = [[0, 1], [0, -1], [1, 0], [-1, 0]]
        neighbors = []
        square = self.squareBoard(board)
        n, m = np.argwhere(square == 0)[0]
        for x in directions:
            t = x[0]
            s = x[1]
            tempSquare = np.copy(square)
            if (n+t >= 0 and n+t < np.shape(tempSquare)[0]) and (m+s >= 0 and m+s < np.shape(tempSquare)[0]):
                temp = tempSquare[n+t][m+s]
                tempSquare[n][m] = temp
                tempSquare[n+t][m+s] = 0
                neighbors.append(self.flattenBoard(tempSquare))
        return neighbors

    # Breadth First Search
    def BFS(self, start):
        frontier = [start]
        came_from = {}
        came_from[self.arrtotuple(start)] = None

        program_start = time.time()
        while len(frontier) is not 0:
            current = frontier
            newFrontier = []
            if any((self.final_state == x).all() for x in current):
                print("Path has been found and took {:0.2f} seconds, generating path.".format(
                                                                                        time.time()-program_start))
                break
            for frontierElement in current:
                for T in self.neighborStates(frontierElement):
                    if self.arrtotuple(self.flattenBoard(T)) not in came_from:
                        newFrontier.append(T)
                        came_from[self.arrtotuple(self.flattenBoard(T))] = frontierElement
            frontier = newFrontier

        # Path Generation
        T = self.final_state
        path = [T]
        while not np.array_equal(T, start):
            T = came_from[self.arrtotuple(T)]
            path.append(T)
        path.reverse()
        print("Path has been generated.")
        return path

# Example
T = TileSearch(3)
start = np.array([4,6,7,5,2,1,8,3,0])

path = T.BFS(start)
print("Path has {} moves".format(len(path)-2))
print("Path is: ")

for k in path:
    print(T.squareBoard(k))
    print("-----------------")



