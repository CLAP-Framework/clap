import os
import os.path as osp
import random
import numpy as np
from rtree import index as rindex
from collections import deque


class Reconstruct_rtree:
    # Construct an rtree from data
    def __init__(self):
        if osp.exists("state_index.dat"):
            os.remove("state_index.dat")
            os.remove("state_index.idx")
        self.obs_dimension = 16
        self.visited_state_value = np.loadtxt("visited_state_value.txt",usecols=(i for i in range(self.obs_dimension + 1, self.obs_dimension + 3, 1)))
        self.visited_state_value = self.visited_state_value.tolist()
        self.visited_state_counter = len(self.visited_state_value)


        visited_state_tree_prop = rindex.Property()
        visited_state_tree_prop.dimension = self.obs_dimension+1
        self.visited_state_dist = np.array([[1, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 0.1]])#, 10, 0.3, 3, 1, 0.1]])
        self.visited_state_tree = rindex.Index('state_index',properties=visited_state_tree_prop)
        self.recostruct_state_counter = 0
        self.recostruct_rtree()

    def recostruct_rtree(self):
        print("[RLS]: Start Reconstruct rtree!")

        self.all_data_visited = np.loadtxt("visited_state_value.txt")
        for i in range(self.visited_state_counter - 1):
            state_slice = slice(0, self.obs_dimension + 1, 1)
            state_to_record = self.all_data_visited[i][state_slice]
            self.visited_state_tree.insert(self.recostruct_state_counter,
                    tuple((state_to_record-self.visited_state_dist[0]).tolist()+(state_to_record+self.visited_state_dist[0]).tolist()))
            self.recostruct_state_counter += 1
        print("[RLS]: Reconstruct rtree success!")


reconstruct = Reconstruct_rtree()
