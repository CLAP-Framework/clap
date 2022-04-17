import os
import os.path as osp
import random
import numpy as np
from rtree import index as rindex
from collections import deque
from scipy import stats

global SAVEROOT
SAVEROOT = "/home/icv/zwt_rls/log"

class RLS(object):

    def __init__(self,
                 visited_times_thres = 30,
                 is_training = True,
                 debug = True,
                 save_new_data = True,
                 create_new_train_file = True,
                 create_new_record_file = False,
                 save_new_driving_data = True):

        self.visited_times_thres = visited_times_thres
        self.is_training = is_training
        self.trajectory_buffer = deque(maxlen=20)
        self.debug = debug
        self.save_new_data = save_new_data
        self.create_new_train_file = create_new_train_file
        self.create_new_record_file = create_new_record_file
        self.save_new_driving_data = save_new_driving_data
        self.obs_dimension = 16  
        self.gamma = 0.95
        self._setup_data_saving()    
        self.confidence_thres = 0.5
    
    def _setup_data_saving(self):

        if self.create_new_train_file:


            if osp.exists("visited_state_value.txt"):
                os.remove("visited_state_value.txt")
            if osp.exists("state_index.dat"):
                os.remove("state_index.dat")
                os.remove("state_index.idx")
            self.visited_state_value = []
            self.visited_state_counter = 0
        else:
            self.visited_state_value = np.loadtxt("visited_state_value.txt",usecols=(i for i in range(self.obs_dimension + 1, self.obs_dimension + 3, 1)))

            self.visited_state_value = self.visited_state_value.tolist()
            self.visited_state_counter = len(self.visited_state_value)

        self.visited_state_value_outfile = open("visited_state_value.txt", "a")
        self.visited_state_value_format = " ".join(("%f",)*(self.obs_dimension+1 +2))+"\n"


        visited_state_tree_prop = rindex.Property()
        visited_state_tree_prop.dimension = self.obs_dimension+1
        self.visited_state_dist = np.array([[1, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 0.1]])#, 10, 0.3, 3, 1, 0.1]])
        self.visited_state_tree = rindex.Index('state_index',properties=visited_state_tree_prop)
        # if not self.create_new_train_file:
        #     self.recostruct_state_counter = 0
        #     self.recostruct_rtree()

        

        if self.create_new_record_file:
            if osp.exists("driving_record.txt"):
                os.remove("driving_record.txt")
        self.driving_record_outfile = open("driving_record.txt","a")
        self.driving_record_format = " ".join(("%f",)*(self.obs_dimension+9))+"\n"

    
    # def recostruct_rtree(self):
    #     print("[RLS]: Start Reconstruct rtree!")

    #     self.all_data_visited = np.loadtxt("visited_state_value.txt")
    #     for i in range(self.visited_state_counter - 1):
    #         state_slice = slice(0, self.obs_dimension + 1, 1)
    #         state_to_record = self.all_data_visited[i][state_slice]
    #         self.visited_state_tree.insert(self.recostruct_state_counter,
    #                 tuple((state_to_record-self.visited_state_dist[0]).tolist()+(state_to_record+self.visited_state_dist[0]).tolist()))
    #         self.recostruct_state_counter += 1
    #     print("[RLS]: Reconstruct rtree success!")
    
    
    def act(self, obs, RL_action):
        if self.is_training:
            return self.act_train(obs, RL_action)
        else:
            return self.act_test(obs, RL_action)
    
    def act_train(self,obs, RL_action):
        if self.should_use_rule(obs): 
            return np.array(0)
        else:
            # epsilon greedy from DQN
            return RL_action
    
    def state_with_action(self,obs,action):
        return np.append(obs, action)

    def should_use_rule(self,obs):
        """
        Whether the state should use rule action
        """
        # Rule action not explore enough

        rule_state = self.state_with_action(obs,0)
        visited_times_rule = self._calculate_visited_times(rule_state,self.visited_state_tree)
        mean_rule, var_rule, sigma_rule = self._calculate_statistics_index(rule_state,
                                                                    self.visited_state_value,
                                                                    self.visited_state_tree) 

        if self.debug:
            print("[RLS]: Rule visited times:",visited_times_rule, mean_rule, var_rule, sigma_rule)

        if visited_times_rule < self.visited_times_thres:
            return True

        # Rule perform good
        # mean_rule in (-10,0), related to reward
        explore_motivation = random.uniform(-10,0)
        if explore_motivation < mean_rule:
            return True
        print("[RLS]: RL exploration")
        return False

    def act_test(self, obs, RL_action):
        if RL_action == 0:
            print("[RLS]: Running: Rule_action")
            return np.array(0)
        rule_state = self.state_with_action(obs,0)
        visited_times_rule = self._calculate_visited_times(rule_state,self.visited_state_tree)
        mean_rule, var_rule, sigma_rule = self._calculate_statistics_index(rule_state,
                                                                    self.visited_state_value,
                                                                    self.visited_state_tree)
        print("[RLS]: Rule_Value:",mean_rule, var_rule, sigma_rule) 
        for candidate_action in range(1,16):

            RL_state = self.state_with_action(obs,candidate_action)
            visited_times_RL = self._calculate_visited_times(RL_state,self.visited_state_tree)
            mean_RL, var_RL, sigma_RL = self._calculate_statistics_index(RL_state,self.visited_state_value,self.visited_state_tree)

            if visited_times_rule < self.visited_times_thres or visited_times_RL < 10 or mean_rule > -0.1:
                continue

            var_diff = var_rule/visited_times_rule + var_RL/visited_times_RL
            sigma_diff = np.sqrt(var_diff)
            mean_diff = mean_RL - mean_rule

            z = mean_diff/sigma_diff
            print(candidate_action, 1 - stats.norm.cdf(-z))
            if 1 - stats.norm.cdf(-z) > self.confidence_thres:
                print("[RLS]: RL take over! Action:",candidate_action)
                return np.array(candidate_action)
        
        print("[RLS]: Running: Rule_action")
        return np.array(0)

    ############## RLS Confidence ##############

    def _calculate_visited_times(self, obs, visited_state_tree):

        return sum(1 for _ in visited_state_tree.intersection(obs.tolist()))

    def _calculate_statistics_index(self, obs, visited_state_value, visited_state_tree):
        """
        Calculate statistics_idx
        """
        if self._calculate_visited_times(obs,visited_state_tree) == 0:
            return -1, -1, -1

        value_list = [visited_state_value[idx] for idx in visited_state_tree.intersection(obs.tolist())]
        value_array_av = np.array(value_list)
        value_array = value_array_av[:,1]
        # value_array_rule = value_array[value_array[:,0]==0][:,1]
        # value_array_RL = value_array
        mean = np.mean(value_array)
        var = np.var(value_array)
        sigma = np.sqrt(var)

        return mean,var,sigma

    ############## DATASET ##############

    def add_data(self, obs, action, rew, new_obs, done):
        self.trajectory_buffer.append((obs, action, rew, new_obs, done))
        while len(self.trajectory_buffer) > 10:
            obs_left, action_left, rew_left, new_obs_left, done_left = self.trajectory_buffer.popleft()
            state_to_record = self.state_with_action(obs_left, action_left)
            action_to_record = action_left
            r_to_record = rew_left
            if self.save_new_data:
                self.visited_state_value.append([action_to_record,r_to_record])
                self.visited_state_tree.insert(self.visited_state_counter,
                    tuple((state_to_record-self.visited_state_dist[0]).tolist()+(state_to_record+self.visited_state_dist[0]).tolist()))


                all_to_record = np.append(state_to_record, action_to_record)
                all_to_record = np.append(all_to_record, r_to_record)
                self.visited_state_value_outfile.write(self.visited_state_value_format % tuple(all_to_record))

                self.visited_state_counter += 1

        if done:
            _, _, rew_right, _, _ = self.trajectory_buffer[-1]
            while len(self.trajectory_buffer)>0:
                obs_left, action_left, rew_left, new_obs_left, done_left = self.trajectory_buffer.popleft()
                action_to_record = action_left
                r_to_record = rew_right*self.gamma**len(self.trajectory_buffer)
                state_to_record = self.state_with_action(obs_left, action_left)
                if self.save_new_data:
                    self.visited_state_value.append([action_to_record,r_to_record])
                    self.visited_state_tree.insert(self.visited_state_counter,
                        tuple((state_to_record-self.visited_state_dist).tolist()[0]+(state_to_record+self.visited_state_dist).tolist()[0]))

                    all_to_record = np.append(state_to_record, action_to_record)
                    all_to_record = np.append(all_to_record, r_to_record)
                    self.visited_state_value_outfile.write(self.visited_state_value_format % tuple(all_to_record))
                    self.visited_state_counter += 1

        if self.save_new_driving_data:
            state_rule = self.state_with_action(obs,0)
            visited_times_rule = self._calculate_visited_times(state_rule,self.visited_state_tree)
            mean_rule, var_rule, sigma_rule = self._calculate_statistics_index(state_rule,self.visited_state_value,self.visited_state_tree)
            if action == 0:
                record_data = state_rule
                visited_times_RL = -1
                mean_RL = -1
                var_RL = -1
            else:
                RL_state = self.state_with_action(obs,action)
                record_data = RL_state
                visited_times_RL = self._calculate_visited_times(RL_state,self.visited_state_tree)
                mean_RL, var_RL, sigma_RL = self._calculate_statistics_index(RL_state,self.visited_state_value,self.visited_state_tree)

            record_data = np.append(record_data,rew)
            record_data = np.append(record_data,float(done))
            record_data = np.append(record_data,visited_times_rule)
            record_data = np.append(record_data,mean_rule)
            record_data = np.append(record_data,var_rule)
            record_data = np.append(record_data,visited_times_RL)
            record_data = np.append(record_data,mean_RL)
            record_data = np.append(record_data,var_RL)

            self.driving_record_outfile.write(self.driving_record_format % tuple(record_data))


    
