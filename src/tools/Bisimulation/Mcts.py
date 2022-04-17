import os
import os.path as osp
import random
import numpy as np
from rtree import index as rindex
from collections import deque



class MCTS(object):

    def __init__(self,
                obs_dimension,
                action_dimension,
                 visited_times_thres = 5,
                 is_training = True,
                 debug = True,
                 save_new_data = True,
                 create_new_train_file = True,
                 create_new_record_file = True,
                 save_new_driving_data = True):

        self.visited_times_thres = visited_times_thres
        self.is_training = is_training
        self.trajectory_buffer = deque(maxlen=20)
        self.debug = debug
        self.save_new_data = save_new_data
        self.create_new_train_file = create_new_train_file
        self.create_new_record_file = create_new_record_file
        self.save_new_driving_data = save_new_driving_data
        self.obs_dimension = obs_dimension
        self.action_dimension = action_dimension
        self.gamma = 0.95
        self._setup_data_saving()
        
    def _setup_data_saving(self):

        if self.create_new_train_file:
            if osp.exists("state_index.dat"):
                os.remove("state_index.dat")
                os.remove("state_index.idx")
            if osp.exists("visited_state.txt"):
                os.remove("visited_state.txt")
            if osp.exists("visited_value.txt"):
                os.remove("visited_value.txt")

            self.visited_state_value = []
            self.visited_state_counter = 0
        else:
            self.visited_state_value = np.loadtxt("visited_value.txt")
            self.visited_state_value = self.visited_state_value.tolist()
            self.visited_state_counter = len(self.visited_state_value)

        self.visited_state_outfile = open("visited_state.txt", "a")
        self.visited_state_format = " ".join(("%f",)*(self.obs_dimension+1))+"\n"

        self.visited_value_outfile = open("visited_value.txt", "a")
        self.visited_value_format = " ".join(("%f",)*2)+"\n"

        visited_state_tree_prop = rindex.Property()
        visited_state_tree_prop.dimension = self.obs_dimension+1
        # ego_x, ego_y, ego_vx, ego_vy, 0f_x, 0f_y, 0f_vx, 0f_vy, 1f_x, 1f_y, 
        # 1f_vx, 1f_vy, 0r_x, 0r_y, 0r_vx, 0r_vy, 1r_x, 1r_y, 1r_vx, 1r_vy
        # 0, -1~2, 
        self.visited_state_dist = np.array([[1, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1, 10, 0.3, 3, 1,10, 0.3, 3, 1, 0.1]])
        self.visited_state_tree = rindex.Index('state_index',properties=visited_state_tree_prop)

        if self.create_new_record_file:
            if osp.exists("driving_record.txt"):
                os.remove("driving_record.txt")
        self.driving_record_outfile = open("driving_record.txt","a")
        self.driving_record_format = " ".join(("%f",)*(self.obs_dimension+9))+"\n"

    def act(self, obs, rule_action, agent, stage):
        if stage == 1:
            return self.act_train(obs, rule_action)
        elif stage == 2:
            return self.act_test(obs, rule_action, agent)
    
    def act_train(self,obs, rule_action):
        if self.should_use_rule(obs, rule_action): 
            return np.array(rule_action)
        else:
            # random action
            return np.array(random.randint(1, self.action_dimension[0])) 
    
    def state_with_action(self,obs,action):
        # print("--------",obs,type(obs),action,type(action))
        return np.append(obs, action)

    def should_use_rule(self,obs, rule_action):
        """
        Whether the state should use rule action
        """
        # Rule action not explore enough

        rule_state = self.state_with_action(obs,rule_action)
        visited_times_rule = self._calculate_visited_times(rule_state,self.visited_state_tree)
        mean_rule, var_rule, sigma_rule = self._calculate_statistics_index(rule_state,
                                                                    self.visited_state_value,
                                                                    self.visited_state_tree) 

        print("MCTS: visited_times_rule, mean_rule", visited_times_rule, mean_rule)
        # if self.debug:
        #     print("rule visited times:",visited_times_rule)

        if visited_times_rule < self.visited_times_thres:
            return True

        # Rule perform good
        # mean_rule in (-1,0)
        explore_motivation = random.uniform(-1,0)
        if explore_motivation < mean_rule:
            return True
        return False

    def act_test(self, obs, rule_action, bisim_agent):
        
        rule_state = self.state_with_action(obs,rule_action)
        visited_times_rule = self._calculate_visited_times(rule_state,self.visited_state_tree)
        mean_rule, var_rule, sigma_rule = self._calculate_statistics_index(rule_state,
                                                                    self.visited_state_value,
                                                                    self.visited_state_tree) 
        if mean_rule < -0.1:
            # find the most similar states in the library
            d_min = 10000
            for state_normal_with_action in self.visited_state_tree:
                state_normal = np.delete(state_normal_with_action, -1)
                action_normal = state_normal_with_action[-1]
                d = bisim_agent.calculate_bisimulation_optimal(obs, state_normal, action_normal)
                if d < d_min:
                    d_min = d
                    bisim_action = action_normal
            # theory bound
            if d_min + mean_rule > 0:
                return np.array(bisim_action)
        else:
            return np.array(rule_action)

    ############## RLS Confidence ##############

    def _calculate_visited_times(self, obs, visited_state_tree):

        return sum(1 for _ in visited_state_tree.intersection(obs.tolist()))

    def _calculate_statistics_index(self, obs, visited_state_value, visited_state_tree):
        """
        Calculate statistics_idx
        """
        if self._calculate_visited_times(obs,visited_state_tree) == 0:
            return 10, 10, 10

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
                    tuple((state_to_record-self.visited_state_dist).tolist()[0]+(state_to_record+self.visited_state_dist).tolist()[0]))
                self.visited_state_outfile.write(self.visited_state_format % tuple(state_to_record))
                self.visited_value_outfile.write(self.visited_value_format % tuple([action_to_record,r_to_record]))
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
                    self.visited_state_outfile.write(self.visited_state_format % tuple(state_to_record))
                    self.visited_value_outfile.write(self.visited_value_format % tuple([action_to_record,r_to_record]))
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


    