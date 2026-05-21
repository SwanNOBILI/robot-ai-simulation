import numpy as np



class ReplayBuffer:
    def __init__(self, max_size, state_size, action_size):
        self.states = np.zeros((max_size, state_size))       # (distance_to_goal(1), angle_to_goal(1), sensor_values(8), ​ωl(1)​, ​ωr(1)​)
        self.actions = np.zeros((max_size, action_size))     # (​ωl(1), ​ωr(1)) -> (left_motor_speed, right_motor_speed)
        self.rewards = np.zeros((max_size, 1))
        self.next_states = np.zeros((max_size, state_size))  # same as states
        self.dones = np.zeros((max_size, 1))                # goal_reached is True or no_progression_time > threshold (15.0 for now)
        # Internal variables
        self.__max_size = max_size  # variable starting by "__" means it is a private variable
        self.__ptr = 0
        self.__size = 0
    
    # Returns the current ReplayBuffer size
    def __len__(self):
        return self.__size
    
    # Add a new experience in the ReplayBuffer
    def add(self, state, action, reward, next_state, done):
        self.states[self.__ptr] = state
        self.actions[self.__ptr] = action
        self.rewards[self.__ptr] = reward
        self.next_states[self.__ptr] = next_state
        self.dones[self.__ptr] = done
        if self.__size < self.__max_size: self.__size += 1  # RelayBuffer size
        self.__ptr = (self.__ptr + 1) % self.__max_size     # Next tuple addr
    
    # Gives a random batch from the ReplayBuffer
    def sample(self, batch_size):
        rand_indices = np.random.randint(0, self.__size, size=batch_size)
        return (self.states[rand_indices], self.actions[rand_indices], self.rewards[rand_indices], 
                self.next_states[rand_indices], self.dones[rand_indices])