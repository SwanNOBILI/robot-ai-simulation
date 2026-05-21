import torch



class Actor(torch.nn.Module):
    def __init__(self, state_size, hidden_size, action_size, max_action):
        super().__init__()
        self.fc1 = torch.nn.Linear(state_size, hidden_size)         # input_size = 12
        self.fc2 = torch.nn.Linear(hidden_size, hidden_size)        # hidden_size (hyperparameter)
        self.mean = torch.nn.Linear(hidden_size, action_size)       # mean of Gaussian functions (1 function per action)
        self.log_std = torch.nn.Linear(hidden_size, action_size)    # log of standard deviation of Gaussian functions
        self.max_action = max_action

    def forward(self, state):
        x = torch.relu(self.fc1(state))                             # activation function (hyperparameter)
        x = torch.relu(self.fc2(x))
        mean = self.mean(x)
        log_std = torch.clamp(self.log_std(x), min=-20, max=2)      # avoids extreme values, as we will apply "torch.exp" afterwards
        return mean, log_std
    
    # Gives an action & probability of this action
    def sample(self, state):
        mean, log_std = self.forward(state)
        normal = torch.distributions.Normal(mean, log_std.exp())    # Gaussian Normal distribution
        x = normal.rsample()                                        # reparameterization trick (to keep the operation differentiable)
        action = torch.tanh(x)                                      # boundaries [-1, 1]
        action_scaled = action * self.max_action                    # boundaries [-self.max_action, self.max_action]
        # log_π(a) = log_π(x) - ∑(log(1−tanh²(xi​))) --> p(a) = p(x)*(dx/da) = p(x)*(1/f'(x)) --> log(p(a)) = log(p(x)) - log(f'(x)))
        log_π = normal.log_prob(x)-torch.log(1-action.pow(2)+1e-6)  # 1e-6 avoids log(0) as "1+1e-6" is out of tanh(x) boundaries
        # Independant actions --> p(a1,a2) = p(a1)*p(a2) --> log(p(a1,a2)) = log(p(a1)) + log(p(a2)) = ∑(log(p(ai)))
        log_π = torch.sum(log_π, dim=1, keepdim=True)               # convert (batch_size*action_size) into (batch_size*1)
        return action_scaled, log_π