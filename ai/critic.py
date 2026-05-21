import torch



class Critic(torch.nn.Module):
    def __init__(self, state_size, hidden_size, action_size):
        super().__init__()
        self.fc1 = torch.nn.Linear(state_size+action_size, hidden_size) # input_size = 12 + 2 = 14
        self.fc2 = torch.nn.Linear(hidden_size, hidden_size)            # hidden_size (hyperparameter)
        self.fc3 = torch.nn.Linear(hidden_size, 1)                      # output_size = 1, Q-value prediction (score of a given action)

    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        x = torch.relu(self.fc1(x))     # activation function (hyperparameter)
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x