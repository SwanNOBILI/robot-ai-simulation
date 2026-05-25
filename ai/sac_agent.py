import torch
import os
from ai.replay_buffer import ReplayBuffer
from ai.actor import Actor
from ai.critic import Critic



class SAC_Agent():
    def __init__(self, state_size, action_size, max_action, buffer_size=1e6, hidden_size=256, 
                 gamma=0.99, tau=0.005, alpha=0.2, lr=3e-4, batch_size=256):
        # Hyperparameters
        self.gamma = gamma              # discount factor (reward longevity rate)
        self.tau = tau                  # soft update rate (update rate of Critics target)
        self.alpha = alpha              # entropy coefficient (explorability vs exploitability)
        self.batch_size = batch_size    # n° of examples used to update once Weights of MLP models
        # Internal objects
        self.replay_buffer = ReplayBuffer(int(buffer_size), state_size, action_size)
        self.actor = Actor(state_size, hidden_size, action_size, max_action)
        self.critic1 = Critic(state_size, hidden_size, action_size)
        self.critic2 = Critic(state_size, hidden_size, action_size)
        # Critics target are copy of Critics, that will learn without a backpropagation (no gradient descend)
        self.critic1_target = Critic(state_size, hidden_size, action_size)
        self.critic2_target = Critic(state_size, hidden_size, action_size)
        self.critic1_target.load_state_dict(self.critic1.state_dict())                  # copy Weights of self.critic1
        self.critic2_target.load_state_dict(self.critic2.state_dict())                  # copy Weights of self.critic2
        for param in self.critic1_target.parameters(): param.requires_grad = False      # self.critic1 gradient descend disabled
        for param in self.critic2_target.parameters(): param.requires_grad = False      # self.critic2 gradient descend disabled
        # Optimizer used for Actor & Critics
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr)
        self.critic1_optimizer = torch.optim.Adam(self.critic1.parameters(), lr)
        self.critic2_optimizer = torch.optim.Adam(self.critic2.parameters(), lr)

    def select_action(self, state):
        with torch.no_grad(): 
            state = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
            action, _ = self.actor.sample(state)
        return action.cpu().numpy()[0]  # delete batch_size --> dim = (action_size,)
    
    def train_step(self):
        # Get replay_buffer values (convert them into tensors)
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(self.batch_size)
        # Compute objective y = r + γ*(1 − d)*(min(Q1(next_st,next_at)​, Q2(next_st,next_at)​) - α*log_π)
        with torch.no_grad():   # disable autogradient -> no Wheights upgrade for self.actor, self.critic1_target, self.critic2_target
            next_actions, next_logs_π = self.actor.sample(next_states)
            q_target1 = self.critic1_target(next_states, next_actions)  # calling .forward() & other useful functionnalities from nn
            q_target2 = self.critic2_target(next_states, next_actions)  # calling .forward() & other useful functionnalities from nn
            y = rewards + self.gamma*(1-dones)*(torch.minimum(q_target1,q_target2)-self.alpha*next_logs_π)
        # Critics update -> L = MSE(Qi(st,at)​, y) [mean-squared error]
        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        loss_critic1 = torch.nn.functional.mse_loss(q1, y)
        loss_critic2 = torch.nn.functional.mse_loss(q2, y)
        self.critic1_optimizer.zero_grad(); loss_critic1.backward(); self.critic1_optimizer.step()
        self.critic2_optimizer.zero_grad(); loss_critic2.backward(); self.critic2_optimizer.step()
        # Actor update -> L = mean(α*log_π − min(Q1​(st​,a_new​), Q2​(st​,a_new​)))
        predicted_actions, predicted_logs_π = self.actor.sample(states)
        predicted_q1 = self.critic1(states, predicted_actions)
        predicted_q2 = self.critic2(states, predicted_actions)
        loss_actor = torch.mean(self.alpha*predicted_logs_π - torch.minimum(predicted_q1,predicted_q2))
        self.actor_optimizer.zero_grad(); loss_actor.backward(); self.actor_optimizer.step()
        # Critics Target update -> θ_target​ = τ*θ + (1−τ)*θ_target​
        for param, target_param in zip(self.critic1.parameters(), self.critic1_target.parameters()):
            target_param.data.copy_(self.tau*param.data + (1-self.tau)*target_param.data)
        for param, target_param in zip(self.critic2.parameters(), self.critic2_target.parameters()):
            target_param.data.copy_(self.tau*param.data + (1-self.tau)*target_param.data)

    def save(self, folder_path):
        torch.save(self.actor.state_dict(), folder_path+"actor.pt")
        torch.save(self.critic1.state_dict(), folder_path+"critic1.pt")
        torch.save(self.critic2.state_dict(), folder_path+"critic2.pt")
        torch.save(self.critic1_target.state_dict(), folder_path+"critic1_target.pt")
        torch.save(self.critic2_target.state_dict(), folder_path+"critic2_target.pt")

    def load(self, folder_path):
        self.actor.load_state_dict(torch.load(os.path.join(folder_path,"actor.pt"),weights_only=True))
        self.critic1.load_state_dict(torch.load(os.path.join(folder_path,"critic1.pt"),weights_only=True))
        self.critic2.load_state_dict(torch.load(os.path.join(folder_path,"critic2.pt"),weights_only=True))
        self.critic1_target.load_state_dict(torch.load(os.path.join(folder_path, "critic1_target.pt"),weights_only=True))
        self.critic2_target.load_state_dict(torch.load(os.path.join(folder_path,"critic2_target.pt"),weights_only=True))