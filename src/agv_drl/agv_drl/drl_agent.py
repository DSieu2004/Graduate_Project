# #!/usr/bin/env python3
# import torch
# import torch.nn as nn
# import torch.optim as optim
# import numpy as np
# from torch.distributions import Normal

# DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# class RolloutBuffer:
#     def __init__(self):
#         self.states, self.actions, self.logprobs, self.rewards, self.is_terminals, self.values = [], [], [], [], [], []
#     def clear(self):
#         del self.states[:], self.actions[:], self.logprobs[:], self.rewards[:], self.is_terminals[:], self.values[:]

# class PPOActorCritic(nn.Module):
#     def __init__(self, action_dim=2):
#         super(PPOActorCritic, self).__init__()
        
#         # 1. Feature Extractor (CNN cho Lidar)
#         self.cnn = nn.Sequential(
#             nn.Conv1d(1, 16, kernel_size=5, stride=2), nn.ReLU(),
#             nn.Conv1d(16, 32, kernel_size=3, stride=2), nn.ReLU(),
#             nn.Flatten()
#         )
        
#         # 2. Shared Layers (Dữ liệu Lidar sau Flatten là 128 + 2 biến Goal = 130)
#         self.shared = nn.Sequential(
#             nn.Linear(128 + 2, 256), nn.ReLU(), 
#             nn.Linear(256, 256), nn.ReLU()
#         )
        
#         # 3. Actor Head: SỬA LẠI THÀNH SEQUENTIAL để khớp với file .pth (tạo ra key .0.)
#         self.actor_mean = nn.Sequential(
#             nn.Linear(256, action_dim)
#         )
        
#         self.actor_logstd = nn.Parameter(torch.zeros(action_dim))
        
#         # 4. Critic Head
#         self.critic = nn.Linear(256, 1)

#     def forward(self, state):
#         # State: [batch, 26] -> 24 tia lidar + 2 biến goal (dist, angle)
#         laser = state[:, :24].unsqueeze(1) # [batch, 1, 24]
#         goal = state[:, 24:] # [batch, 2]
        
#         # Trích xuất đặc trưng Lidar và gộp với Goal
#         features = self.cnn(laser)
#         shared_features = self.shared(torch.cat([features, goal], dim=1))
        
#         # Trả về Mean (cho Actor) và Value (cho Critic)
#         return self.actor_mean(shared_features), self.critic(shared_features)

#     def get_action(self, state):
#         state = torch.FloatTensor(state).unsqueeze(0).to(DEVICE)
#         mean, value = self.forward(state)
#         std = torch.exp(self.actor_logstd)
#         dist = Normal(mean, std)
        
#         action = dist.sample()
#         action_bounded = torch.tanh(action)
        
#         return action_bounded.detach().cpu().numpy()[0], 0.0, value.detach().cpu().item()

# # Class PPOAgent giữ nguyên để đảm bảo cấu trúc nếu cần dùng lại cho việc training/update
# class PPOAgent:
#     def __init__(self, action_dim=2):
#         self.policy = PPOActorCritic(action_dim).to(DEVICE)
#         self.policy_old = PPOActorCritic(action_dim).to(DEVICE)
#         self.policy_old.load_state_dict(self.policy.state_dict())

#!/usr/bin/env python3
import torch
import torch.nn as nn
import numpy as np
from torch.distributions import Normal

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class PPOActorCritic(nn.Module):
    """
    Mạng neural PPO dùng cho inference trên robot thật.
    Input:  state [batch, 26] = 24 tia LiDAR (chuẩn hóa) + 2 biến goal (dist, angle)
    Output: action [v, w] và value (critic)
    """

    def __init__(self, action_dim=2):
        super(PPOActorCritic, self).__init__()

        # 1. Feature Extractor (CNN cho LiDAR)
        self.cnn = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=3, stride=2), nn.ReLU(),
            nn.Flatten()
        )

        # 2. Shared Layers (128 features từ CNN + 2 biến goal = 130)
        self.shared = nn.Sequential(
            nn.Linear(128 + 2, 256), nn.ReLU(),
            nn.Linear(256, 256),     nn.ReLU()
        )

        # 3. Actor Head
        self.actor_mean   = nn.Sequential(nn.Linear(256, action_dim))
        self.actor_logstd = nn.Parameter(torch.zeros(action_dim))

        # 4. Critic Head
        self.critic = nn.Linear(256, 1)

    def forward(self, state):
        # state: [batch, 26] → 24 tia LiDAR + 2 biến goal (dist, angle)
        laser  = state[:, :24].unsqueeze(1)   # [batch, 1, 24]
        goal   = state[:, 24:]                # [batch, 2]

        features        = self.cnn(laser)
        shared_features = self.shared(torch.cat([features, goal], dim=1))

        return self.actor_mean(shared_features), self.critic(shared_features)

    def get_action(self, state):
        """Lấy action trong lúc inference (không cần gradient)"""
        state = torch.FloatTensor(state).unsqueeze(0).to(DEVICE)
        with torch.no_grad():
            mean, value = self.forward(state)
            std         = torch.exp(self.actor_logstd)
            dist        = Normal(mean, std)
            action      = dist.sample()
            action_bounded = torch.tanh(action)
        return action_bounded.cpu().numpy()[0], 0.0, value.cpu().item()