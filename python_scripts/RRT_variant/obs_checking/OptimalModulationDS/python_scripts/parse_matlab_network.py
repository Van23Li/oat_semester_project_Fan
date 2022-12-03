import numpy as np
import scipy.io as sio

import torch
import torch.nn as nn
from torch.autograd import Variable
import torch.nn.functional as F


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.layers = []

    def setWeights(self, Weights, biases):
        with torch.no_grad():
            n_layers = len(Weights)
            for i in range(n_layers):
                tmp_layer = nn.Linear(Weights[i].shape[0], Weights[i].shape[1])
                tmp_layer.weight = nn.Parameter(torch.Tensor(Weights[i]))

                tmp_layer.bias = nn.Parameter(torch.Tensor(biases[i]).squeeze())
                self.layers.append(tmp_layer)

    def forward(self, x):
        #positional encoding
        x = torch.hstack([x, torch.sin(x), torch.cos(x)])
        for layer in self.layers:
            x = layer(x)
            if layer.in_features > 1: #idk why in_features are confused with out_features here, but it works
                x = torch.tanh(x)
        return x

    # def backward(self, x):
    #     y = []
    #     x = torch.hstack([x, torch.sin(x), torch.cos(x)])
    #     for layer in self.layers:
    #         x = layer(x)
    #         y.append(x)
    #         if layer.in_features > 1:
    #             x = torch.tanh(x)
    #
    #     result = []
    #     for num in range(len(y[0])):
    #         for i in range(len(self.layers)):
    #             if i == 0:
    #                 # x = torch.matmul(torch.diag(1 - torch.tanh(y[-1 - i][num,:]) * torch.tanh(y[-1 - i][num,:])), self.layers[-1 - i].weight)
    #                 x = torch.matmul(torch.diagflat(1 - torch.tanh(y[-1 - i]) * torch.tanh(y[-1 - i])), self.layers[-1 - i].weight)
    #             else:
    #                 x = torch.matmul(torch.matmul(x, torch.diag(1 - torch.tanh(y[-1-i][num,:]) * torch.tanh(y[-1-i][num,:]))),self.layers[-1-i].weight)
    #         result.append(x)
    #     return result

#load weights
mat_contents = sio.loadmat('../matlab_scripts/planar_robot_2d/data/net_parsed.mat')
W = mat_contents['W'][0]
b = mat_contents['b'][0]

#create net
net = Net()
net.setWeights(W, b)
x = torch.tensor([[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,2.0],[0.0,0.0,0.0,3.0]], requires_grad=True)
output1 = net.forward(x)

gradient = torch.tensor([[1.0],[1.0],[1.0]])
output1.backward(gradient=gradient) #反向传播
dy2_dx2 = x.grad
print(output1)
print(dy2_dx2)


####################### Original code
# #load weights
# mat_contents = sio.loadmat('../matlab_scripts/planar_robot_7d/data/net_parsed.mat')
# W = mat_contents['W'][0]
# b = mat_contents['b'][0]
#
# #create net
# net = Net()
# net.setWeights(W, b)
#
# DIM = int(W[0].shape[1]/3) #overall input dimension (div3 because positional input x = [x sin(x) cos(x)]
# DOF = DIM - 2 #robot DoF (-2 for planar model)
# rob_pos = torch.zeros(DIM) #zero angles, so robot spans from (0,0) to (0, 7) ,as links = 1
# rob_pos[DOF:] = torch.tensor([8, 0]) #input point at (8,0), so distance = 1
# print(net.forward(rob_pos))
#
# rob_pos[DOF:] = torch.tensor([0, 4]) #input point at (0,4), so distance = 4
# print(net.forward(rob_pos))
#
# torch.save(net.state_dict(), 'data/mlp_state_dict.pt')