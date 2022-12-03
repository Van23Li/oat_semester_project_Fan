import torch
from sdf.robot_sdf import RobotSdfCollisionNet

s = 256
n_layers = 5
skips = []
device = torch.device('cpu')
tensor_args = {'device': device, 'dtype': torch.float32}
nn_model = RobotSdfCollisionNet(in_channels=10, out_channels=9, layers=[s] * n_layers, skips=skips)
nn_model.load_weights('sdf_256x5_mesh.pt', tensor_args)

# nn_model.model.to(**tensor_args)
model = nn_model.model
x = torch.tensor([[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.5,0]], requires_grad=True)
output1 = model.forward(x)


a=3