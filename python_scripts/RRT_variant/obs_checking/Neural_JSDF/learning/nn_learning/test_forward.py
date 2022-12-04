import torch
from sdf.robot_sdf import RobotSdfCollisionNet

device = torch.device('cpu')

s = 256
n_layers = 5
skips = []
if skips == []:
    n_layers-=1
tensor_args = {'device': device, 'dtype': torch.float32}
nn_model = RobotSdfCollisionNet(in_channels=10, out_channels=9, layers=[s] * n_layers, skips=skips)
nn_model.load_weights('/Users/vanli/Documents/MATLAB/oat_semester_project_Fan/python_scripts/RRT_variant/obs_checking/Neural_JSDF/learning/nn_learning/sdf_256x5_mesh_origin.pt', tensor_args)
# nn_model.load_weights('/Users/vanli/Downloads/result/sdf_256x5_mesh.pt', tensor_args)

nn_model.model.to(**tensor_args)
model = nn_model.model
# x = torch.tensor([[2.9632,  1.7820, -2.0959, -3.1082,  1.6234,  0.7221, -1.5457, -0.5558, 0.0828,  0.7472]], requires_grad=True)
x = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], requires_grad=True)
output1 = model.forward(x)
a=6