import torch
# import torch.nn as nn
from Rete_neurale_classificatrice import LitNeuralNet
#from pytorch_lightning import Trainer



if __name__ == '__main__':

    model = LitNeuralNet(0.8,0.1,0.1,2,32,64,35)
    state_dict = torch.load("/home/alberto/catkin_ws/src/fusion/model/fusion_model.pth")
    model.load_state_dict(state_dict)
    model.eval()

    input = torch.tensor([0,40], dtype= torch.float32)     # 
    output = model(input)
    prediction = output.argmax(dim=0)
    print("Numero del comando:", prediction.item()) 
