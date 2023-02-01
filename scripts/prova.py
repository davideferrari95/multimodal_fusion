import torch
# import torch.nn as nn
from scripts.Rete_neurale import LitNeuralNet
#from pytorch_lightning import Trainer



if __name__ == '__main__':

    model = LitNeuralNet(0.7,0.15,0.15,6,32,64,9)
    state_dict = torch.load("/home/alberto/Scrivania/Codici_Tirocinio/Corso_py/classic/Classificatori/Classic_adv.pth")
    model.load_state_dict(state_dict)
    model.eval()

    input = torch.tensor([0.1,0.8,0.1,0,1,0])     #Comando 6 
    output = model(input)
    prediction = output.argmax(dim=0)
    print("Numero del comando:", prediction.item()) 


