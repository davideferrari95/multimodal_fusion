#!/usr/bin/env python3

import torch
import pytorch_lightning as pl
import torch.nn as nn
import numpy as np
import pandas as pd
import torch.nn.functional as F
from pytorch_lightning import Trainer
from torch.utils.data import TensorDataset, DataLoader, Dataset
from pytorch_lightning.callbacks.early_stopping import EarlyStopping


# Hyper-parameters 
input_size = 2       
hidden_size1 = 32        
hidden_size2 = 64      
output_size = 35
num_epochs = 1500              
learning_rate = 0.004
train_percent=0.8
val_percent=0.1
test_percent=0.1

#Class to use a CSV file as a dataset 
class CSVDataset(Dataset):
    def __init__(self):

        # Initialize data, download, etc.
        df = pd.read_csv("/home/alberto/catkin_ws/src/fusion/dataset/fusion_dataset.csv",delimiter=',',skiprows= 1)

        self.n_samples = df.shape[0]

        self.x_data = torch.from_numpy(df.iloc[:, 1:].values).float()# size [n_samples, n_features]
        self.y_data = torch.from_numpy(df.iloc[:, 0].values).long() # size [n_samples, 1]
    
    def __len__(self):
        return self.n_samples
    
    def __getitem__(self, index):
        return self.x_data[index], self.y_data[index]


class LitNeuralNet(pl.LightningModule):

    def __init__(self, train_percent: float, val_percent: float, test_percent: float, input_size, hidden_size1, hidden_size2,output_size, transform=None):
        super(LitNeuralNet, self).__init__()

        #Definiamo gi strati della rete neurale 
        self.fc1 = nn.Linear(input_size, hidden_size1)
        self.fc2 = nn.Linear(hidden_size1, hidden_size2)
        self.fc3 = nn.Linear(hidden_size2, output_size)

        #Definiamo le percentuali di divisione del dataset tra allenamento test e validazione 
        self.train_percent = train_percent
        self.val_percent = val_percent
        self.test_percent = test_percent
        self.transform = transform
    
    def setup(self, stage: str):

        # Carica i dati dal file CSV e li divide in set di addestramento, validazione e test in base alle percentuali specificate
        dataset = CSVDataset()
        num_train = int(len(dataset) * self.train_percent)
        num_val = int(len(dataset) * self.val_percent)
        num_test = int(len(dataset) * self.test_percent)

        #print("num_train:", num_train, "num_val:", num_val, "num_test:", num_test)
        self.train_dataset, self.val_dataset, self.test_dataset = torch.utils.data.random_split(dataset, [num_train, num_val, num_test])
       

    def forward(self, x):

        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr = learning_rate)
        return optimizer

    def training_step(self, batch, batch_idx):
        x, y = batch
        y_pred = self(x)
        loss = F.cross_entropy(y_pred, y)
        self.log("train_loss", loss)
        return {'loss': loss}

    def validation_step(self, batch, batch_idx):
        x, y = batch
        y_pred = self(x)
        loss = F.cross_entropy(y_pred, y)
        self.log("val_loss", loss)
        return {'val_loss': loss}

    def test_step(self, batch, batch_idx):
        x, y = batch
        y_pred = self(x)
        loss = F.cross_entropy(y_pred, y)
        return {'test_loss': loss}

    def train_dataloader(self):
        # Crea un DataLoader per il dataset di addestramento
        return DataLoader(self.train_dataset, batch_size=64, num_workers = 12, shuffle=True)

    def val_dataloader(self):
        # Crea un DataLoader per il dataset di validazione
        return DataLoader(self.val_dataset, batch_size=64, num_workers = 12, shuffle=False)

    def test_dataloader(self):
        # Crea un DataLoader per il dataset di test
        return DataLoader(self.test_dataset, batch_size=64, num_workers = 12, shuffle=False)

    
def train():
    trainer = Trainer(
        auto_lr_find=True, 
        max_epochs = num_epochs, 
        fast_dev_run = False, 
        log_every_n_steps=10, 
        callbacks=[EarlyStopping(monitor="train_loss", patience = 100, mode = "min", min_delta = 0.01)]
         )    
    model = LitNeuralNet(train_percent, val_percent, test_percent, input_size, hidden_size1, hidden_size2, output_size)
    # compiled_model = torch.compile(model)
    trainer.fit(model)
    #trainer.logger = False

    FILE = "/home/alberto/catkin_ws/src/fusion/model/fusion_model.pth"
    torch.save(model.state_dict(), FILE)


if __name__ == '__main__':

    train()
    print("Done! (Speramm a crist)")