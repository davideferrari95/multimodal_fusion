#!/usr/bin/env python3

import os
import pandas as pd

# Import PyTorch
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset, random_split

# Import PyTorch Lightning
from pytorch_lightning import Trainer, LightningModule
from pytorch_lightning.callbacks.early_stopping import EarlyStopping

# Project Folder (ROOT Project Location)
PROJECT_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

# Get Torch Device ('cuda' or 'cpu')
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# Set Torch Matmul Precision
torch.set_float32_matmul_precision('high')

# Hyper-Parameters (15 Labels)
input_size, hidden_size, output_size = 2, [32, 64], 15
train_percent, val_percent, test_percent = 0.8, 0.1, 0.1
num_epochs, batch_size, learning_rate = 1500, 64, 0.004

class CSVDataset(Dataset):

    """ Create a PyTorch Dataset from a CSV file """

    def __init__(self):

        # Read the CSV file
        df = pd.read_csv(f'{PROJECT_FOLDER}/dataset/fusion_dataset.csv', delimiter=',', skiprows=1)

        self.n_samples = df.shape[0]

        # Size | X: [n_samples, n_features] | Y: [n_samples, 1]
        self.x_data = torch.from_numpy(df.iloc[:, 1:].values).float()
        self.y_data = torch.from_numpy(df.iloc[:, 0].values)

        # Move to GPU
        self.x_data.to(DEVICE)
        self.y_data.to(DEVICE)

    def __len__(self) -> int:
        return self.n_samples

    def __getitem__(self, idx):
        return self.x_data[idx], self.y_data[idx]

def prepareDataloaders(batch_size:int, train_set_size:float, validation_set_size:float, test_set_size:float):

    """ Prepare Dataloaders """

    # Load the data from the CSV file and split it into training, validation and test sets based on the specified percentages
    dataset = CSVDataset()

    # Split Dataset
    assert train_set_size + validation_set_size + test_set_size <= 1, 'Train + Validation + Test Set Size must be less than 1'
    train_data, val_data, test_data = random_split(dataset, [train_set_size, validation_set_size, test_set_size], generator=torch.Generator())
    assert len(train_data) + len(val_data) + len(test_data) == len(dataset), 'Train + Validation + Test Set Size must be equal to Dataset Size'

    # Create data loaders for training and testing
    train_dataloader = DataLoader(train_data, batch_size=batch_size, num_workers=os.cpu_count(), shuffle=True)
    val_dataloader   = DataLoader(val_data,   batch_size=batch_size, num_workers=os.cpu_count(), shuffle=False)
    test_dataloader  = DataLoader(test_data,  batch_size=batch_size, num_workers=os.cpu_count(), shuffle=False)

    return train_dataloader, val_dataloader, test_dataloader

class LitNeuralNet(LightningModule):

    """ PyTorch Lightning Neural Network """

    def __init__(self, train_percent: float, val_percent: float, test_percent: float, input_size=2, hidden_size=[32,64], output_size=15, transform=None):

        super(LitNeuralNet, self).__init__()

        # Define Network Layers
        self.fc1 = nn.Linear(input_size, hidden_size[0])
        self.fc2 = nn.Linear(hidden_size[0], hidden_size[1])
        self.fc3 = nn.Linear(hidden_size[1], output_size)

        # Define Hyper-Parameters
        self.train_percent, self.val_percent, self.test_percent = train_percent, val_percent, test_percent
        self.transform = transform

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

if __name__ == '__main__':

    # Prepare Dataset and Dataloaders
    train_dataloader, val_dataloader, test_dataloader = prepareDataloaders(batch_size, train_percent, val_percent, test_percent)

    # Init PyTorch Lightning Trainer
    trainer = Trainer(

        # Devices
        devices = 'auto',
        accelerator = 'auto',

        # Some Parameters
        max_epochs = num_epochs,
        log_every_n_steps = 10,

        # Early Stopping Callback
        callbacks = [EarlyStopping(monitor="train_loss", mode='min', patience=100, min_delta=0, verbose=True)],

        fast_dev_run = False

    )

    # Init PyTorch Lightning Model
    model = LitNeuralNet(input_size, hidden_size, output_size)
    # compiled_model = torch.compile(model)

    # Train the Model
    trainer.fit(model, train_dataloaders=train_dataloader, val_dataloaders=val_dataloader)

    # Save the Model
    MODEL_FILE = f'{PROJECT_FOLDER}/model/fusion_model.pth'
    torch.save(model.state_dict(), MODEL_FILE)

    print('Training Done!')
