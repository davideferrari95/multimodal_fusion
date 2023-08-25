#!/usr/bin/env python3

import os
import pandas as pd

# Import PyTorch
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

# Import PyTorch Lightning
import pytorch_lightning as pl
from pytorch_lightning import Trainer
from pytorch_lightning.callbacks.early_stopping import EarlyStopping

# Project Folder (ROOT Project Location)
PROJECT_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

# Hyper-Parameters (15 Labels)
input_size, hidden_size, output_size = 2, [32, 64], 15
train_percent, val_percent, test_percent = 0.8, 0.1, 0.1
num_epochs      = 1500
learning_rate   = 0.004

class CSVDataset(Dataset):

    """ Create a PyTorch Dataset from a CSV file """

    def __init__(self):

        # Read the CSV file
        df = pd.read_csv(f'{PROJECT_FOLDER}/dataset/fusion_dataset.csv', delimiter=',', skiprows=1)

        self.n_samples = df.shape[0]

        # Size | X: [n_samples, n_features] | Y: [n_samples, 1]
        self.x_data = torch.from_numpy(df.iloc[:, 1:].values).float()
        self.y_data = torch.from_numpy(df.iloc[:, 0].values).long()

    def __len__(self):

        return self.n_samples

    def __getitem__(self, index):

        return self.x_data[index], self.y_data[index]

class LitNeuralNet(pl.LightningModule):

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

    def setup(self):

        # Load the data from the CSV file and split it into training, validation and test sets based on the specified percentages
        dataset = CSVDataset()

        # Compute the number of samples for each set
        num_train, num_val, num_test = int(len(dataset) * self.train_percent), int(len(dataset) * self.val_percent), int(len(dataset) * self.test_percent)
        # print("num_train:", num_train, "num_val:", num_val, "num_test:", num_test)

        # Split the dataset into training, validation and test sets
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

        # DataLoader for the Training Set
        return DataLoader(self.train_dataset, batch_size=64, num_workers = 12, shuffle=True)

    def val_dataloader(self):

        # DataLoader for the Validation Set
        return DataLoader(self.val_dataset, batch_size=64, num_workers = 12, shuffle=False)

    def test_dataloader(self):

        # DataLoader for the Test Set
        return DataLoader(self.test_dataset, batch_size=64, num_workers = 12, shuffle=False)

if __name__ == '__main__':

    # Init PyTorch Lightning Trainer
    trainer = Trainer(

        # Some Parameters
        max_epochs = num_epochs, 
        log_every_n_steps=10, 
        auto_lr_find=True, 
        fast_dev_run = False, 

        # Early Stopping Callback
        callbacks = [EarlyStopping(monitor="train_loss", patience = 100, mode = "min", min_delta = 0.01)]

    )

    # Init PyTorch Lightning Model
    model = LitNeuralNet(train_percent, val_percent, test_percent, input_size, hidden_size, output_size)
    # compiled_model = torch.compile(model)

    # Train the Model
    trainer.fit(model)

    # Save the Model
    MODEL_FILE = f'{PROJECT_FOLDER}/model/fusion_model.pth'
    torch.save(model.state_dict(), MODEL_FILE)

    print('Training Done!')
