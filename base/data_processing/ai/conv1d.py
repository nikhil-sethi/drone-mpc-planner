import torch
from torch import nn


class CNN(torch.nn.Module):
    '''
    This is the Convolutional Neural Network.
    It gives userwarning about the MaxPool1D since the last pytorch update. This is a similar issue:
    https://github.com/pytorch/pytorch/issues/60053
    '''

    def __init__(self, amount_classes=3, features=6):
        super(CNN, self).__init__()
        self.amount = int(amount_classes)
        self.layer1 = nn.Sequential(
            nn.Conv1d(in_channels=features, out_channels=16, kernel_size=7, stride=1, padding=2),
            nn.BatchNorm1d(16),
            nn.LeakyReLU(),
            nn.MaxPool1d(kernel_size=2)
        )
        self.layer2 = nn.Sequential(
            nn.Conv1d(in_channels=16, out_channels=64, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm1d(64),
            nn.LeakyReLU(),
            nn.MaxPool1d(kernel_size=2),
        )
        self.layer3 = nn.Sequential(
            nn.Conv1d(in_channels=64, out_channels=32, kernel_size=7, stride=1, padding=2),
            nn.BatchNorm1d(32),
            nn.LeakyReLU(),
            nn.MaxPool1d(kernel_size=2),
        )
        self.layer4 = nn.Sequential(
            nn.Conv1d(in_channels=32, out_channels=8, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm1d(8),
            nn.LeakyReLU(),
            nn.MaxPool1d(kernel_size=2),
        )
        self.mlp = nn.Sequential(
            nn.Linear(40, 64),  # 45:16, 90:40, 180:80
            nn.LeakyReLU(),
            nn.Linear(64, 1),
        )
        self.multimlp = nn.Sequential(
            nn.Linear(40, 64),  # 45:16, 90:40, 180:80
            nn.LeakyReLU(),
            nn.Linear(64, self.amount),
        )
        self.drop_out = torch.nn.Dropout(0.5)
        self.out = nn.Sigmoid()
        self.multi_out = nn.Softmax(dim=1)

    def forward(self, x):
        x = x.permute(0, 2, 1)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)

        x = self.drop_out(x)
        # change with sequence size = 2 - 5 - 10
        x = x.view(-1, 8 * 5)

        if self.amount == 2:
            out = self.mlp(x)
            return self.out(out.squeeze()), x
        else:
            out = self.multimlp(x)
            return self.multi_out(out), x
