from model import CNN
import pyrealsense2 as rs
import torch
import torch.nn as nn
import numpy as np

# Define your CNN and loss function
net = CNN()
criterion = nn.MSELoss()
optimizer = torch.optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

# Load images
images_array = np.load("../../data/test1.npz")

# Define the training loop
for epoch in range(10):
    running_loss = 0.0
    for i in range(100):

        # Preprocess the image and label
        image = images_array[:, :]
        image = torch.from_numpy(color_image.transpose((2, 0, 1))).float() / 255.0
        label = torch.tensor(0).unsqueeze(0)

        # Zero the parameter gradients
        optimizer.zero_grad()

        # Forward + backward + optimize
        outputs = net(image.unsqueeze(0))
        loss = criterion(outputs, label)
        loss.backward()
        optimizer.step()

        # Print statistics
        running_loss += loss.item()
        if i % 10 == 9:    # print every 10 mini-batches
            print('[%d, %5d] loss: %.3f' %
                  (epoch + 1, i + 1, running_loss / 10))
            running_loss = 0.0