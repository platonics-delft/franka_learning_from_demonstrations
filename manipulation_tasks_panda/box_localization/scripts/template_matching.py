import torch
import cv2
import numpy as np

def template_matching(target_img, template_img, angle_range):
    # Convert images to tensors and move to GPU if available
    template_tensor = torch.from_numpy(template_img).float().permute(2, 0, 1).unsqueeze(0)
    target_tensor = torch.from_numpy(target_img).float().permute(2, 0, 1).unsqueeze(0)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    template_tensor = template_tensor.to(device)
    target_tensor = target_tensor.to(device)

    # Define convolutional layer with template tensor as kernel
    conv_layer = torch.nn.Conv2d(in_channels=3, out_channels=1, kernel_size=template_tensor.shape[-2:])

    # Initialize convolutional layer with template tensor as kernel
    with torch.no_grad():
        conv_layer.weight.data = template_tensor.mean(dim=1, keepdim=True)
        conv_layer.bias.data = torch.tensor([0.0])

    # Compute padding needed to handle smaller template image
    h_diff = target_tensor.shape[-2] - template_tensor.shape[-2]
    w_diff = target_tensor.shape[-1] - template_tensor.shape[-1]
    top = h_diff // 2
    bottom = h_diff - top
    left = w_diff // 2
    right = w_diff - left
    padding = (left, right, top, bottom)

    # Pad target tensor to match size of template tensor
    target_tensor = torch.nn.functional.pad(target_tensor, padding)

    # Rotate template tensor by all angles
    rotated_template_tensors = []
    for angle in angle_range:
        # Rotate template tensor by current angle
        rotated_template_tensor = torch.tensor(cv2.rotate(template_img, cv2.ROTATE_90_CLOCKWISE)).float().permute(2, 0, 1).unsqueeze(0)
        rotated_template_tensor = torch.nn.functional.rotate(rotated_template_tensor, angle, mode='nearest')
        rotated_template_tensors.append(rotated_template_tensor)

    # Concatenate rotated template tensors along new dimension
    rotated_template_tensor = torch.cat(rotated_template_tensors, dim=0)

    # Move rotated template tensor to GPU
    rotated_template_tensor = rotated_template_tensor.to(device)

    # Initialize convolutional layer with rotated template tensor as kernel
    with torch.no_grad():
        conv_layer.weight.data = rotated_template_tensor.mean(dim=1, keepdim=True)
        conv_layer.bias.data = torch.tensor([0.0])

    # Perform convolution on padded target tensor
    result_tensor = conv_layer(target_tensor)

    # Find maximum response and location in result tensor
    max_value, max_index = torch.max(result_tensor.view(result_tensor.shape[0], -1), dim=1)
    max_index_2d = torch.unravel_index(max_index, result_tensor.shape[-2:])
    max_index_2d = torch.stack(max_index_2d, dim=1)

    # Convert location to original image coordinates
    max_locations = max_index_2d - torch.tensor([left, top])

    # Find index of maximum response across all rotations
    max_index = torch.argmax(max_value)

    # Compute the best angle
    best_angle = angle_range[max_index]

    # Return location of maximum response and best angle
    return tuple(max_locations[max_index].tolist()), best_angle
