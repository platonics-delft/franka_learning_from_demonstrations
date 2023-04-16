import os
import torch
import cv2

def template_matching(source_tensor, template_tensor):
    # Define the convolution operation
    conv = torch.nn.Conv2d(1, 1, kernel_size=template_tensor.shape[2:], bias=False)
    conv.weight.data = template_tensor

    # Perform template matching
    result_tensor = conv(source_tensor)

    # Find the location of the maximum correlation value
    max_val, max_idx = torch.max(result_tensor.view(-1), dim=0)
    max_loc = torch.tensor([max_idx % result_tensor.shape[3], max_idx // result_tensor.shape[3]])

    # Compute the distance location of the template image with respect to the center of the source image
    source_center = (source_tensor.shape[3] / 2, source_tensor.shape[2] / 2)
    template_center = (max_loc[0] + template_tensor.shape[3] / 2, max_loc[1] + template_tensor.shape[2] / 2)
    distance = (template_center[0] - source_center[0], template_center[1] - source_center[1])
    
    return distance, max_val

def max_correlation(template_tensor):
    # Define the convolution operation
    conv = torch.nn.Conv2d(1, 1, kernel_size=template_tensor.shape[2:], bias=False)
    conv.weight.data = template_tensor

    # Perform template matching
    result_tensor = conv(template_tensor)

    # Find the location of the maximum correlation value
    max_val = torch.max(result_tensor.view(-1), dim=0)

    return max_val

# Define the path to the directory containing the target images that are the demonstration 
target_dir = "demo_images/"
# Load the target images from the demonstration, convert them to grayscale tensors, and move to GPU
demo_img_tensors = []
demo_max_corr=[]
for filename in os.listdir(target_dir):
    img = cv2.imread(os.path.join(target_dir, filename))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tensor = torch.from_numpy(gray).float().unsqueeze(0).unsqueeze(0)
    #tensor = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0)
    if torch.cuda.is_available():
        tensor = tensor.cuda()
    max_val=max_correlation(tensor)    
    demo_max_corr.append(max_val)
    demo_img_tensors.append(tensor)

curr_img = cv2.imread("current_image.jpg")
curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
# Convert images to tensors and move to GPU
if torch.cuda.is_available():   
    curr_img_tensor = torch.from_numpy(curr_gray).float().unsqueeze(0).unsqueeze(0).cuda()
#the source is going to be the current image receive from the robot and the target is the cropped image saved during the demonstration at time step i
i=0
distance,  max_val = template_matching(curr_img_tensor, demo_img_tensors[i]) 
uncertainty = max_val/demo_max_corr[i]

#apply a delta x in the direction of the distance direction after coverting it back in base frame
