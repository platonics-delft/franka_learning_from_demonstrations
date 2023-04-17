import os
import torch
import torch.nn.functional as F
import cv2
import pdb
def template_matching(source_tensor, template_tensor):
    # Define the convolution operation
    source_tensor = source_tensor.float()
    template_tensor = template_tensor.float()
    conv = torch.nn.Conv2d(1, 1, kernel_size=template_tensor.shape[2:], bias=False)
    conv.weight.data = template_tensor #.view(1,1, *template_tensor.shape)
    # pdb.set_trace()
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

def template_matching_SSD(search_image, template):
    # Convert images to tensors
    template = template.float()
    search_image = search_image.float()

    cross_corr = F.conv2d(search_image, template.pow(2)) - 2 * F.conv2d(search_image, template) + F.conv2d(search_image.pow(2), torch.ones_like(template))

    max_corr, max_idx = torch.min(cross_corr.view(-1), 0)
    x, y = torch.tensor([max_idx % cross_corr.shape[3], max_idx // cross_corr.shape[3]])

    return x, y, max_corr

def max_correlation(template_tensor):
    # Define the convolution operation
    conv = torch.nn.Conv2d(1, 1, kernel_size=template_tensor.shape[2:], bias=False)
    conv.weight.data = template_tensor

    # Perform template matching
    template_mult = template_tensor * template_tensor

    max_val = torch.sum(template_mult.view(-1), dim=0)

    return max_val

# # Define the path to the directory containing the target images that are the demonstration 
# target_dir = "demo_images/"
# # Load the target images from the demonstration, convert them to grayscale tensors, and move to GPU
# demo_img_tensors = []
# demo_max_corr=[]
# for filename in os.listdir(target_dir):
#     img = cv2.imread(os.path.join(target_dir, filename))
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     tensor = torch.from_numpy(gray).float().unsqueeze(0).unsqueeze(0)
#     #tensor = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0)
#     if torch.cuda.is_available():
#         tensor = tensor.cuda()
#     max_val=max_correlation(tensor)    
#     demo_max_corr.append(max_val)
#     demo_img_tensors.append(tensor)

# curr_img = cv2.imread("current_image.jpg")
# curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
# # Convert images to tensors and move to GPU
# if torch.cuda.is_available():   
#     curr_img_tensor = torch.from_numpy(curr_gray).float().unsqueeze(0).unsqueeze(0).cuda()
# #the source is going to be the current image receive from the robot and the target is the cropped image saved during the demonstration at time step i
# i=0
# distance,  max_val = template_matching(curr_img_tensor, demo_img_tensors[i]) 
# uncertainty = max_val/demo_max_corr[i]

#apply a delta x in the direction of the distance direction after coverting it back in base frame
