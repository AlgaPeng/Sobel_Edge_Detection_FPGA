import numpy as np
from PIL import Image

def read_pixel_values_from_file(file_path):
    pixel_values = []
    pixel_time = []
    start_cal = 0
    with open(file_path, "r") as file:
        for line in file:
            if("2380832500" in line):
                start_cal = 1
            if(start_cal):
                if("x" in line):
                    columns = line.strip().split()
                    pixel_time.append(int(columns[0]))
                    break
                columns = line.strip().split()
                pixel_values.append(int(columns[2], 2))
                pixel_time.append(int(columns[0]))    
    return pixel_values, pixel_time

def cal_full_img(pixel_values, pixel_time):
    full_img = []
    for i in range(len(pixel_values)):
        # print((pixel_time[i+1] - pixel_time[i]))
        duration = int((pixel_time[i+1] - pixel_time[i])/1000)
        # print(duration)
        for _ in range(duration):
            full_img.append(pixel_values[i])
    return full_img

def display_image_from_pixel_values(pixel_values, width, height):
    # Convert the list of pixel values to a NumPy array
    pixel_array = np.array(pixel_values, dtype=np.uint8)

    # Reshape the array to match the image dimensions
    pixel_array = pixel_array.reshape((height, width))

    # Create an image from the NumPy array and display it
    image = Image.fromarray(pixel_array, mode="L")
    image.show()

# Replace this path with your actual input text file path
input_file_path = "list_w30"

pixel_values, pixel_time = read_pixel_values_from_file(input_file_path)
full_img = cal_full_img(pixel_values, pixel_time)
display_image_from_pixel_values(full_img, width=318, height=238)
# [0:75684]
# [0:75366]