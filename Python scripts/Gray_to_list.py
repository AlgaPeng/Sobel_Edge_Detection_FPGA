from PIL import Image

def save_pixel_values_to_file(image_path, output_path):
    # Open the image and convert to grayscale (just in case it's not already)
    image = Image.open(image_path).convert("L")

    # Ensure the image has the expected dimensions
    if image.size != (320, 240):
        raise ValueError("Image size must be 320x240")

    # Get pixel values
    pixel_values = list(image.getdata())

    # Write pixel values to the output text file
    with open(output_path, "w") as output_file:
        for pixel in pixel_values:
            output_file.write(str(pixel) + "\n")

# Replace these paths with your actual image and output file paths
image_path = "8-bit_8-bit_Md_edge_320x240.png"
output_path = "edge_pixel_values.txt"

save_pixel_values_to_file(image_path, output_path)