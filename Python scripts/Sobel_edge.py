from PIL import Image, ImagePalette
import numpy as np

def create_8bit_color_palette():
    palette = []
    for r in range(8):
        for g in range(8):
            for b in range(4):
                palette.extend([(r << 5) | (r << 2), (g << 5) | (g << 2), (b << 6) | (b << 4)])
    return palette

def apply_sobel_filter(grayscale_image):
    width, height = grayscale_image.size
    output_image = Image.new('P', (width, height))

    # Create a grayscale color palette
    custom_palette = ImagePalette.ImagePalette('gray', create_8bit_color_palette())
    output_image.putpalette(custom_palette)

    # Sobel kernels
    Gx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    Gy = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

    for x in range(1, width - 1):
        for y in range(1, height - 1):
            # Apply the Sobel kernels to the surrounding pixels
            region = np.array([
                [grayscale_image.getpixel((x - 1, y - 1))>>5<<5, grayscale_image.getpixel((x, y - 1))>>5<<5, grayscale_image.getpixel((x + 1, y - 1))>>5<<5],
                [grayscale_image.getpixel((x - 1, y))>>5<<5, grayscale_image.getpixel((x, y))>>5<<5, grayscale_image.getpixel((x + 1, y))>>5<<5],
                [grayscale_image.getpixel((x - 1, y + 1))>>5<<5, grayscale_image.getpixel((x, y + 1))>>5<<5, grayscale_image.getpixel((x + 1, y + 1))>>5<<5]
            ])

            gradient_x = np.sum(Gx * region)
            gradient_y = np.sum(Gy * region)
            edge_magnitude = int(abs(gradient_x)+ abs(gradient_y))

            # Normalize the edge magnitude to fit the 8-bit range (0-255)
            edge_magnitude = min(edge_magnitude, 255)
            edge_magnitude_r = edge_magnitude >> 5
            edge_magnitude_g = edge_magnitude >> 5
            edge_magnitude_b = edge_magnitude >> 6
            combined_edge_magnitude = (edge_magnitude_r << 5) | (edge_magnitude_g << 2) | edge_magnitude_b

            output_image.putpixel((x, y), (combined_edge_magnitude))

    return output_image

# Load the 8-bit grayscale image
input_image = Image.open("8-bit_Md_gray_320x240.png")
# Apply the Sobel filter
sobel_image = apply_sobel_filter(input_image)
# Save the output image
sobel_image.save("8-bit_8-bit_Md_edge_320x240.png", "PNG")