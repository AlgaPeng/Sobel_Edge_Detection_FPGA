from PIL import Image, ImagePalette

def create_8bit_color_palette():
    palette = []
    for r in range(8):
        for g in range(8):
            for b in range(4):
                palette.extend([(r << 5) | (r << 2), (g << 5) | (g << 2), (b << 6) | (b << 4)])
    return palette

def rgb_to_8bit_color(rgb_image):
    width, height = rgb_image.size
    output_image = Image.new('P', (width, height))
    
    # Set the custom color palette
    custom_palette = ImagePalette.ImagePalette('RGB', create_8bit_color_palette())
    output_image.putpalette(custom_palette)

    for x in range(width):
        for y in range(height):
            r, g, b = rgb_image.getpixel((x, y))[:3]
            r_reduced = r >> 5
            g_reduced = g >> 5
            b_reduced = b >> 6
            combined_8bit = (r_reduced << 5) | (g_reduced << 2) | b_reduced
            output_image.putpixel((x, y), combined_8bit)

    return output_image

# Load the RGB image
input_image = Image.open("Muse_dash_320x240.png")
# Convert the image
output_image = rgb_to_8bit_color(input_image)
# Save the converted image
output_image.save("8-bit_Md_320x240.png", "PNG")

