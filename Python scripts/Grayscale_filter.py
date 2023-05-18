from PIL import Image, ImagePalette

def create_8bit_color_palette():
    palette = []
    for r in range(8):
        for g in range(8):
            for b in range(4):
                palette.extend([(r << 5) | (r << 2), (g << 5) | (g << 2), (b << 6) | (b << 4)])
    return palette

def color_to_gray(bit_img):
    width, height = bit_img.size
    output_image = Image.new('P', (width, height))
    # Set the custom color palette
    custom_palette = ImagePalette.ImagePalette('RGB', create_8bit_color_palette())
    output_image.putpalette(custom_palette)

    for x in range(width):
        for y in range(height):
            pixel_color = bit_img.getpixel((x, y))
            # multiplication factors are to expand 3/3/2 bits to their original 8/8/8 bits
            r = (pixel_color >> 5)*36
            g = ((pixel_color >> 2) & 0x07)*36
            b = (pixel_color & 0x03)*85
            gray_value = int(0.2989 * r + 0.5870 * g + 0.1140 * b)
            gray_r = gray_value >> 5
            gray_g = gray_value >> 5
            gray_b = gray_value >> 6
            combined_gray_value = (gray_r << 5) | (gray_g << 2) | gray_b
            output_image.putpixel((x, y), (combined_gray_value))
    return output_image

input_image = Image.open("8-bit_Md_320x240.png")
output_image = color_to_gray(input_image)
output_image.save("8-bit_Md_gray_320x240.png", "PNG")

