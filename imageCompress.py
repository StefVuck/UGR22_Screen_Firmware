from PIL import Image
import numpy as np
import sys

"""
Author: Stefan Vuckovic
Created: 29 May 2024
@Brief: This program compresses an image into a C array.
"""


"""
This function converts 8-bit RGB values to a 16-bit RGB565 format:

r & 0xF8 ensures the red value uses 5 bits.
g & 0xFC ensures the green value uses 6 bits.
b >> 3 ensures the blue value uses 5 bits.
"""
def convert_rgb_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


# Use of Numpy Array to Help Speed
def load_image(filename):
    img = Image.open(filename)
    img = img.convert('RGB')
    width, height = img.size
    rgb_array = np.array(img)
    rgb565_array = np.zeros((height, width), dtype=np.uint16)

    for y in range(height):
        for x in range(width):
            r, g, b = rgb_array[y, x]
            rgb565_array[y, x] = convert_rgb_to_rgb565(r, g, b)
    
    return rgb565_array, width, height


"""
'Run-Length Encoding (RLE) is a simple form of data compression that
converts consecutive identical values into a count-value pair.'

For example, the array [1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3] would be encoded as:
                       [(1, 4), (2, 3), (3, 4)].

This was used as the compiler was unable to handle the large amount of data in the original image array.
"""
def rle_encode(data):
    encoded_data = []
    prev_value = data[0]
    count = 1

    for value in data[1:]:
        if value == prev_value and count < 65535: # 65535 is the maximum value for a 16-bit unsigned integer
            count += 1
        else:
            encoded_data.append((prev_value, count))
            prev_value = value
            count = 1
    
    encoded_data.append((prev_value, count))
    return encoded_data


# Required for Colors to display Correct.
def swap_endianness_16bit(value):
    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8)


"""
This function saves the compressed data as a C array. Meaning it can be embedded in a C program.
This is useful as the embedded system is unable to read binaries.
"""
def save_as_c_array(rle_data, width, height, filename):
    with open(filename, 'w') as f:
        f.write('#include <stdint.h>\n\n')
        f.write(f'const uint32_t image_width = {width};\n')
        f.write(f'const uint32_t image_height = {height};\n')
        f.write('const uint8_t compressed_data[] = {\n')
        for value, count in rle_data:
            swapped_value = swap_endianness_16bit(value)
            f.write(f'    0x{swapped_value >> 8:02X}, 0x{swapped_value & 0xFF:02X}, 0x{count >> 8:02X}, 0x{count & 0xFF:02X},\n')
        f.write('};\n')

# Generic main function to run the program
def main(input_filename, output_filename):
    rgb565_array, width, height = load_image(input_filename)
    if rgb565_array is None:
        print("Failed to load image.")
        return
    flat_rgb565_array = rgb565_array.flatten()
    rle_data = rle_encode(flat_rgb565_array)
    save_as_c_array(rle_data, width, height, output_filename)

# CLI argument handling
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python imageCompress.py <input_image> <output_file>")
    else:
        main(sys.argv[1], sys.argv[2])
