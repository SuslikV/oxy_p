# Converts BMP images to byte array for SSD1306 display running in vertical addressing mode
#
# Usage:
#   python.exe "bmp_to_vertical_addressing_byte_array.py" input_BMP_file [output_TXT_file]
#
# Example:
#   python.exe "bmp_to_vertical_addressing_byte_array.py" "single_glyph.bmp" "byte_array.txt"
#
# Tested with python-3.6.7-embed-amd64
# -----------------------------------------------------------------------------
# When set to 1 - each byte in the array will has "b_XXXX_XXXX" notation:

easy_bits_output = 0

#------------------------------------------------------------------------------

import struct
import sys

if len(sys.argv) < 2:
    print("\nUsage:")
    print("  python.exe bmp_to_vertical_addressing_byte_array.py input_BMP_file [output_TXT_file]")
    sys.exit()

input_file_name = sys.argv[1] # "single_glyph.bmp"
output_file_name = None
if len(sys.argv) > 2:
    output_file_name = sys.argv[2] # "byte_array.txt"
print(input_file_name)

# Read whole file to RAM (max expected ~32768 bytes)
with open(input_file_name, 'rb') as img_file:
     file_data_buffer = img_file.read()
img_file.close()

# In file offsets
BMP_offset_Type = 0
BMP_offset_PixelArray = 10
BMP_offset_width = 18
BMP_offset_height = 22
BMP_offset_Planes = 26
BMP_offset_BPP = 28
BMP_offset_Compression = 30

img_file_type = struct.unpack_from('2s', file_data_buffer, BMP_offset_Type)[0]
if not img_file_type == b'BM':
    print("\nError: Unknown Format! This converter expects Windows BITMAPINFOHEADER for BMP file.")
    sys.exit()

img_file_compression = struct.unpack_from('I', file_data_buffer, BMP_offset_Compression)[0]
if not (img_file_compression == 0) and not (img_file_compression == 3):
    print("\nError: Unknown Compression! This converter expects not compressed BMP file (support for bitmasks is very limited).")
    sys.exit()

PixelArrayStart = struct.unpack_from('I', file_data_buffer, BMP_offset_PixelArray)[0]
image_width = struct.unpack_from('I', file_data_buffer, BMP_offset_width)[0]
image_height = struct.unpack_from('I', file_data_buffer, BMP_offset_height)[0]
bits_per_pixel = int(struct.unpack_from('H', file_data_buffer, BMP_offset_BPP)[0])

pixels_total = image_width * image_height
row_size_bytes = (int((bits_per_pixel * image_width + 31) / 32)) * 4
print("Pixels total: {}".format(pixels_total))
print("Row size bytes: {}".format(row_size_bytes))

# 4294967296 (BGRA etc)
bytes_per_pixel = 4
if bits_per_pixel == 24:     # 16777216 (BGR)
    bytes_per_pixel = 3
elif bits_per_pixel == 16:   # 65536 colors
    bytes_per_pixel = 2
elif bits_per_pixel == 8:    # 256 colors
    bytes_per_pixel = 1
elif bits_per_pixel == 4:    # 16 colors
    bytes_per_pixel = 1 / 2
elif bits_per_pixel == 2:    # 4 colors
    bytes_per_pixel = 1 / 4
elif bits_per_pixel == 1:    # monochrome
    bytes_per_pixel = 1 / 8

print("Bytes per pixel: {}".format(bytes_per_pixel))

if (bytes_per_pixel < 1):
    print("\nError: Color depth less than 1 byte per pixel! Not yet supported by this converter.")
    sys.exit()

# Brief info
print("W: {}".format(image_width))
print("H: {}".format(image_height))

# For indexed color formats analyze index number rather than actual color.
# Assuming that background colors lies at lower indexes than this number.
white_pixel_threshold = 1

# Array element size in bits
elem_size = 8 * 1

elem = 0
bit_pow = 0

# Ease human perception of data set for per page display outputs (vertical mode)
easy_bits = [
    'b_0000_0000', 'b_0000_0001', 'b_0000_0010', 'b_0000_0011', 'b_0000_0100', 'b_0000_0101', 'b_0000_0110', 'b_0000_0111',
    'b_0000_1000', 'b_0000_1001', 'b_0000_1010', 'b_0000_1011', 'b_0000_1100', 'b_0000_1101', 'b_0000_1110', 'b_0000_1111',
    'b_0001_0000', 'b_0001_0001', 'b_0001_0010', 'b_0001_0011', 'b_0001_0100', 'b_0001_0101', 'b_0001_0110', 'b_0001_0111',
    'b_0001_1000', 'b_0001_1001', 'b_0001_1010', 'b_0001_1011', 'b_0001_1100', 'b_0001_1101', 'b_0001_1110', 'b_0001_1111',
    'b_0010_0000', 'b_0010_0001', 'b_0010_0010', 'b_0010_0011', 'b_0010_0100', 'b_0010_0101', 'b_0010_0110', 'b_0010_0111',
    'b_0010_1000', 'b_0010_1001', 'b_0010_1010', 'b_0010_1011', 'b_0010_1100', 'b_0010_1101', 'b_0010_1110', 'b_0010_1111',
    'b_0011_0000', 'b_0011_0001', 'b_0011_0010', 'b_0011_0011', 'b_0011_0100', 'b_0011_0101', 'b_0011_0110', 'b_0011_0111',
    'b_0011_1000', 'b_0011_1001', 'b_0011_1010', 'b_0011_1011', 'b_0011_1100', 'b_0011_1101', 'b_0011_1110', 'b_0011_1111',
    'b_0100_0000', 'b_0100_0001', 'b_0100_0010', 'b_0100_0011', 'b_0100_0100', 'b_0100_0101', 'b_0100_0110', 'b_0100_0111',
    'b_0100_1000', 'b_0100_1001', 'b_0100_1010', 'b_0100_1011', 'b_0100_1100', 'b_0100_1101', 'b_0100_1110', 'b_0100_1111',
    'b_0101_0000', 'b_0101_0001', 'b_0101_0010', 'b_0101_0011', 'b_0101_0100', 'b_0101_0101', 'b_0101_0110', 'b_0101_0111',
    'b_0101_1000', 'b_0101_1001', 'b_0101_1010', 'b_0101_1011', 'b_0101_1100', 'b_0101_1101', 'b_0101_1110', 'b_0101_1111',
    'b_0110_0000', 'b_0110_0001', 'b_0110_0010', 'b_0110_0011', 'b_0110_0100', 'b_0110_0101', 'b_0110_0110', 'b_0110_0111',
    'b_0110_1000', 'b_0110_1001', 'b_0110_1010', 'b_0110_1011', 'b_0110_1100', 'b_0110_1101', 'b_0110_1110', 'b_0110_1111',
    'b_0111_0000', 'b_0111_0001', 'b_0111_0010', 'b_0111_0011', 'b_0111_0100', 'b_0111_0101', 'b_0111_0110', 'b_0111_0111',
    'b_0111_1000', 'b_0111_1001', 'b_0111_1010', 'b_0111_1011', 'b_0111_1100', 'b_0111_1101', 'b_0111_1110', 'b_0111_1111',
    'b_1000_0000', 'b_1000_0001', 'b_1000_0010', 'b_1000_0011', 'b_1000_0100', 'b_1000_0101', 'b_1000_0110', 'b_1000_0111',
    'b_1000_1000', 'b_1000_1001', 'b_1000_1010', 'b_1000_1011', 'b_1000_1100', 'b_1000_1101', 'b_1000_1110', 'b_1000_1111',
    'b_1001_0000', 'b_1001_0001', 'b_1001_0010', 'b_1001_0011', 'b_1001_0100', 'b_1001_0101', 'b_1001_0110', 'b_1001_0111',
    'b_1001_1000', 'b_1001_1001', 'b_1001_1010', 'b_1001_1011', 'b_1001_1100', 'b_1001_1101', 'b_1001_1110', 'b_1001_1111',
    'b_1010_0000', 'b_1010_0001', 'b_1010_0010', 'b_1010_0011', 'b_1010_0100', 'b_1010_0101', 'b_1010_0110', 'b_1010_0111',
    'b_1010_1000', 'b_1010_1001', 'b_1010_1010', 'b_1010_1011', 'b_1010_1100', 'b_1010_1101', 'b_1010_1110', 'b_1010_1111',
    'b_1011_0000', 'b_1011_0001', 'b_1011_0010', 'b_1011_0011', 'b_1011_0100', 'b_1011_0101', 'b_1011_0110', 'b_1011_0111',
    'b_1011_1000', 'b_1011_1001', 'b_1011_1010', 'b_1011_1011', 'b_1011_1100', 'b_1011_1101', 'b_1011_1110', 'b_1011_1111',
    'b_1100_0000', 'b_1100_0001', 'b_1100_0010', 'b_1100_0011', 'b_1100_0100', 'b_1100_0101', 'b_1100_0110', 'b_1100_0111',
    'b_1100_1000', 'b_1100_1001', 'b_1100_1010', 'b_1100_1011', 'b_1100_1100', 'b_1100_1101', 'b_1100_1110', 'b_1100_1111',
    'b_1101_0000', 'b_1101_0001', 'b_1101_0010', 'b_1101_0011', 'b_1101_0100', 'b_1101_0101', 'b_1101_0110', 'b_1101_0111',
    'b_1101_1000', 'b_1101_1001', 'b_1101_1010', 'b_1101_1011', 'b_1101_1100', 'b_1101_1101', 'b_1101_1110', 'b_1101_1111',
    'b_1110_0000', 'b_1110_0001', 'b_1110_0010', 'b_1110_0011', 'b_1110_0100', 'b_1110_0101', 'b_1110_0110', 'b_1110_0111',
    'b_1110_1000', 'b_1110_1001', 'b_1110_1010', 'b_1110_1011', 'b_1110_1100', 'b_1110_1101', 'b_1110_1110', 'b_1110_1111',
    'b_1111_0000', 'b_1111_0001', 'b_1111_0010', 'b_1111_0011', 'b_1111_0100', 'b_1111_0101', 'b_1111_0110', 'b_1111_0111',
    'b_1111_1000', 'b_1111_1001', 'b_1111_1010', 'b_1111_1011', 'b_1111_1100', 'b_1111_1101', 'b_1111_1110', 'b_1111_1111',
]

# Full picture maket in RAM as strings (expected max of 64 lines)
art_byte = [""] * image_height

# Full byte array maket in RAM as strings (expected max of 128 / elem_size lines)
byte_array = [""] * image_width
byte_array_idx = 0

# Try to skip alpha channel
R_location = bytes_per_pixel - 1
if img_file_compression == 0 and bytes_per_pixel == 4:
    R_location = bytes_per_pixel - 2

# Read image file by columns from left to right, from bottom to top (BMP starts from bottom left)
for col in range (0, image_width, 1):
    column = col * bytes_per_pixel
    for pixel_i in range(image_height - 1, -1, -1):
        # Get value of the last not alpha byte from pixel (usually BGR triad or ABGR or BGRA)
        byte_R = struct.unpack_from('B', file_data_buffer, PixelArrayStart + column + pixel_i * row_size_bytes + R_location)[0]

        # Calculate bit value of this pixel
        if (byte_R >= white_pixel_threshold):
            byte_R = 1
        else:
            byte_R = 0

		# Building byte. LBS at bottom because BMP starts from bottom.
        elem = elem + round(byte_R * 2 ** bit_pow)
        bit_pow = bit_pow + 1

        # Pixel art
        art_byte[pixel_i] = art_byte[pixel_i] + "{}".format(byte_R)

        # Prepare to the next element
        if (pixel_i % elem_size) == 0:
            space = " "
            if pixel_i == 0:
                space = ""

            # Element complete, pack columns to rows (array's column = page of the display)
            if easy_bits_output:
                byte_array[byte_array_idx] = byte_array[byte_array_idx] + easy_bits[elem] + "," + space
            else:
                byte_array[byte_array_idx] = byte_array[byte_array_idx] + "0x{:0>2X},".format(elem) + space

            bit_pow = 0
            elem = 0
    byte_array_idx = byte_array_idx + 1

# Pixel art
print("Preview:")
for pixel_i in range(image_height - 1, -1, -1):
    print(art_byte[pixel_i])

print("\nByte array (vertical addressing mode):\n")
# Byte array
for elem_i in range(0, image_width, 1):
    print(byte_array[elem_i])

print("\nArray size, bytes: {} [{}x{}]".format(int(len(byte_array) * image_height / 8), image_width, image_height))

# Output file creation
if output_file_name is not None:
    try:
        out_txt_file = open("{}".format(output_file_name), "x")
    except FileExistsError:
        print("\nError: Output file already exist! {}".format(output_file_name))
        sys.exit()

    for elem_i in range(0, image_width, 1):
        out_txt_file.write(byte_array[elem_i])
        out_txt_file.write("\n")
    out_txt_file.write("\n")
    out_txt_file.write("\n// Array size, bytes: {} [{}x{}]".format(int(len(byte_array) * image_height / 8), image_width, image_height))    
    out_txt_file.close()
