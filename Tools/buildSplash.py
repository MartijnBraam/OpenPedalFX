from PIL import Image
import struct
import sys

canvas = Image.open(sys.argv[1])
canvas.load()
output = []

for y in range(0, int(canvas.size[1]/8)):
    for x in range(0, canvas.size[0]):
        output_byte = ''
        for p in range(0,8):
            pixel = canvas.getpixel((x,(y*8)+p))
            output_byte=str(pixel)+output_byte
        output.append("0x%x" % int(output_byte,2))

print("void PedalSplash(uint8_t buffer[1024]){")
print("  const uint8_t pixels[1024] = {")
print("  "+", ".join(output))
print("  };")
print("  memcpy(buffer, pixels, 1024);")
print("}")
