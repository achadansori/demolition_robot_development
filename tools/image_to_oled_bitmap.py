#!/usr/bin/env python3
"""
Convert image to OLED bitmap array (128x64, horizontal byte format)
Compatible with SSD1306/SSD1309 displays

Usage:
    python3 image_to_oled_bitmap.py input_image.png
    python3 image_to_oled_bitmap.py input_image.png --invert
    python3 image_to_oled_bitmap.py input_image.png --threshold 128
"""

from PIL import Image
import sys
import argparse


def image_to_oled_bitmap(image_path, invert=False, threshold=128):
    """
    Convert image to OLED bitmap array

    Args:
        image_path: Path to input image
        invert: Invert colors (white becomes on, black becomes off)
        threshold: Brightness threshold (0-255) for converting to black/white

    Returns:
        List of bytes representing the bitmap
    """
    # Open and convert image
    img = Image.open(image_path)

    # Resize to 128x64 if needed
    if img.size != (128, 64):
        print(f"⚠ Resizing image from {img.size} to 128x64")
        img = img.resize((128, 64), Image.Resampling.LANCZOS)

    # Convert to grayscale then to black & white
    img = img.convert('L')  # Grayscale

    # Apply threshold to create pure black & white
    img = img.point(lambda x: 255 if x > threshold else 0, '1')

    # Get pixel data
    pixels = list(img.getdata())

    # Convert to OLED format (horizontal byte array)
    # OLED memory is organized in pages (8 rows per page)
    # Each byte represents 8 vertical pixels
    bitmap = []

    for page in range(8):  # 64 pixels / 8 = 8 pages
        for col in range(128):  # 128 columns
            byte_val = 0
            for bit in range(8):  # 8 rows per page
                row = page * 8 + bit
                pixel_index = row * 128 + col

                # Get pixel value (255 = white, 0 = black in PIL)
                pixel = pixels[pixel_index]

                # Convert to bit (1 = pixel on, 0 = pixel off)
                if invert:
                    # Invert: white (255) becomes 1 (pixel on)
                    pixel_on = (pixel > 0)
                else:
                    # Normal: black (0) becomes 1 (pixel on)
                    pixel_on = (pixel == 0)

                if pixel_on:
                    byte_val |= (1 << bit)

            bitmap.append(byte_val)

    return bitmap


def format_c_array(bitmap, array_name="bitmap"):
    """Format bitmap as C array"""
    output = f"static const uint8_t {array_name}[] = {{\n"

    for i in range(0, len(bitmap), 16):
        chunk = bitmap[i:i+16]
        hex_values = ", ".join(f"0x{b:02x}" for b in chunk)
        output += f"\t{hex_values},\n"

    output = output.rstrip(",\n") + "\n};\n"
    return output


def main():
    parser = argparse.ArgumentParser(
        description='Convert image to OLED bitmap array (128x64)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 image_to_oled_bitmap.py logo.png
  python3 image_to_oled_bitmap.py logo.png --invert
  python3 image_to_oled_bitmap.py logo.png --threshold 128 --output logo_bitmap.txt

Tips:
  - Input image will be resized to 128x64 automatically
  - Use --invert if white should be "on" (bright) on OLED
  - Adjust --threshold (0-255) to control which pixels become white/black
  - Lower threshold = more white pixels, Higher threshold = more black pixels
  - Default threshold 128 works well for most images
        """
    )

    parser.add_argument('image', help='Input image file (PNG, JPG, BMP, etc.)')
    parser.add_argument('-i', '--invert', action='store_true',
                       help='Invert colors (white=on, black=off)')
    parser.add_argument('-t', '--threshold', type=int, default=128,
                       help='Brightness threshold 0-255 (default: 128)')
    parser.add_argument('-n', '--name', default='logo_aldzama',
                       help='C array name (default: logo_aldzama)')
    parser.add_argument('-o', '--output', help='Output file (default: stdout)')

    args = parser.parse_args()

    # Validate threshold
    if not 0 <= args.threshold <= 255:
        print("Error: threshold must be between 0 and 255")
        return 1

    # Convert image
    print(f"Converting {args.image} to OLED bitmap...")
    print(f"  Threshold: {args.threshold}")
    print(f"  Invert: {args.invert}")

    try:
        bitmap = image_to_oled_bitmap(args.image, args.invert, args.threshold)
    except FileNotFoundError:
        print(f"Error: File '{args.image}' not found")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1

    # Format as C array
    c_array = format_c_array(bitmap, args.name)

    # Output
    if args.output:
        with open(args.output, 'w') as f:
            f.write(c_array)
        print(f"✓ Bitmap saved to {args.output}")
        print(f"  Array name: {args.name}")
        print(f"  Size: {len(bitmap)} bytes (128x64 = {len(bitmap)} bytes expected)")
    else:
        print("\n" + "="*70)
        print("Copy this array to your oled.c file:")
        print("="*70 + "\n")
        print(c_array)

    print("\n✓ Conversion complete!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
