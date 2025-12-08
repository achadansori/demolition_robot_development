# OLED Bitmap Converter

Tool untuk mengubah gambar menjadi bitmap array untuk OLED SSD1309/SSD1306 (128x64 pixels).

## 📋 Requirements

```bash
pip3 install Pillow
```

## 🎨 Cara Menggunakan

### 1. Siapkan Gambar Logo

**Penting:** Gambar harus kontras tinggi (hitam putih) untuk hasil terbaik!

- Buka logo di image editor (GIMP, Photoshop, Paint.NET)
- Resize ke **128x64 pixels** (atau biarkan, script akan auto-resize)
- **Tingkatkan kontras** agar logo lebih jelas
- **Tebalkan garis/font** jika perlu (bold logo = lebih jelas di OLED)
- Convert ke Black & White atau High Contrast
- Save sebagai PNG, JPG, atau BMP

### 2. Convert Gambar ke Bitmap

```bash
cd ~/STM32CubeIDE/demolition_robot_development/tools

# Basic convert (white background, black logo)
python3 image_to_oled_bitmap.py logo_aldzama.png

# Invert colors (black background, white logo) - RECOMMENDED untuk OLED!
python3 image_to_oled_bitmap.py logo_aldzama.png --invert

# Custom threshold (0-255, default 128)
# Lower = lebih banyak putih, Higher = lebih banyak hitam
python3 image_to_oled_bitmap.py logo_aldzama.png --invert --threshold 150

# Save to file
python3 image_to_oled_bitmap.py logo_aldzama.png --invert --output logo_bitmap.txt
```

### 3. Copy Bitmap ke oled.c

Copy output array, lalu:

```bash
# Edit oled.c
nano ~/STM32CubeIDE/demolition_robot_development/transmitter_demolition_robot/Core/Src/oled.c

# Cari bagian "static const uint8_t logo_aldzama[]"
# Ganti seluruh array dengan hasil convert
```

### 4. Rebuild & Flash

```bash
# Rebuild di STM32CubeIDE
# Flash ke STM32F401
# Test tampilan OLED
```

## 💡 Tips Membuat Logo Lebih Jelas

### Di Image Editor (GIMP):

1. **Open image**
2. **Image → Scale Image** → 128x64 pixels
3. **Colors → Brightness-Contrast** → Tingkatkan contrast (+50)
4. **Filters → Enhance → Sharpen** → Sharpen image
5. **Image → Mode → Indexed** → Use black/white palette (1-bit)
6. **Filters → Edge-Detect → Edge** (optional, untuk outline)
7. **Export as PNG**

### Di Photoshop:

1. Resize to 128x64
2. Image → Adjustments → Threshold (pilih threshold yang bikin logo jelas)
3. Filter → Sharpen → Unsharp Mask
4. Save as PNG

### Tips Desain:

- ✅ **Gunakan font bold/tebal** - lebih jelas di OLED
- ✅ **Hindari detail halus** - OLED 128x64 resolusi rendah
- ✅ **Kontras tinggi** - hitam putih murni, bukan grayscale
- ✅ **Test dengan --threshold** berbeda (100, 128, 150, 180)
- ✅ **Gunakan --invert** untuk logo putih di background hitam

## 🔍 Troubleshooting

**Logo terlalu gelap:**
```bash
python3 image_to_oled_bitmap.py logo.png --invert --threshold 100
```

**Logo terlalu terang:**
```bash
python3 image_to_oled_bitmap.py logo.png --invert --threshold 180
```

**Logo blur/tidak tajam:**
- Edit gambar asli, tingkatkan sharpness
- Gunakan font yang lebih bold
- Hilangkan anti-aliasing (gunakan pure black & white)

## 📐 Format Bitmap

Script menghasilkan bitmap dalam format **horizontal byte array**:
- 128 columns × 64 rows = 8192 pixels
- Organized in 8 pages (8 pixels per byte vertically)
- Total: 128 × 8 = 1024 bytes
- Compatible dengan SSD1306/SSD1309 horizontal addressing mode

## 🌐 Alternative: Online Tool

Jika tidak mau install Python:
1. https://javl.github.io/image2cpp/
2. Upload gambar 128x64 B&W
3. Settings: Horizontal, Invert colors
4. Generate code

## 📝 Example Output

```c
static const uint8_t logo_aldzama[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    ...
};
```

Copy array ini ke `transmitter_demolition_robot/Core/Src/oled.c`, ganti array `logo_aldzama[]` yang lama.
