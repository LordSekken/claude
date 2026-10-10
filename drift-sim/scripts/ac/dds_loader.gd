class_name DdsLoader
extends RefCounted
## Minimal DDS reader for Assetto Corsa textures. Block-compressed formats
## (DXT1/3/5, BC4/5, BC7) go straight to the GPU without re-encoding.

const FOURCC_DXT1 := 0x31545844
const FOURCC_DXT3 := 0x33545844
const FOURCC_DXT5 := 0x35545844
const FOURCC_ATI1 := 0x31495441
const FOURCC_BC4U := 0x55344342
const FOURCC_ATI2 := 0x32495441
const FOURCC_BC5U := 0x55354342
const FOURCC_DX10 := 0x30315844


static func _u32(b: PackedByteArray, o: int) -> int:
	return b.decode_u32(o)


## Returns an Image or null.
static func load_from_buffer(b: PackedByteArray) -> Image:
	if b.size() < 128 or _u32(b, 0) != 0x20534444:  # "DDS "
		return null
	var h := _u32(b, 12)
	var w := _u32(b, 16)
	var mips := maxi(_u32(b, 28), 1)
	var pf_flags := _u32(b, 80)
	var fourcc := _u32(b, 84)
	var bpp := _u32(b, 88)
	var rmask := _u32(b, 92)
	var offset := 128
	var fmt := -1
	var block := 0
	var swap_bgr := false
	if pf_flags & 0x4:  # DDPF_FOURCC
		match fourcc:
			FOURCC_DXT1: fmt = Image.FORMAT_DXT1; block = 8
			FOURCC_DXT3: fmt = Image.FORMAT_DXT3; block = 16
			FOURCC_DXT5: fmt = Image.FORMAT_DXT5; block = 16
			FOURCC_ATI1, FOURCC_BC4U: fmt = Image.FORMAT_RGTC_R; block = 8
			FOURCC_ATI2, FOURCC_BC5U: fmt = Image.FORMAT_RGTC_RG; block = 16
			FOURCC_DX10:
				var dxgi := _u32(b, 128)
				offset = 148
				match dxgi:
					70, 71, 72: fmt = Image.FORMAT_DXT1; block = 8
					73, 74, 75: fmt = Image.FORMAT_DXT3; block = 16
					76, 77, 78: fmt = Image.FORMAT_DXT5; block = 16
					79, 80, 81: fmt = Image.FORMAT_RGTC_R; block = 8
					82, 83, 84: fmt = Image.FORMAT_RGTC_RG; block = 16
					97, 98, 99: fmt = Image.FORMAT_BPTC_RGBA; block = 16
					27, 28, 29: fmt = Image.FORMAT_RGBA8
					87, 90, 91: fmt = Image.FORMAT_RGBA8; swap_bgr = true
	elif bpp == 32:
		fmt = Image.FORMAT_RGBA8
		swap_bgr = rmask == 0x00ff0000
	elif bpp == 24:
		fmt = Image.FORMAT_RGB8
		swap_bgr = rmask == 0x00ff0000
	if fmt < 0:
		return null
	# Work out how many mip levels the data actually contains.
	var total := 0
	var levels := 0
	var mw := w
	var mh := h
	for i in mips:
		var sz := (maxi(1, (mw + 3) / 4) * maxi(1, (mh + 3) / 4) * block) if block > 0 else mw * mh * (4 if fmt == Image.FORMAT_RGBA8 else 3)
		if offset + total + sz > b.size():
			break
		total += sz
		levels += 1
		if mw == 1 and mh == 1:
			break
		mw = maxi(1, mw / 2)
		mh = maxi(1, mh / 2)
	if levels == 0:
		return null
	# Godot needs either one level or the full chain down to 1x1.
	var full := 1
	var t := maxi(w, h)
	while t > 1:
		t /= 2
		full += 1
	var use_mips := levels >= full
	total = 0
	mw = w
	mh = h
	for i in (full if use_mips else 1):
		total += (maxi(1, (mw + 3) / 4) * maxi(1, (mh + 3) / 4) * block) if block > 0 else mw * mh * (4 if fmt == Image.FORMAT_RGBA8 else 3)
		mw = maxi(1, mw / 2)
		mh = maxi(1, mh / 2)
	var data := b.slice(offset, offset + total)
	if swap_bgr:
		var stride := 4 if fmt == Image.FORMAT_RGBA8 else 3
		for i in range(0, data.size() - stride + 1, stride):
			var tmp := data[i]
			data[i] = data[i + 2]
			data[i + 2] = tmp
	# Block-compressed data stays compressed: desktop GPUs sample it directly.
	return Image.create_from_data(w, h, use_mips, fmt, data)


## Any texture blob AC may embed: DDS, PNG, JPG.
static func image_from_bytes(b: PackedByteArray) -> Image:
	if b.size() < 4:
		return null
	if b.decode_u32(0) == 0x20534444:
		return load_from_buffer(b)
	var img := Image.new()
	if b[0] == 0x89 and b[1] == 0x50:
		return img if img.load_png_from_buffer(b) == OK else null
	if b[0] == 0xFF and b[1] == 0xD8:
		return img if img.load_jpg_from_buffer(b) == OK else null
	return null
