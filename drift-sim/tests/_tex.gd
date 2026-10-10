extends SceneTree
func _init() -> void:
	var dir: String = OS.get_cmdline_user_args()[0]
	for fn in ["skin_00.dds", "metal_detail.dds", "livery.png"]:
		var b := FileAccess.get_file_as_bytes(dir.path_join(fn))
		var img := DdsLoader.image_from_bytes(b)
		if img == null:
			print(fn, " -> null (", b.size(), " bytes, fourcc ", b.decode_u32(84) if b.size() > 88 else 0, ")")
			continue
		print(fn, " ", img.get_width(), "x", img.get_height(), " fmt ", img.get_format(), " mips ", img.has_mipmaps())
		if img.is_compressed():
			img.decompress()
		if img.has_mipmaps():
			img.clear_mipmaps()
		img.resize(256, 256)
		var c := img.get_pixel(128, 128)
		print("  centre pixel ", c)
		img.save_png("/tmp/claude-0/tex_" + fn.get_basename() + ".png")
	quit()
