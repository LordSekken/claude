class_name UiUtil
extends RefCounted
## Tiny helpers for building menus in code.


static func button(text: String, cb: Callable, min_h := 46) -> Button:
	var b := Button.new()
	b.text = text
	b.custom_minimum_size = Vector2(0, min_h)
	b.pressed.connect(cb)
	return b


static func label(text: String, size := 16, color := Color.WHITE, wrap := false) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	# Only paragraphs wrap. A wrapping label inside a row with no width of its
	# own collapses to one character per line (the "vertical text" bug).
	if wrap or text.length() > 70:
		l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		l.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		l.custom_minimum_size.x = 300
	return l


static func title(text: String) -> Label:
	return label(text, 34, Color(1, 0.8, 0.25))


static func slider(min_v: float, max_v: float, step: float, value: float, cb: Callable) -> HSlider:
	var s := HSlider.new()
	s.min_value = min_v
	s.max_value = max_v
	s.step = step
	s.value = value
	s.custom_minimum_size = Vector2(220, 24)
	s.value_changed.connect(cb)
	return s


static func row(label_text: String, control: Control, label_w := 200) -> HBoxContainer:
	var h := HBoxContainer.new()
	var l := label(label_text)
	l.custom_minimum_size = Vector2(label_w, 0)
	l.autowrap_mode = TextServer.AUTOWRAP_OFF
	h.add_child(l)
	control.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(control)
	return h


static func option(items: Array, selected: int, cb: Callable) -> OptionButton:
	var o := OptionButton.new()
	for it: Variant in items:
		o.add_item(str(it))
	o.selected = selected
	o.item_selected.connect(cb)
	return o


## Full-screen dark backdrop + centred, scrollable column.
static func screen(parent: Control, width := 900) -> VBoxContainer:
	var bg := ColorRect.new()
	bg.color = Color(0.07, 0.08, 0.1)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	parent.add_child(bg)
	var scroll := ScrollContainer.new()
	scroll.set_anchors_preset(Control.PRESET_FULL_RECT)
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	parent.add_child(scroll)
	var center := CenterContainer.new()
	center.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(center)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_top", 30)
	margin.add_theme_constant_override("margin_bottom", 30)
	center.add_child(margin)
	var vb := VBoxContainer.new()
	vb.custom_minimum_size = Vector2(width, 0)
	vb.add_theme_constant_override("separation", 10)
	margin.add_child(vb)
	return vb


static func file_dialog(parent: Node, filters: PackedStringArray, cb: Callable, save := false) -> FileDialog:
	var fd := FileDialog.new()
	fd.access = FileDialog.ACCESS_FILESYSTEM
	fd.file_mode = FileDialog.FILE_MODE_SAVE_FILE if save else FileDialog.FILE_MODE_OPEN_FILE
	fd.filters = filters
	fd.use_native_dialog = true
	fd.size = Vector2i(900, 600)
	fd.file_selected.connect(cb)
	if save:
		fd.dir_selected.connect(cb)
	parent.add_child(fd)
	fd.popup_centered()
	return fd


static func dir_dialog(parent: Node, cb: Callable) -> FileDialog:
	var fd := FileDialog.new()
	fd.access = FileDialog.ACCESS_FILESYSTEM
	fd.file_mode = FileDialog.FILE_MODE_OPEN_DIR
	fd.use_native_dialog = true
	fd.size = Vector2i(900, 600)
	fd.dir_selected.connect(cb)
	parent.add_child(fd)
	fd.popup_centered()
	return fd
