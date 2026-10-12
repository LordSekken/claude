class_name ForzaRawLog
extends RefCounted
## Raw Forza Data Out session log: every UDP packet as received, with its
## arrival time, so a whole session (e.g. 20 minutes of laps) can be analysed
## again later without re-driving.
##
## File: "DLFR" + u32 version, then per packet: u32 ms since start, u16 size,
## packet bytes. ~60 packets/s x ~330 bytes = ~1.2 MB per minute.

const MAGIC := "DLFR"
const VERSION := 1

var _f: FileAccess
var _t0 := 0
var packets := 0
var bytes := 0
var path := ""


func open(p: String) -> bool:
	DirAccess.make_dir_recursive_absolute(p.get_base_dir())
	_f = FileAccess.open(p, FileAccess.WRITE)
	if _f == null:
		return false
	path = p
	_f.store_buffer(MAGIC.to_ascii_buffer())
	_f.store_32(VERSION)
	_t0 = Time.get_ticks_msec()
	packets = 0
	bytes = 8
	return true


func is_open() -> bool:
	return _f != null


func write(pkt: PackedByteArray) -> void:
	if _f == null or pkt.size() > 65535:
		return
	_f.store_32(Time.get_ticks_msec() - _t0)
	_f.store_16(pkt.size())
	_f.store_buffer(pkt)
	packets += 1
	bytes += 6 + pkt.size()


func close() -> void:
	if _f != null:
		_f.close()
		_f = null


func minutes() -> float:
	return (Time.get_ticks_msec() - _t0) / 60000.0 if _f != null else 0.0


## Reads a log back: [{t_ms, data: PackedByteArray}], or [] if it isn't one.
static func read(p: String) -> Array:
	var f := FileAccess.open(p, FileAccess.READ)
	if f == null or f.get_length() < 8 or f.get_buffer(4).get_string_from_ascii() != MAGIC:
		return []
	f.get_32()  # version
	var out := []
	while f.get_position() + 6 <= f.get_length():
		var t := f.get_32()
		var n := f.get_16()
		if f.get_position() + n > f.get_length():
			break
		out.append({"t_ms": t, "data": f.get_buffer(n)})
	return out
