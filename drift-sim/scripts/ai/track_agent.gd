class_name TrackAgent
extends ClaudeClient
## Picture of a place -> drift track, done agentically:
##  - Claude looks at the photo (top-down, map screenshot, sketch, or a
##    ground-level photo of a real spot),
##  - can run the offline pixel tracer on it for an exact centreline,
##  - submits a track spec, gets back validation errors + a rendered top-down
##    preview of what it built, compares that to the photo, and refines.

const SYSTEM := """You design drift tracks for Drift Lab from pictures of real places.

The user gives you an image. It may be: a top-down satellite/map screenshot, a drone shot, a hand-drawn sketch, or a ground-level photo of a road, mountain pass, parking lot, industrial estate or race track.

Workflow:
1. Look at the image. Work out what it shows and its real-world scale from cues (lane ~3.5 m, parking bay ~2.5 x 5 m, car ~4.5 m, road widths, buildings).
2. For top-down images or sketches, call trace_image first: it returns a pixel-accurate centreline in metres that you should build on (fix the scale/width, smooth glitches, pick the right loop). For ground-level photos, design a plausible layout that captures the place (its corner types, elevation, walls/run-off, surface, mood) - you can't trace those.
3. Call submit_track with a complete track spec. You'll get validation errors/warnings and a top-down preview image of what you built (white = road, orange = drift zones, green dot = start). Compare it with the photo and fix problems. Iterate until there are no errors and it looks right. Usually 1-3 submissions.
4. Finish with a 2-3 sentence description of the track.

Track spec (JSON, metres, x = east, z = south, y = elevation):
{"name", "description", "closed": bool (loop or point-to-point), "points": [[x, z] or [x, z, y], ...] centreline control points spaced ~10-30 m apart (closed tracks: do not repeat the first point), "width": road width (drift tracks 10-16 m; mountain roads 6-9 m), "widths": optional per-point widths, "runoff": metres beside the road before walls (0-15), "runoff_surface": grass|asphalt|gravel|dirt, "walls": {"left": bool, "right": bool, "height": m}, "kerbs": bool, "pads": [{"points": [[x, z], ...], "surface": "asphalt"}] for open lots/skidpads, "zones": [{"name", "start": 0-1, "end": 0-1}] fractions of track length (judged drift sections - put them on flowing corner sequences), "clips": [{"t": 0-1, "side": "left"|"right", "radius": 2}] clipping points (outer clips on the outside wall of a corner, inner clips at apexes), "start": 0-1, "scenery": {"seed": int, "density": 0-2, "time_of_day": "day"|"dusk"|"night"}}

Good drift tracks: corners with 15-60 m radius, linked transitions, an entry at speed (a straight before the first zone), walls close enough to tease (runoff 2-6 m) but not so tight it's a wall-ride. Keep grades under ~15%. Keep the overall size true to the real place."""

var image: Image
var best_spec: Dictionary = {}
var trace_options: Dictionary = {}


func _tools() -> Array:
	return [
		{"name": "trace_image", "description": "Run the offline pixel tracer on the user's image. Returns a centreline (metres) for the largest road/line shape, whether it's a loop, the estimated scale, and a debug image (blue = detected road, red = traced path). Best for top-down images and sketches.",
			"input_schema": {"type": "object", "properties": {
				"mode": {"type": "string", "enum": ["auto", "road", "lines"], "description": "road = grey asphalt in aerial imagery; lines = dark lines on a light sketch"},
				"road_width_m": {"type": "number", "description": "Real road width in metres, used to set scale from the road's pixel width."},
				"target_length_m": {"type": "number", "description": "For sketches: desired track length in metres."}},
				"required": ["mode"]}},
		{"name": "submit_track", "description": "Validate a complete track spec and render a top-down preview. Returns errors, warnings, stats and the preview image. The last valid submission is what the player gets.",
			"input_schema": {"type": "object", "properties": {"track": {"type": "object"}}, "required": ["track"]}},
	]


## Returns {spec, text, error}
func run(img: Image, notes: String) -> Dictionary:
	image = img
	var text := "Turn this into a drift track."
	if notes.strip_edges() != "":
		text += " Notes from me: " + notes.strip_edges()
	var messages := [{"role": "user", "content": [ClaudeClient.image_block(img), {"type": "text", "text": text}]}]
	var r := await run_agent(SYSTEM, messages, _tools(), _handle)
	if str(r.text).strip_edges() != "":
		progress.emit("[b]Claude:[/b] " + str(r.text).strip_edges())
	if not best_spec.is_empty() and str(r.text).strip_edges() != "":
		best_spec["description"] = str(best_spec.get("description", r.text.strip_edges()))
	return {"spec": best_spec, "text": r.text, "error": r.error}


func _handle(name: String, input: Dictionary) -> Dictionary:
	match name:
		"trace_image":
			var opts := trace_options.duplicate()
			opts.mode = str(input.get("mode", "auto"))
			if input.has("road_width_m"):
				opts.road_width_m = float(input.road_width_m)
			if input.has("target_length_m"):
				opts.target_length_m = float(input.target_length_m)
			var tr := ImageTracer.trace(image, opts)
			if not tr.ok:
				return {"content": [{"type": "text", "text": "Trace failed: " + str(tr.error)}, ClaudeClient.image_block(tr.debug, 800, false)], "is_error": true}
			var summary := {"closed": tr.spec.closed, "width_m": tr.spec.width, "points": tr.spec.points, "info": tr.info}
			progress.emit("   traced %d points, %.0f m, %s" % [(tr.spec.points as Array).size(), float(tr.info.length_m), "loop" if tr.spec.closed else "open"])
			return {"content": [{"type": "text", "text": JSON.stringify(summary)}, ClaudeClient.image_block(tr.debug, 800, false)]}
		"submit_track":
			if not (input.get("track") is Dictionary):
				return {"content": "'track' must be an object", "is_error": true}
			var spec: Dictionary = input.track
			var v := TrackSpec.validate(spec)
			var preview := TrackRaster.render(spec, 640, Color(0.1, 0.1, 0.12), Color(0.95, 0.95, 0.95), true)
			var report := {"errors": v.errors, "warnings": v.warnings, "stats": v.stats}
			if (v.errors as Array).is_empty():
				best_spec = spec
				progress.emit("[color=#66ff88]✔ track accepted: %s (%d m)[/color]" % [spec.get("name", "track"), int(v.stats.get("length_m", 0))])
			else:
				progress.emit("[color=#ffcc66]  %d issue(s), refining...[/color]" % (v.errors as Array).size())
			return {"content": [{"type": "text", "text": JSON.stringify(report)}, ClaudeClient.image_block(preview.image, 640, false)]}
	return {"content": "Unknown tool", "is_error": true}
