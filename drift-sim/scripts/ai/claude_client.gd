class_name ClaudeClient
extends Node
## Minimal Claude Messages API client + tool-use agent loop for GDScript.
## (There is no official Anthropic SDK for GDScript, so this talks to the REST
## API directly.) Requires an API key from console.anthropic.com in Settings.

signal progress(text: String)

const URL := "https://api.anthropic.com/v1/messages"
const MAX_TURNS := 24

var api_key := ""
var model := "claude-opus-5-5"
var effort := "high"


func _ready() -> void:
	var s: Node = get_node_or_null("/root/Settings")
	if s:
		api_key = str(s.api_key)
		model = str(s.model)


## One Messages API call. Returns the parsed response or {"error": msg}.
func create(body: Dictionary) -> Dictionary:
	var http := HTTPRequest.new()
	http.timeout = 900.0
	add_child(http)
	var headers := PackedStringArray([
		"x-api-key: " + api_key,
		"anthropic-version: 2023-06-01",
		"content-type: application/json",
		# Server-side fallback: if a request is declined by a safety
		# classifier, the API retries it on a fallback model automatically.
		"anthropic-beta: server-side-fallback-2026-07-01",
	])
	var err := http.request(URL, headers, HTTPClient.METHOD_POST, JSON.stringify(body))
	if err != OK:
		http.queue_free()
		return {"error": "HTTP request failed to start (%d)" % err}
	var res: Array = await http.request_completed
	http.queue_free()
	var code: int = res[1]
	var text: String = (res[3] as PackedByteArray).get_string_from_utf8()
	var data: Variant = JSON.parse_string(text)
	if res[0] != HTTPRequest.RESULT_SUCCESS:
		return {"error": "Network error (%d). Check your internet connection." % res[0]}
	if code != 200:
		var msg := text
		if data is Dictionary and data.get("error") is Dictionary:
			msg = str(data.error.get("message", text))
		match code:
			401: msg = "Invalid API key (401). Check Settings."
			429: msg = "Rate limited (429) - wait a moment and retry. " + msg
			529: msg = "API overloaded (529) - retry shortly."
		return {"error": "API error %d: %s" % [code, msg]}
	if not (data is Dictionary):
		return {"error": "Unexpected API response"}
	return data


## Runs a tool-use loop until Claude stops calling tools.
## tools: client tool definitions (plus any server tools such as web_search).
## handler: Callable(name: String, input: Dictionary) -> Dictionary
##          returning {"content": String | Array, "is_error": bool}; may be async.
## Returns {"text": final text, "messages": history, "error": ""}.
func run_agent(system: String, messages: Array, tools: Array, handler: Callable) -> Dictionary:
	for turn in MAX_TURNS:
		var body := {
			"model": model,
			"max_tokens": 16000,
			"system": system,
			"messages": messages,
			"tools": tools,
			"fallbacks": "default",
			"output_config": {"effort": effort},
		}
		var resp := await create(body)
		if resp.has("error"):
			return {"text": "", "messages": messages, "error": str(resp.error)}
		var content: Array = resp.get("content", [])
		# Append the assistant turn verbatim (thinking blocks included).
		messages.append({"role": "assistant", "content": content})
		var stop := str(resp.get("stop_reason", ""))
		var text := ""
		var tool_uses := []
		for block: Dictionary in content:
			match str(block.get("type", "")):
				"text":
					text += str(block.text)
				"tool_use":
					tool_uses.append(block)
				"server_tool_use":
					var q: Variant = (block.get("input", {}) as Dictionary).get("query", "")
					progress.emit("[color=#88aaff]🔎 searching: %s[/color]" % q)
		if text.strip_edges() != "" and stop == "tool_use":
			progress.emit("[color=#cccccc]%s[/color]" % text.strip_edges())
		if stop == "refusal":
			return {"text": text, "messages": messages, "error": "Claude declined this request."}
		if stop == "pause_turn":
			continue  # server tool loop paused; resend to let it continue
		if stop != "tool_use":
			if stop == "max_tokens":
				return {"text": text, "messages": messages, "error": "Response hit the token limit."}
			return {"text": text, "messages": messages, "error": ""}
		var results := []
		for tu: Dictionary in tool_uses:
			progress.emit("[color=#ffcc66]⚙ %s[/color]" % tu.name)
			var input: Dictionary = tu.get("input", {}) if tu.get("input") is Dictionary else {}
			var out: Variant = await handler.call(str(tu.name), input)
			var r: Dictionary = out if out is Dictionary else {"content": str(out)}
			var block := {"type": "tool_result", "tool_use_id": tu.id, "content": r.get("content", "")}
			if bool(r.get("is_error", false)):
				block["is_error"] = true
			results.append(block)
		messages.append({"role": "user", "content": results})
	return {"text": "", "messages": messages, "error": "Agent stopped after %d turns." % MAX_TURNS}


static func image_block(img: Image, max_side := 1568, jpeg := true) -> Dictionary:
	var im := img.duplicate() as Image
	var longest := maxi(im.get_width(), im.get_height())
	if longest > max_side:
		var sc := float(max_side) / longest
		im.resize(int(im.get_width() * sc), int(im.get_height() * sc), Image.INTERPOLATE_BILINEAR)
	if im.get_format() != Image.FORMAT_RGB8 and jpeg:
		im.convert(Image.FORMAT_RGB8)
	var buf := im.save_jpg_to_buffer(0.85) if jpeg else im.save_png_to_buffer()
	return {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg" if jpeg else "image/png", "data": Marshalls.raw_to_base64(buf)}}
