// Assetto Corsa ksPerPixelMultiMap-style car paint: where the diffuse
// (livery) alpha is low, the tiled detail texture (base paint) shows through.
shader_type spatial;

uniform sampler2D tx_diffuse : source_color, filter_linear_mipmap, repeat_enable;
uniform sampler2D tx_detail : source_color, filter_linear_mipmap, repeat_enable;
uniform float detail_mult = 1.0;
uniform float use_detail = 1.0;
uniform float diffuse_k = 1.0;
uniform float rough = 0.3;
uniform float spec = 0.5;

void fragment() {
	vec4 d = texture(tx_diffuse, UV);
	vec3 c = d.rgb;
	if (use_detail > 0.5) {
		vec3 det = texture(tx_detail, UV * detail_mult).rgb;
		c = mix(det, d.rgb, d.a);
	}
	ALBEDO = c * diffuse_k;
	ROUGHNESS = rough;
	SPECULAR = spec;
}
