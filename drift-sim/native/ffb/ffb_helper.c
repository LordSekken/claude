/*
 * Drift Lab force-feedback helper.
 *
 * Godot has no API for DirectInput force feedback, so the game launches this
 * small SDL3 program and streams steering forces to it over UDP (localhost).
 * It drives a constant-force effect (self-aligning torque from the front
 * tyres) plus damper/spring effects on the first FFB wheel it finds
 * (Logitech G920/G29/G923, Thrustmaster, Fanatec, Moza... anything with
 * DirectInput FFB).
 *
 * Packets in  (game -> helper, port 47811):
 *   "DLFB" f32 force(-1..1, +right) f32 damper(0..1) f32 spring(0..1) f32 gain(0..1)
 *   "DLQT"  -> quit
 *   "DLTS"  -> run a short left/right test
 * Packets out (helper -> game, port 47812): "DLST" + UTF-8 status text.
 *
 * Safety: forces fade to zero if no packet arrives for 250 ms, and the helper
 * exits if the game process dies or after 20 s of silence.
 */
#include <SDL3/SDL.h>
#include <winsock2.h>
#include <windows.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define IN_PORT 47811
#define OUT_PORT 47812

static SOCKET sock_in = INVALID_SOCKET, sock_out = INVALID_SOCKET;
static struct sockaddr_in out_addr;

static void send_status(const char *fmt, ...) {
	char buf[512];
	memcpy(buf, "DLST", 4);
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf + 4, sizeof(buf) - 4, fmt, ap);
	va_end(ap);
	if (n < 0) n = 0;
	if (n > (int)sizeof(buf) - 5) n = sizeof(buf) - 5;
	sendto(sock_out, buf, 4 + n, 0, (struct sockaddr *)&out_addr, sizeof(out_addr));
	printf("%s\n", buf + 4);
	fflush(stdout);
}

static int contains_ci(const char *hay, const char *needle) {
	if (!hay) return 0;
	size_t n = strlen(needle);
	for (; *hay; hay++)
		if (_strnicmp(hay, needle, n) == 0) return 1;
	return 0;
}

static Sint16 to_level(float f) {
	if (f > 1.0f) f = 1.0f;
	if (f < -1.0f) f = -1.0f;
	return (Sint16)(f * 32767.0f);
}

int main(int argc, char **argv) {
	DWORD parent_pid = 0;
	for (int i = 1; i < argc - 1; i++)
		if (strcmp(argv[i], "--parent") == 0) parent_pid = (DWORD)strtoul(argv[i + 1], NULL, 10);
	HANDLE parent = parent_pid ? OpenProcess(SYNCHRONIZE, FALSE, parent_pid) : NULL;

	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
	sock_in = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sock_out = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in in_addr = {0};
	in_addr.sin_family = AF_INET;
	in_addr.sin_port = htons(IN_PORT);
	in_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	out_addr = in_addr;
	out_addr.sin_port = htons(OUT_PORT);
	if (bind(sock_in, (struct sockaddr *)&in_addr, sizeof(in_addr)) != 0) {
		send_status("FFB helper: port %d busy (another copy running?)", IN_PORT);
		return 1;
	}
	u_long nonblock = 1;
	ioctlsocket(sock_in, FIONBIO, &nonblock);

	/* Use DirectInput for wheels (needed for force feedback). */
	SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT, "0");
	if (!SDL_Init(SDL_INIT_HAPTIC | SDL_INIT_JOYSTICK)) {
		send_status("FFB helper: SDL init failed: %s", SDL_GetError());
		return 1;
	}

	int count = 0;
	SDL_HapticID *ids = SDL_GetHaptics(&count);
	SDL_Haptic *haptic = NULL;
	const char *chosen = NULL;
	/* Prefer something that looks like a wheel, then anything with constant force. */
	for (int pass = 0; pass < 2 && !haptic; pass++) {
		for (int i = 0; i < count && !haptic; i++) {
			const char *name = SDL_GetHapticNameForID(ids[i]);
			int wheelish = contains_ci(name, "wheel") || contains_ci(name, "G920") || contains_ci(name, "G29") ||
				contains_ci(name, "G923") || contains_ci(name, "Driving Force") || contains_ci(name, "Thrustmaster") ||
				contains_ci(name, "Fanatec") || contains_ci(name, "MOZA") || contains_ci(name, "Simucube");
			if (pass == 0 && !wheelish) continue;
			SDL_Haptic *h = SDL_OpenHaptic(ids[i]);
			if (!h) continue;
			if (SDL_GetHapticFeatures(h) & SDL_HAPTIC_CONSTANT) {
				haptic = h;
				chosen = name;
			} else {
				SDL_CloseHaptic(h);
			}
		}
	}
	SDL_free(ids);
	if (!haptic) {
		send_status("No force-feedback wheel found (%d haptic device(s)). Is G HUB running and the wheel plugged in?", count);
	}

	int eff_const = -1, eff_damper = -1, eff_spring = -1;
	SDL_HapticEffect ce, de, se;
	Uint32 features = 0;
	if (haptic) {
		features = SDL_GetHapticFeatures(haptic);
		if (features & SDL_HAPTIC_GAIN) SDL_SetHapticGain(haptic, 100);
		if (features & SDL_HAPTIC_AUTOCENTER) SDL_SetHapticAutocenter(haptic, 0);
		SDL_zero(ce);
		ce.type = SDL_HAPTIC_CONSTANT;
		ce.constant.direction.type = SDL_HAPTIC_CARTESIAN;
		ce.constant.direction.dir[0] = 1; /* +X: level > 0 pushes the wheel right */
		ce.constant.length = SDL_HAPTIC_INFINITY;
		ce.constant.level = 0;
		eff_const = SDL_CreateHapticEffect(haptic, &ce);
		if (eff_const >= 0) SDL_RunHapticEffect(haptic, eff_const, SDL_HAPTIC_INFINITY);
		if (features & SDL_HAPTIC_DAMPER) {
			SDL_zero(de);
			de.type = SDL_HAPTIC_DAMPER;
			de.condition.length = SDL_HAPTIC_INFINITY;
			for (int a = 0; a < 3; a++) { de.condition.right_sat[a] = de.condition.left_sat[a] = 0xFFFF; }
			eff_damper = SDL_CreateHapticEffect(haptic, &de);
			if (eff_damper >= 0) SDL_RunHapticEffect(haptic, eff_damper, SDL_HAPTIC_INFINITY);
		}
		if (features & SDL_HAPTIC_SPRING) {
			SDL_zero(se);
			se.type = SDL_HAPTIC_SPRING;
			se.condition.length = SDL_HAPTIC_INFINITY;
			for (int a = 0; a < 3; a++) { se.condition.right_sat[a] = se.condition.left_sat[a] = 0xFFFF; }
			eff_spring = SDL_CreateHapticEffect(haptic, &se);
			if (eff_spring >= 0) SDL_RunHapticEffect(haptic, eff_spring, SDL_HAPTIC_INFINITY);
		}
		send_status("FFB ready: %s (constant%s%s)", chosen ? chosen : "wheel",
			eff_damper >= 0 ? ", damper" : "", eff_spring >= 0 ? ", spring" : "");
	}

	float force = 0, damper = 0, spring = 0, gain = 1;
	float shown_force = 0, shown_damper = -1, shown_spring = -1;
	Uint64 last_packet = SDL_GetTicks();
	Uint64 test_until = 0;
	Uint64 last_status = 0;
	int running = 1;
	while (running) {
		char buf[256];
		int n;
		while ((n = recv(sock_in, buf, sizeof(buf), 0)) >= 4) {
			last_packet = SDL_GetTicks();
			if (memcmp(buf, "DLFB", 4) == 0 && n >= 20) {
				memcpy(&force, buf + 4, 4);
				memcpy(&damper, buf + 8, 4);
				memcpy(&spring, buf + 12, 4);
				memcpy(&gain, buf + 16, 4);
			} else if (memcmp(buf, "DLQT", 4) == 0) {
				running = 0;
			} else if (memcmp(buf, "DLTS", 4) == 0) {
				test_until = SDL_GetTicks() + 1600;
			} else if (memcmp(buf, "DLHI", 4) == 0) {
				last_status = 0; /* re-announce */
			}
		}
		Uint64 now = SDL_GetTicks();
		if (parent && WaitForSingleObject(parent, 0) == WAIT_OBJECT_0) running = 0;
		if (now - last_packet > 20000) running = 0;
		if (now - last_status > 2000) {
			last_status = now;
			if (haptic) send_status("FFB ready: %s", chosen ? chosen : "wheel");
			else send_status("No force-feedback wheel found. Is G HUB running and the wheel plugged in?");
		}
		float f = force, d = damper, s = spring;
		if (now - last_packet > 250) { f = 0; d = 0; s = 0; } /* stale: go limp */
		if (test_until > now) {
			Uint64 left = test_until - now;
			f = (left > 800) ? -0.45f : 0.45f;
			d = 0.1f;
			s = 0;
		}
		if (gain < 0) gain = 0;
		if (gain > 1.5f) gain = 1.5f;
		f *= gain;
		if (haptic) {
			if (eff_const >= 0 && SDL_fabsf(f - shown_force) > 0.002f) {
				ce.constant.level = to_level(f);
				SDL_UpdateHapticEffect(haptic, eff_const, &ce);
				shown_force = f;
			}
			if (eff_damper >= 0 && SDL_fabsf(d - shown_damper) > 0.01f) {
				Uint16 c = (Uint16)(SDL_clamp(d, 0.0f, 1.0f) * 32767.0f);
				for (int a = 0; a < 3; a++) { de.condition.right_coeff[a] = c; de.condition.left_coeff[a] = c; }
				SDL_UpdateHapticEffect(haptic, eff_damper, &de);
				shown_damper = d;
			}
			if (eff_spring >= 0 && SDL_fabsf(s - shown_spring) > 0.01f) {
				Uint16 c = (Uint16)(SDL_clamp(s, 0.0f, 1.0f) * 32767.0f);
				for (int a = 0; a < 3; a++) { se.condition.right_coeff[a] = c; se.condition.left_coeff[a] = c; }
				SDL_UpdateHapticEffect(haptic, eff_spring, &se);
				shown_spring = s;
			}
		}
		SDL_Delay(2);
	}
	if (haptic) {
		SDL_StopHapticEffects(haptic);
		SDL_CloseHaptic(haptic);
	}
	SDL_Quit();
	closesocket(sock_in);
	closesocket(sock_out);
	WSACleanup();
	return 0;
}
