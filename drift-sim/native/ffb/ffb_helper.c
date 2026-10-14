/*
 * Drift Lab / DriftGame steering-wheel helper.
 *
 * Neither Godot nor our Unreal setup can drive DirectInput force feedback, so
 * the game launches this small program and talks to it over UDP (localhost).
 * Force feedback goes straight through DirectInput - a constant force
 * (self-aligning torque from the front tyres) plus damper/spring - the way
 * Assetto Corsa and most sims do it. The wheel's controls are read from the
 * same DirectInput device (SDL3 for wheels without force feedback) and
 * streamed back to the game.
 *
 * Packets in  (game -> helper, port 47811):
 *   "DLFB" f32 force(-1..1, +right) f32 damper(0..1) f32 spring(0..1) f32 gain(0..1)
 *   "DLQT"  -> quit
 *   "DLTS"  -> run a short left/right test
 * Packets out (helper -> game, port 47812): "DLST" + UTF-8 status text, and
 * every ~4 ms the wheel's controls (DriftGame reads its wheel through this):
 *   "DLIN" u8 axes u8 buttons u8 hat u8 0, f32 axis[axes] (-1..1, raw SDL
 *   order), u32 button bits (button i = bit i, first 32)
 *
 * Logitech's driver (G920/G29/G923 through G HUB) only accepts a one-axis
 * cartesian effect on the X axis with a non-zero direction, and takes the push
 * side from that direction, not the magnitude's sign. SDL's haptic layer never
 * got a force through to the G920, hence raw DirectInput.
 *
 * Safety: forces fade to zero if no force packet arrives for 250 ms; the
 * helper exits when the game process exits. Log: %TEMP%\DriftLab_ffb.log
 */
#define DIRECTINPUT_VERSION 0x0800
#include <SDL3/SDL.h>
#include <winsock2.h>
#include <windows.h>
#include <dinput.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define IN_PORT 47811
#define OUT_PORT 47812

static SOCKET sock_in = INVALID_SOCKET, sock_out = INVALID_SOCKET;
static FILE *log_file = NULL;

static void logmsg(const char *fmt, ...) {
	if (!log_file) return;
	va_list ap;
	va_start(ap, fmt);
	vfprintf(log_file, fmt, ap);
	va_end(ap);
	fputc('\n', log_file);
	fflush(log_file);
}
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
	logmsg("status: %s", buf + 4);
}

static int contains_ci(const char *hay, const char *needle) {
	if (!hay) return 0;
	size_t n = strlen(needle);
	for (; *hay; hay++)
		if (_strnicmp(hay, needle, n) == 0) return 1;
	return 0;
}

static int wheelish(const char *name) {
	return contains_ci(name, "wheel") || contains_ci(name, "G920") || contains_ci(name, "G29") ||
		contains_ci(name, "G923") || contains_ci(name, "Driving Force") || contains_ci(name, "Logitech") ||
		contains_ci(name, "Thrustmaster") || contains_ci(name, "Fanatec") || contains_ci(name, "MOZA") ||
		contains_ci(name, "Simucube");
}

/* --- Force feedback (DirectInput) ---------------------------------------- */

static HWND ff_window;
static LPDIRECTINPUT8 di;
static LPDIRECTINPUTDEVICE8 ff_dev;
static LPDIRECTINPUTEFFECT fx_const, fx_damper, fx_spring;
static DWORD ff_axis[1];
static LONG ff_dir[1] = {1};   /* sign = side (see ff_apply); magnitude stays >= 0 */
static DICONSTANTFORCE cf;
static DICONDITION damp_cond, spring_cond;
static DIEFFECT eff_const, eff_damper, eff_spring;
static char chosen[MAX_PATH] = "";
static char last_problem[300] = "";

/* The wheel's axes, sent as steering, throttle, brake, clutch, then the rest -
 * the order DriftGame's [DriftWheel] defaults expect. DirectInput's own order
 * differs per wheel (the G920 has a pedal on X and the wheel on Y), so they are
 * picked by the names the driver gives them ("Wheel axis", "Accelerator"...). */
static DWORD in_axis_ofs[8];
static int in_naxes;
static DWORD found_ofs[8];
static char found_name[8][MAX_PATH];
static int nfound_axes;
static BOOL CALLBACK on_axis(LPCDIDEVICEOBJECTINSTANCEA o, LPVOID ctx) {
	(void)ctx;
	if (nfound_axes < 8) {
		/* dwOfs is the device's native offset; where the axis lands in
		 * DIJOYSTATE2 depends on its type. */
		static int sliders;
		if (nfound_axes == 0) sliders = 0;
		DWORD ofs;
		if (IsEqualGUID(&o->guidType, &GUID_XAxis)) ofs = DIJOFS_X;
		else if (IsEqualGUID(&o->guidType, &GUID_YAxis)) ofs = DIJOFS_Y;
		else if (IsEqualGUID(&o->guidType, &GUID_ZAxis)) ofs = DIJOFS_Z;
		else if (IsEqualGUID(&o->guidType, &GUID_RxAxis)) ofs = DIJOFS_RX;
		else if (IsEqualGUID(&o->guidType, &GUID_RyAxis)) ofs = DIJOFS_RY;
		else if (IsEqualGUID(&o->guidType, &GUID_RzAxis)) ofs = DIJOFS_RZ;
		else if (IsEqualGUID(&o->guidType, &GUID_Slider) && sliders < 2) ofs = DIJOFS_SLIDER(sliders++);
		else return DIENUM_CONTINUE;
		found_ofs[nfound_axes] = ofs;
		snprintf(found_name[nfound_axes], MAX_PATH, "%s%s", o->tszName, (o->dwFlags & DIDOI_FFACTUATOR) ? " [ff]" : "");
		nfound_axes++;
	}
	return DIENUM_CONTINUE;
}

static void order_axes(void) {
	static const char *roles[4][4] = {
		{"wheel", "steer", "[ff]", NULL}, {"accel", "gas", "throttle", NULL},
		{"brake", NULL, NULL, NULL}, {"clutch", NULL, NULL, NULL}};
	int used[8] = {0};
	in_naxes = 0;
	for (int r = 0; r < 4; r++)
		for (int a = 0; a < nfound_axes; a++) {
			int hit = 0;
			for (int k = 0; k < 4 && roles[r][k]; k++) hit |= contains_ci(found_name[a], roles[r][k]);
			if (!used[a] && hit) { used[a] = 1; in_axis_ofs[in_naxes++] = found_ofs[a]; break; }
		}
	/* Unnamed or extra axes follow in offset order. */
	for (;;) {
		int best = -1;
		for (int a = 0; a < nfound_axes; a++)
			if (!used[a] && (best < 0 || found_ofs[a] < found_ofs[best])) best = a;
		if (best < 0) break;
		used[best] = 1;
		in_axis_ofs[in_naxes++] = found_ofs[best];
	}
	for (int a = 0; a < nfound_axes; a++) logmsg("  axis \"%s\" offset %lu", found_name[a], (unsigned long)found_ofs[a]);
	for (int a = 0; a < in_naxes; a++) logmsg("  sent as axis %d: offset %lu", a, (unsigned long)in_axis_ofs[a]);
}

static DIDEVICEINSTANCEA ff_found[8];
static int ff_nfound;
static BOOL CALLBACK on_ff_device(LPCDIDEVICEINSTANCEA d, LPVOID ctx) {
	(void)ctx;
	if (ff_nfound < 8) ff_found[ff_nfound++] = *d;
	return DIENUM_CONTINUE;
}

static void init_effect(DIEFFECT *e, void *params, DWORD size) {
	memset(e, 0, sizeof(*e));
	e->dwSize = sizeof(DIEFFECT);
	e->dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
	e->dwDuration = INFINITE;
	e->dwGain = DI_FFNOMINALMAX;
	e->dwTriggerButton = DIEB_NOTRIGGER;
	e->cAxes = 1;
	e->rgdwAxes = ff_axis;
	e->rglDirection = ff_dir;
	e->cbTypeSpecificParams = size;
	e->lpvTypeSpecificParams = params;
}

static void ff_release(void) {
	LPDIRECTINPUTEFFECT *all[3] = {&fx_const, &fx_damper, &fx_spring};
	for (int i = 0; i < 3; i++)
		if (*all[i]) { (*all[i])->lpVtbl->Stop(*all[i]); (*all[i])->lpVtbl->Release(*all[i]); *all[i] = NULL; }
	if (ff_dev) { ff_dev->lpVtbl->Unacquire(ff_dev); ff_dev->lpVtbl->Release(ff_dev); ff_dev = NULL; }
}

/* Opens the first force-feedback wheel (wheel-looking names first). */
static int find_ffb(void) {
	if (!di) return 0;
	ff_nfound = 0;
	di->lpVtbl->EnumDevices(di, DI8DEVCLASS_GAMECTRL, on_ff_device, NULL, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK);
	logmsg("scan: %d DirectInput force-feedback device(s)", ff_nfound);
	for (int pass = 0; pass < 2 && !fx_const; pass++) {
		for (int i = 0; i < ff_nfound && !fx_const; i++) {
			const char *name = ff_found[i].tszProductName;
			if (pass == 0) logmsg("  [%d] %s", i, name);
			if ((pass == 0) != (wheelish(name) != 0)) continue;
			HRESULT hr = di->lpVtbl->CreateDevice(di, &ff_found[i].guidInstance, &ff_dev, NULL);
			if (FAILED(hr)) { snprintf(last_problem, sizeof(last_problem), "can't open %s (0x%08lx)", name, (unsigned long)hr); continue; }
			ff_dev->lpVtbl->SetDataFormat(ff_dev, &c_dfDIJoystick2);
			/* Controls come from this same device: once it's acquired exclusively
			 * for force feedback, SDL's own DirectInput handle reads only zeros. */
			DIPROPRANGE range = {{sizeof(DIPROPRANGE), sizeof(DIPROPHEADER), 0, DIPH_DEVICE}, -32768, 32767};
			ff_dev->lpVtbl->SetProperty(ff_dev, DIPROP_RANGE, &range.diph);
			nfound_axes = 0;
			ff_dev->lpVtbl->EnumObjects(ff_dev, on_axis, NULL, DIDFT_AXIS);
			order_axes();
			hr = ff_dev->lpVtbl->SetCooperativeLevel(ff_dev, ff_window, DISCL_EXCLUSIVE | DISCL_BACKGROUND);
			if (FAILED(hr)) logmsg("  SetCooperativeLevel: 0x%08lx", (unsigned long)hr);
			DIPROPDWORD ac = {{sizeof(DIPROPDWORD), sizeof(DIPROPHEADER), 0, DIPH_DEVICE}, DIPROPAUTOCENTER_OFF};
			ff_dev->lpVtbl->SetProperty(ff_dev, DIPROP_AUTOCENTER, &ac.diph);
			hr = ff_dev->lpVtbl->Acquire(ff_dev);
			if (FAILED(hr)) {
				snprintf(last_problem, sizeof(last_problem), "%s is in use by another program (0x%08lx)", name, (unsigned long)hr);
				logmsg("%s", last_problem);
				ff_release();
				continue;
			}
			ff_axis[0] = DIJOFS_X;
			memset(&cf, 0, sizeof(cf));
			init_effect(&eff_const, &cf, sizeof(cf));
			hr = ff_dev->lpVtbl->CreateEffect(ff_dev, &GUID_ConstantForce, &eff_const, &fx_const, NULL);
			if (FAILED(hr)) {
				fx_const = NULL;
				snprintf(last_problem, sizeof(last_problem), "%s refused the steering force (0x%08lx)", name, (unsigned long)hr);
				logmsg("%s", last_problem);
				ff_release();
				continue;
			}
			fx_const->lpVtbl->Start(fx_const, 1, 0);
			memset(&damp_cond, 0, sizeof(damp_cond));
			damp_cond.dwPositiveSaturation = damp_cond.dwNegativeSaturation = DI_FFNOMINALMAX;
			init_effect(&eff_damper, &damp_cond, sizeof(damp_cond));
			if (FAILED(ff_dev->lpVtbl->CreateEffect(ff_dev, &GUID_Damper, &eff_damper, &fx_damper, NULL))) fx_damper = NULL;
			else fx_damper->lpVtbl->Start(fx_damper, 1, 0);
			spring_cond = damp_cond;
			init_effect(&eff_spring, &spring_cond, sizeof(spring_cond));
			if (FAILED(ff_dev->lpVtbl->CreateEffect(ff_dev, &GUID_Spring, &eff_spring, &fx_spring, NULL))) fx_spring = NULL;
			else fx_spring->lpVtbl->Start(fx_spring, 1, 0);
			snprintf(chosen, sizeof(chosen), "%s", name);
			last_problem[0] = 0;
			logmsg("force feedback + controls on %s (%d axes, damper %s, spring %s)", name, in_naxes, fx_damper ? "yes" : "no", fx_spring ? "yes" : "no");
		}
	}
	return fx_const != NULL;
}

static float shown_force = 0, shown_damper = -1, shown_spring = -1;

/* After a lost device (another app grabbed it, sleep, unplug): re-acquire and restart. */
static void ff_recover(HRESULT hr) {
	if (hr != DIERR_INPUTLOST && hr != DIERR_NOTACQUIRED && hr != DIERR_NOTEXCLUSIVEACQUIRED) return;
	if (SUCCEEDED(ff_dev->lpVtbl->Acquire(ff_dev))) {
		LPDIRECTINPUTEFFECT all[3] = {fx_const, fx_damper, fx_spring};
		for (int i = 0; i < 3; i++)
			if (all[i]) all[i]->lpVtbl->Start(all[i], 1, 0);
		shown_force = 0; shown_damper = shown_spring = -1;
	}
}

static void ff_apply(float f, float d, float s) {
	if (!fx_const) return;
	if (SDL_fabsf(f - shown_force) > 0.002f) {
		/* Logitech's driver ignores the magnitude's sign: push direction comes
		 * only from the direction vector (+1 pushes the G920 left), so send
		 * |force| and put the side in the direction. */
		cf.lMagnitude = (LONG)(SDL_fabsf(SDL_clamp(f, -1.0f, 1.0f)) * DI_FFNOMINALMAX);
		ff_dir[0] = f >= 0.0f ? -1 : 1;
		HRESULT hr = fx_const->lpVtbl->SetParameters(fx_const, &eff_const, DIEP_TYPESPECIFICPARAMS | DIEP_DIRECTION);
		if (FAILED(hr)) ff_recover(hr);
		else shown_force = f;
	}
	if (fx_damper && SDL_fabsf(d - shown_damper) > 0.01f) {
		damp_cond.lPositiveCoefficient = damp_cond.lNegativeCoefficient = (LONG)(SDL_clamp(d, 0.0f, 1.0f) * DI_FFNOMINALMAX);
		if (SUCCEEDED(fx_damper->lpVtbl->SetParameters(fx_damper, &eff_damper, DIEP_TYPESPECIFICPARAMS))) shown_damper = d;
	}
	if (fx_spring && SDL_fabsf(s - shown_spring) > 0.01f) {
		spring_cond.lPositiveCoefficient = spring_cond.lNegativeCoefficient = (LONG)(SDL_clamp(s, 0.0f, 1.0f) * DI_FFNOMINALMAX);
		if (SUCCEEDED(fx_spring->lpVtbl->SetParameters(fx_spring, &eff_spring, DIEP_TYPESPECIFICPARAMS))) shown_spring = s;
	}
}

/* Sends the FFB wheel's controls, read through DirectInput. */
static int send_input_di(void) {
	if (!ff_dev) return 0;
	DIJOYSTATE2 js;
	ff_dev->lpVtbl->Poll(ff_dev);
	HRESULT hr = ff_dev->lpVtbl->GetDeviceState(ff_dev, sizeof(js), &js);
	if (FAILED(hr)) {
		ff_recover(hr);
		return 1;
	}
	unsigned char buf[4 + 4 + 8 * 4 + 4];
	memcpy(buf, "DLIN", 4);
	buf[4] = (unsigned char)in_naxes;
	buf[5] = 32;
	DWORD pov = js.rgdwPOV[0];
	buf[6] = LOWORD(pov) == 0xFFFF ? 0 : (unsigned char)(1 + pov / 4500);   /* 0 = centred, 1..8 = N, NE.. */
	buf[7] = 0;
	int o = 8;
	for (int a = 0; a < in_naxes; a++) {
		LONG raw = *(const LONG *)((const unsigned char *)&js + in_axis_ofs[a]);
		float v = raw / 32767.0f;
		if (v < -1.0f) v = -1.0f;
		memcpy(buf + o, &v, 4);
		o += 4;
	}
	Uint32 bits = 0;
	for (int b = 0; b < 32; b++)
		if (js.rgbButtons[b] & 0x80) bits |= 1u << b;
	memcpy(buf + o, &bits, 4);
	o += 4;
	sendto(sock_out, (const char *)buf, o, 0, (struct sockaddr *)&out_addr, sizeof(out_addr));
	return 1;
}

/* --- Controls (SDL, for wheels without force feedback) ---------------------- */

static SDL_Joystick *wheel_js = NULL;

/* Opens the first wheel-looking joystick (wheels without force feedback still drive). */
static void open_input(void) {
	if (wheel_js) return;
	int jn = 0;
	SDL_JoystickID *js = SDL_GetJoysticks(&jn);
	for (int i = 0; i < jn && !wheel_js; i++)
		if (wheelish(SDL_GetJoystickNameForID(js[i]))) wheel_js = SDL_OpenJoystick(js[i]);
	SDL_free(js);
	if (wheel_js) logmsg("controls from: %s (%d axes, %d buttons)", SDL_GetJoystickName(wheel_js),
		SDL_GetNumJoystickAxes(wheel_js), SDL_GetNumJoystickButtons(wheel_js));
}

static void send_input(void) {
	if (send_input_di()) return;
	if (!wheel_js) return;
	SDL_UpdateJoysticks();
	if (!SDL_JoystickConnected(wheel_js)) {
		logmsg("wheel disconnected");
		SDL_CloseJoystick(wheel_js);
		wheel_js = NULL;
		return;
	}
	unsigned char buf[4 + 4 + 8 * 4 + 4];
	int na = SDL_GetNumJoystickAxes(wheel_js), nb = SDL_GetNumJoystickButtons(wheel_js);
	if (na > 8) na = 8;
	if (na < 0) na = 0;
	if (nb > 32) nb = 32;
	if (nb < 0) nb = 0;
	memcpy(buf, "DLIN", 4);
	buf[4] = (unsigned char)na;
	buf[5] = (unsigned char)nb;
	buf[6] = SDL_GetNumJoystickHats(wheel_js) > 0 ? SDL_GetJoystickHat(wheel_js, 0) : 0;
	buf[7] = 0;
	int o = 8;
	for (int a = 0; a < na; a++) {
		float v = SDL_GetJoystickAxis(wheel_js, a) / 32767.0f;
		if (v < -1.0f) v = -1.0f;
		memcpy(buf + o, &v, 4);
		o += 4;
	}
	Uint32 bits = 0;
	for (int b = 0; b < nb; b++)
		if (SDL_GetJoystickButton(wheel_js, b)) bits |= 1u << b;
	memcpy(buf + o, &bits, 4);
	o += 4;
	sendto(sock_out, (const char *)buf, o, 0, (struct sockaddr *)&out_addr, sizeof(out_addr));
}

int main(int argc, char **argv) {
	DWORD parent_pid = 0;
	for (int i = 1; i < argc - 1; i++)
		if (strcmp(argv[i], "--parent") == 0) parent_pid = (DWORD)strtoul(argv[i + 1], NULL, 10);
	HANDLE parent = parent_pid ? OpenProcess(SYNCHRONIZE, FALSE, parent_pid) : NULL;
	char logpath[MAX_PATH];
	DWORD tl = GetTempPathA(MAX_PATH, logpath);
	if (tl > 0 && tl < MAX_PATH - 32) {
		strcat(logpath, "DriftLab_ffb.log");
		log_file = fopen(logpath, "w");
	}
	logmsg("Drift Lab wheel helper (DirectInput FFB, SDL %d.%d.%d input), parent pid %lu", SDL_VERSIONNUM_MAJOR(SDL_GetVersion()),
		SDL_VERSIONNUM_MINOR(SDL_GetVersion()), SDL_VERSIONNUM_MICRO(SDL_GetVersion()), (unsigned long)parent_pid);

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

	/* DirectInput's exclusive mode needs a top-level window; it never shows. */
	HINSTANCE inst = GetModuleHandle(NULL);
	WNDCLASSA wc = {0};
	wc.lpfnWndProc = DefWindowProcA;
	wc.hInstance = inst;
	wc.lpszClassName = "DriftLabFfb";
	RegisterClassA(&wc);
	ff_window = CreateWindowA("DriftLabFfb", "Drift Lab FFB", WS_OVERLAPPEDWINDOW, 0, 0, 64, 64, NULL, NULL, inst, NULL);
	if (FAILED(DirectInput8Create(inst, DIRECTINPUT_VERSION, &IID_IDirectInput8A, (void **)&di, NULL))) {
		di = NULL;
		snprintf(last_problem, sizeof(last_problem), "DirectInput unavailable");
	}

	/* Controls through SDL, in the background (our window never has focus). */
	SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT, "0");
	SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
	if (!SDL_Init(SDL_INIT_JOYSTICK)) logmsg("SDL init failed: %s", SDL_GetError());

	find_ffb();
	if (!fx_const) open_input();
	Uint64 last_scan = SDL_GetTicks();
	float force = 0, damper = 0, spring = 0, gain = 1;
	Uint64 last_force = 0;
	Uint64 test_until = 0;
	Uint64 last_status = 0;
	Uint64 last_trace = 0;
	int trace = getenv("DRIFT_FFB_TRACE") != NULL;   /* or --trace */
	for (int i = 1; i < argc; i++)
		if (strcmp(argv[i], "--trace") == 0) trace = 1;
	int running = 1;
	while (running) {
		MSG msg;
		while (PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) DispatchMessageA(&msg);
		char buf[256];
		int n;
		while ((n = recv(sock_in, buf, sizeof(buf), 0)) >= 4) {
			if (memcmp(buf, "DLFB", 4) == 0 && n >= 20) {
				last_force = SDL_GetTicks();
				memcpy(&force, buf + 4, 4);
				memcpy(&damper, buf + 8, 4);
				memcpy(&spring, buf + 12, 4);
				memcpy(&gain, buf + 16, 4);
			} else if (memcmp(buf, "DLQT", 4) == 0) {
				running = 0;
			} else if (memcmp(buf, "DLTS", 4) == 0) {
				test_until = SDL_GetTicks() + 1600;
				logmsg("test requested");
			}
		}
		Uint64 now = SDL_GetTicks();
		if (parent && WaitForSingleObject(parent, 0) == WAIT_OBJECT_0) running = 0;
		send_input();
		if (!fx_const && now - last_scan > 3000) {
			last_scan = now;
			find_ffb();
			if (!fx_const && !wheel_js) open_input();
		}
		if (now - last_status > 1000) {
			last_status = now;
			if (fx_const) send_status("FFB ready: %s", chosen);
			else if (last_problem[0]) send_status("FFB problem: %s", last_problem);
			else send_status("No force-feedback wheel found yet (is G HUB running and the wheel plugged in?)");
		}
		float f = force, d = damper, s = spring;
		if (now - last_force > 250) { f = 0; d = 0; s = 0; } /* no live forces: go limp */
		if (test_until > now) {
			Uint64 left = test_until - now;
			f = (left > 800) ? -0.45f : 0.45f;
			d = 0.1f;
			s = 0;
		} else {
			if (gain < 0) gain = 0;
			if (gain > 1.5f) gain = 1.5f;
			f *= gain;
		}
		ff_apply(f, d, s);
		if (trace && now - last_trace >= 100) {
			last_trace = now;
			float wheel = 0.0f;
			DIJOYSTATE2 js;
			if (ff_dev && SUCCEEDED(ff_dev->lpVtbl->GetDeviceState(ff_dev, sizeof(js), &js)) && in_naxes > 0)
				wheel = *(const LONG *)((const unsigned char *)&js + in_axis_ofs[0]) / 32767.0f;
			else if (wheel_js)
				wheel = SDL_GetJoystickAxis(wheel_js, 0) / 32767.0f;
			logmsg("trace t=%llu force=%+.3f damper=%.2f spring=%.2f dir=%ld mag=%ld wheel=%+.3f",
				(unsigned long long)now, f, d, s, (long)ff_dir[0], (long)cf.lMagnitude, wheel);
		}
		SDL_Delay(2);
	}
	ff_release();
	if (di) di->lpVtbl->Release(di);
	if (wheel_js) SDL_CloseJoystick(wheel_js);
	SDL_Quit();
	closesocket(sock_in);
	closesocket(sock_out);
	WSACleanup();
	return 0;
}
