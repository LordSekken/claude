/*
 * Assetto Corsa telemetry recorder (Drift Lab / DriftGame tuning).
 *
 * AC publishes its live state in shared memory ("Local\acpmf_physics",
 * "acpmf_graphics", "acpmf_static") whenever a session is running - no
 * Content Manager setup needed. This records the physics page at 100 Hz to
 * CSV while you drive, one file per session:
 *   Documents\DriftLab\telemetry\ac_<car>_<track>_<date>.csv
 * Columns: speed, gear, rpm, pedals, steering, AC's force feedback (finalFF),
 * attitude, local velocity / yaw rate (drift angle), per-wheel slip, load,
 * suspension, and - when this AC version publishes them - per-tyre slip
 * angle/ratio, Fy and Mz (self-aligning torque).
 * Run it before or after starting AC; close the window to stop.
 */
#include <windows.h>
#include <shlobj.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#pragma pack(push, 4)
typedef struct {
	int packetId;
	float gas, brake, fuel;
	int gear, rpms;
	float steerAngle, speedKmh;
	float velocity[3], accG[3];
	float wheelSlip[4], wheelLoad[4], wheelsPressure[4], wheelAngularSpeed[4];
	float tyreWear[4], tyreDirtyLevel[4], tyreCoreTemperature[4], camberRAD[4], suspensionTravel[4];
	float drs, tc, heading, pitch, roll, cgHeight;
	float carDamage[5];
	int numberOfTyresOut, pitLimiterOn;
	float abs, kersCharge, kersInput;
	int autoShifterOn;
	float rideHeight[2];
	float turboBoost, ballast, airDensity, airTemp, roadTemp;
	float localAngularVel[3];
	float finalFF;
	float performanceMeter;
	int engineBrake, ersRecoveryLevel, ersPowerLevel, ersHeatCharging, ersIsCharging;
	float kersCurrentKJ;
	int drsAvailable, drsEnabled;
	float brakeTemp[4];
	float clutch;
	float tyreTempI[4], tyreTempM[4], tyreTempO[4];
	int isAIControlled;
	float tyreContactPoint[4][3], tyreContactNormal[4][3], tyreContactHeading[4][3];
	float brakeBias;
	float localVelocity[3];
	/* Newer AC builds append these. */
	int P2PActivations, P2PStatus, currentMaxRpm;
	float mz[4], fx[4], fy[4], slipRatio[4], slipAngle[4];
} AcPhysics;

typedef struct {
	int packetId;
	int status;   /* 0 off, 1 replay, 2 live, 3 pause */
	int session;
	wchar_t currentTime[15], lastTime[15], bestTime[15], split[15];
	int completedLaps, position, iCurrentTime, iLastTime, iBestTime;
	float sessionTimeLeft, distanceTraveled;
	int isInPit, currentSectorIndex, lastSectorTime, numberOfLaps;
	wchar_t tyreCompound[33];
	float replayTimeMultiplier, normalizedCarPosition;
	float carCoordinates[3];
} AcGraphics;

typedef struct {
	wchar_t smVersion[15], acVersion[15];
	int numberOfSessions, numCars;
	wchar_t carModel[33], track[33], playerName[33], playerSurname[33], playerNick[33];
	int sectorCount;
	float maxTorque, maxPower;
	int maxRpm;
	float maxFuel;
	float suspensionMaxTravel[4], tyreRadius[4];
} AcStatic;
#pragma pack(pop)

typedef struct {
	HANDLE map;
	const void *view;
	SIZE_T size;
} Shm;

static int shm_open(Shm *s, const char *name) {
	if (s->view) return 1;
	s->map = OpenFileMappingA(FILE_MAP_READ, FALSE, name);
	if (!s->map) return 0;
	s->view = MapViewOfFile(s->map, FILE_MAP_READ, 0, 0, 0);
	if (!s->view) { CloseHandle(s->map); s->map = NULL; return 0; }
	MEMORY_BASIC_INFORMATION mbi;
	s->size = VirtualQuery(s->view, &mbi, sizeof(mbi)) ? mbi.RegionSize : 0;
	return 1;
}

static void shm_close(Shm *s) {
	if (s->view) UnmapViewOfFile(s->view);
	if (s->map) CloseHandle(s->map);
	memset(s, 0, sizeof(*s));
}

static void clean(char *out, const wchar_t *in, int n) {
	int o = 0;
	for (int i = 0; i < n && in[i] && o < 60; i++) {
		wchar_t c = in[i];
		out[o++] = (c < 128 && (iswalnum(c) || c == '-' || c == '_')) ? (char)c : '_';
	}
	out[o] = 0;
	if (!o) strcpy(out, "unknown");
}

static FILE *open_csv(const AcStatic *st, int extended, char *path_out) {
	char docs[MAX_PATH], car[64], track[64];
	if (FAILED(SHGetFolderPathA(NULL, CSIDL_PERSONAL, NULL, 0, docs))) strcpy(docs, ".");
	char dir[MAX_PATH];
	snprintf(dir, sizeof(dir), "%s\\DriftLab", docs);
	CreateDirectoryA(dir, NULL);
	snprintf(dir, sizeof(dir), "%s\\DriftLab\\telemetry", docs);
	CreateDirectoryA(dir, NULL);
	clean(car, st->carModel, 33);
	clean(track, st->track, 33);
	SYSTEMTIME t;
	GetLocalTime(&t);
	snprintf(path_out, MAX_PATH, "%s\\ac_%s_%s_%04d-%02d-%02d_%02d-%02d-%02d.csv", dir, car, track,
		t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
	FILE *f = fopen(path_out, "w");
	if (!f) return NULL;
	fprintf(f, "# Assetto Corsa telemetry, car=%s track=%s ac=%ls sm=%ls\n", car, track, st->acVersion, st->smVersion);
	fprintf(f, "t,speed_kmh,gear,rpm,gas,brake,clutch,steer,final_ff,heading,pitch,roll,"
		"vel_x,vel_y,vel_z,yaw_rate,pitch_rate,roll_rate,acc_x,acc_y,acc_z,pos_x,pos_y,pos_z");
	const char *w[4] = {"fl", "fr", "rl", "rr"};
	const char *per[] = {"slip", "load", "omega", "susp"};
	for (int k = 0; k < 4; k++)
		for (int i = 0; i < 4; i++) fprintf(f, ",%s_%s", per[k], w[i]);
	if (extended) {
		const char *ext[] = {"slip_angle", "slip_ratio", "fy", "mz"};
		for (int k = 0; k < 4; k++)
			for (int i = 0; i < 4; i++) fprintf(f, ",%s_%s", ext[k], w[i]);
	}
	fputc('\n', f);
	return f;
}

int main(void) {
	SetConsoleTitleA("AC telemetry recorder (Drift Lab)");
	printf("Assetto Corsa telemetry recorder\n");
	printf("Start a session in Assetto Corsa and drive - recording starts automatically.\n");
	printf("Close this window when you're done.\n\n");
	Shm phys = {0}, gfx = {0}, stat = {0};
	FILE *csv = NULL;
	char path[MAX_PATH] = "";
	int extended = 0, last_packet = -1, rows = 0, waiting_shown = 0;
	LARGE_INTEGER freq, t0, now;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&t0);
	double next_sample = 0, last_print = 0;
	for (;;) {
		if (!phys.view && (!shm_open(&phys, "Local\\acpmf_physics") || !shm_open(&gfx, "Local\\acpmf_graphics") ||
			!shm_open(&stat, "Local\\acpmf_static"))) {
			shm_close(&phys); shm_close(&gfx); shm_close(&stat);
			if (!waiting_shown) { printf("Waiting for Assetto Corsa...\n"); waiting_shown = 1; }
			Sleep(1000);
			continue;
		}
		const AcPhysics *p = (const AcPhysics *)phys.view;
		const AcGraphics *g = (const AcGraphics *)gfx.view;
		const AcStatic *st = (const AcStatic *)stat.view;
		QueryPerformanceCounter(&now);
		double t = (double)(now.QuadPart - t0.QuadPart) / (double)freq.QuadPart;
		if (g->status != 2) {   /* not live: finish the current file */
			if (csv) {
				fclose(csv);
				csv = NULL;
				printf("\nSaved %d samples (%.0f s) to\n  %s\n\n", rows, rows / 100.0, path);
			}
			if (g->status == 0 && t - last_print > 5) {   /* AC closed: release the maps */
				shm_close(&phys); shm_close(&gfx); shm_close(&stat);
				waiting_shown = 0;
			}
			Sleep(50);
			continue;
		}
		if (!csv) {
			extended = phys.size >= sizeof(AcPhysics);
			csv = open_csv(st, extended, path);
			if (!csv) { printf("Can't write %s\n", path); Sleep(2000); continue; }
			rows = 0;
			next_sample = t;
			printf("Recording %ls on %ls%s\n  -> %s\n", st->carModel, st->track,
				extended ? "" : " (this AC build has no per-tyre forces; recording the rest)", path);
		}
		if (t < next_sample || p->packetId == last_packet) { Sleep(1); continue; }
		next_sample += 0.01;
		if (next_sample < t) next_sample = t + 0.01;
		last_packet = p->packetId;
		fprintf(csv, "%.3f,%.2f,%d,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
			rows / 100.0, p->speedKmh, p->gear - 1, p->rpms, p->gas, p->brake, p->clutch, p->steerAngle, p->finalFF,
			p->heading, p->pitch, p->roll, p->localVelocity[0], p->localVelocity[1], p->localVelocity[2],
			p->localAngularVel[1], p->localAngularVel[0], p->localAngularVel[2], p->accG[0], p->accG[1], p->accG[2],
			g->carCoordinates[0], g->carCoordinates[1], g->carCoordinates[2]);
		const float *per[4] = {p->wheelSlip, p->wheelLoad, p->wheelAngularSpeed, p->suspensionTravel};
		for (int k = 0; k < 4; k++)
			for (int i = 0; i < 4; i++) fprintf(csv, ",%.4f", per[k][i]);
		if (extended) {
			const float *ext[4] = {p->slipAngle, p->slipRatio, p->fy, p->mz};
			for (int k = 0; k < 4; k++)
				for (int i = 0; i < 4; i++) fprintf(csv, ",%.4f", ext[k][i]);
		}
		fputc('\n', csv);
		rows++;
		if (t - last_print > 1.0) {
			last_print = t;
			double drift = 0;
			if (fabs(p->localVelocity[2]) > 1.0) drift = atan2(p->localVelocity[0], p->localVelocity[2]) * 57.2958;
			printf("\r  %5.0f s  %3.0f km/h  gear %d  drift %+4.0f deg  FFB %+.2f     ", rows / 100.0, p->speedKmh, p->gear - 1, drift, p->finalFF);
			fflush(stdout);
		}
	}
}
