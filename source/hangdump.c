/* hangdump.c -- hang reporter.
 *
 * Writes a new file sdmc:/switch/acpc_nx/hang_report_001.txt, _002, ... for each report, with every registered thread's
 * program counter, link register, stack pointer and a short frame-pointer call trace,
 * plus the process memory use, when
 *   - the Unity frame hook has not run for HD_STALL_SECONDS (the whole game is stuck), or
 *   - ZL+ZR+L+R are held for two seconds (the game still draws but a load never ends).
 *
 * Threads are paused only long enough to copy their registers and walk their stack into
 * static buffers: nothing between threadPause and threadResume allocates memory or takes
 * a lock, so the reporter cannot deadlock on a lock another thread holds. The text is
 * formatted and written after every thread is running again, with no malloc/fopen.
 *
 * When a report has been written the controller buzzes twice (once, long, if the SD write
 * failed). Build with -DHD_VIBRATE=0 to turn that off.
 */

#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <pthread.h>
#include <switch.h>

#include "hangdump.h"

#define HD_MAX_THREADS   128
#define HD_BT_DEPTH      16
#define HD_STALL_SECONDS 8
#define HD_COMBO_TICKS   8      /* 8 x 250 ms = 2 s */
#define HD_COOLDOWN_NS   (10ULL * 1000000000ULL)
#ifndef HD_REPORT_DIR
#define HD_REPORT_DIR    "sdmc:/switch/acpc_nx"
#endif
#define HD_MAX_REPORTS   50     /* after this many files the last one is overwritten */
#ifndef HD_VIBRATE
#define HD_VIBRATE 1
#endif

typedef struct {
  uintptr_t id;
  int       state;                /* 0 captured, 1 pause failed (already paused?), 2 no context */
  uint64_t  pc, lr, sp;
  uint64_t  bt[HD_BT_DEPTH];
  int       nbt;
} HdThread;

static volatile uintptr_t g_threads[HD_MAX_THREADS];
static HdThread  g_cap[HD_MAX_THREADS];
static int       g_ncap;
static char      g_buf[56 * 1024];
static uintptr_t g_il2cpp_base, g_unity_base;
static volatile uint64_t g_last_frame_ns;
static volatile uint64_t g_frames;
static Thread    g_thr;
static int       g_started;
static const char *g_report_dir = HD_REPORT_DIR;
static HidVibrationDeviceHandle g_vib[5];
static int       g_nvib;

static uint64_t hd_now_ns(void) { return armTicksToNs(armGetSystemTick()); }

void hangdump_frame(void) {
  g_last_frame_ns = hd_now_ns();
  g_frames++;
}

void hangdump_thread_register(void) {
  uintptr_t self = (uintptr_t)pthread_self();
  if (!self) return;
  for (int i = 0; i < HD_MAX_THREADS; i++) {
    uintptr_t expected = 0;
    if (__atomic_compare_exchange_n(&g_threads[i], &expected, self, 0,
                                    __ATOMIC_ACQ_REL, __ATOMIC_RELAXED))
      return;
  }
}

void hangdump_thread_unregister(void) {
  uintptr_t self = (uintptr_t)pthread_self();
  for (int i = 0; i < HD_MAX_THREADS; i++) {
    uintptr_t expected = self;
    if (__atomic_compare_exchange_n(&g_threads[i], &expected, 0, 0,
                                    __ATOMIC_ACQ_REL, __ATOMIC_RELAXED))
      return;
  }
}

static int hd_readable(uintptr_t a, size_t n) {
  MemoryInfo mi; u32 pi;
  if (R_FAILED(svcQueryMemory(&mi, &pi, a))) return 0;
  return (mi.perm & Perm_R) && a >= (uintptr_t)mi.addr &&
         a + n <= (uintptr_t)mi.addr + (uintptr_t)mi.size;
}

/* Only svcs and plain loads in here: it runs while the target thread is paused. */
static void hd_capture_one(HdThread *out, uintptr_t id) {
  Thread *t = (Thread *)id;
  memset(out, 0, sizeof *out);
  out->id = id;
  if (R_FAILED(threadPause(t))) { out->state = 1; return; }
  ThreadContext ctx;
  memset(&ctx, 0, sizeof ctx);
  if (R_FAILED(threadDumpContext(&ctx, t)) || !threadContextIsAArch64(&ctx)) {
    out->state = 2;
    threadResume(t);
    return;
  }
  out->pc = ctx.pc.x; out->lr = ctx.lr; out->sp = ctx.sp;
  uintptr_t fp = (uintptr_t)ctx.fp;
  while (out->nbt < HD_BT_DEPTH && fp && !(fp & 15) && hd_readable(fp, 16)) {
    const uint64_t *frame = (const uint64_t *)fp;
    uintptr_t next = (uintptr_t)frame[0];
    out->bt[out->nbt++] = frame[1];
    if (next <= fp) break;
    fp = next;
  }
  threadResume(t);
}

static void hd_capture_all(void) {
  g_ncap = 0;
  uintptr_t self = (uintptr_t)pthread_self();   /* the watchdog is not registered */
  for (int i = 0; i < HD_MAX_THREADS; i++) {
    uintptr_t id = __atomic_load_n(&g_threads[i], __ATOMIC_ACQUIRE);
    if (!id || id == self) continue;
    hd_capture_one(&g_cap[g_ncap++], id);
  }
}

/* Append "addr (il2cpp+0x.. / unity+0x..)" for an address. */
static size_t hd_addr(char *p, size_t room, uint64_t a) {
  if (!a) return (size_t)snprintf(p, room, "0 ");
  size_t n = (size_t)snprintf(p, room, "%llx", (unsigned long long)a);
  if (g_il2cpp_base && a >= g_il2cpp_base && a - g_il2cpp_base < 0x10000000ULL)
    n += (size_t)snprintf(p + n, room - n, "(i+%llx)", (unsigned long long)(a - g_il2cpp_base));
  if (g_unity_base && a >= g_unity_base && a - g_unity_base < 0x10000000ULL)
    n += (size_t)snprintf(p + n, room - n, "(u+%llx)", (unsigned long long)(a - g_unity_base));
  return n + (size_t)snprintf(p + n, room - n, " ");
}

static void hd_vib_init(void) {
#if HD_VIBRATE
  HidVibrationDeviceHandle h[2];
  g_nvib = 0;
  if (R_SUCCEEDED(hidInitializeVibrationDevices(h, 2, HidNpadIdType_Handheld, HidNpadStyleTag_NpadHandheld))) {
    g_vib[g_nvib++] = h[0]; g_vib[g_nvib++] = h[1];
  }
  if (R_SUCCEEDED(hidInitializeVibrationDevices(h, 2, HidNpadIdType_No1, HidNpadStyleTag_NpadJoyDual))) {
    g_vib[g_nvib++] = h[0]; g_vib[g_nvib++] = h[1];
  }
  if (R_SUCCEEDED(hidInitializeVibrationDevices(h, 1, HidNpadIdType_No1, HidNpadStyleTag_NpadFullKey)))
    g_vib[g_nvib++] = h[0];
#endif
}

static void hd_vib_set(float amp) {
  HidVibrationValue v;
  memset(&v, 0, sizeof v);
  v.amp_low = amp;  v.freq_low = 160.0f;
  v.amp_high = amp; v.freq_high = 320.0f;
  for (int i = 0; i < g_nvib; i++) hidSendVibrationValue(g_vib[i], &v);   /* errors ignored */
}

/* ok: two short buzzes. Not ok (SD write failed): one long buzz. Runs on the watchdog thread. */
static void hd_buzz(int ok) {
#if HD_VIBRATE
  if (!g_nvib) return;
  int pulses = ok ? 2 : 1;
  for (int i = 0; i < pulses; i++) {
    hd_vib_set(0.7f);
    svcSleepThread(ok ? 150000000LL : 700000000LL);
    hd_vib_set(0.0f);
    if (i + 1 < pulses) svcSleepThread(120000000LL);
  }
#else
  (void)ok;
#endif
}

static int hd_write_report(const char *reason, uint64_t now_ns) {
  size_t n = 0, cap = sizeof g_buf;
#define PUT(...) do { if (n < cap) { int w = snprintf(g_buf + n, cap - n, __VA_ARGS__); \
                      if (w > 0) n += (size_t)w; } } while (0)
  u64 total = 0, used = 0;
  svcGetInfo(&total, InfoType_TotalMemorySize, CUR_PROCESS_HANDLE, 0);
  svcGetInfo(&used, InfoType_UsedMemorySize, CUR_PROCESS_HANDLE, 0);
  uint64_t last = g_last_frame_ns;
  PUT("==== hang report: %s ====\n", reason);
  PUT("uptime_s=%llu frames=%llu last_frame_age_ms=%lld\n",
      (unsigned long long)(now_ns / 1000000000ULL), (unsigned long long)g_frames,
      last ? (long long)((now_ns - last) / 1000000ULL) : -1LL);
  PUT("memory used=%llu MiB of %llu MiB\n",
      (unsigned long long)(used >> 20), (unsigned long long)(total >> 20));
  PUT("il2cpp_base=%llx unity_base=%llx self_anchor(hangdump_init)=%llx\n",
      (unsigned long long)g_il2cpp_base, (unsigned long long)g_unity_base,
      (unsigned long long)(uintptr_t)&hangdump_init);
  PUT("threads=%d  (i+=offset in libil2cpp, u+=offset in libunity)\n", g_ncap);
  for (int i = 0; i < g_ncap; i++) {
    const HdThread *t = &g_cap[i];
    PUT("T%02d id=%llx ", i, (unsigned long long)t->id);
    if (t->state == 1) { PUT("pause failed (thread already paused or gone)\n"); continue; }
    if (t->state == 2) { PUT("no register context\n"); continue; }
    PUT("pc="); if (n < cap) n += hd_addr(g_buf + n, cap - n, t->pc);
    PUT("lr="); if (n < cap) n += hd_addr(g_buf + n, cap - n, t->lr);
    PUT("sp=%llx\n   bt:", (unsigned long long)t->sp);
    for (int k = 0; k < t->nbt; k++)
      if (n < cap) n += hd_addr(g_buf + n, cap - n, t->bt[k]);
    PUT("\n");
  }
  PUT("\n");
#undef PUT
  if (n >= cap) n = cap - 1;

  mkdir("sdmc:/switch", 0777);
  mkdir(g_report_dir, 0777);
  /* A new numbered file per report: the first hang_report_NNN.txt that does not exist yet. */
  char path[160];
  int fd = -1;
  for (int i = 1; i <= HD_MAX_REPORTS && fd < 0; i++) {
    snprintf(path, sizeof path, "%s/hang_report_%03d.txt", g_report_dir, i);
    fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
  }
  if (fd < 0)   /* every number is taken (or the card is not writable): reuse the last one */
    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  size_t off = 0;
  if (fd >= 0) {
    while (off < n) {
      ssize_t w = write(fd, g_buf + off, n - off);
      if (w <= 0) break;
      off += (size_t)w;
    }
    close(fd);
  }
  return fd >= 0 && off == n;
}

static void hd_dump(const char *reason) {
  hd_capture_all();
  hd_buzz(hd_write_report(reason, hd_now_ns()));
}

static void hd_thread_main(void *arg) {
  (void)arg;
  PadState pad;
  int pad_ok = R_SUCCEEDED(hidInitialize());
  if (pad_ok) { padInitializeDefault(&pad); hd_vib_init(); }
  const u64 combo = HidNpadButton_L | HidNpadButton_R | HidNpadButton_ZL | HidNpadButton_ZR;
  int held = 0, stall_reported = 0;
  uint64_t cooldown_until = 0;

  for (;;) {
    svcSleepThread(250000000ULL);
    uint64_t now = hd_now_ns();
    uint64_t last = g_last_frame_ns;

    if (last && now - last > (uint64_t)HD_STALL_SECONDS * 1000000000ULL) {
      if (!stall_reported) { stall_reported = 1; hd_dump("main loop stalled"); }
    } else {
      stall_reported = 0;
    }

    if (pad_ok) {
      padUpdate(&pad);
      if ((padGetButtons(&pad) & combo) == combo) held++; else held = 0;
      if (held >= HD_COMBO_TICKS && now >= cooldown_until) {
        held = 0;
        cooldown_until = now + HD_COOLDOWN_NS;
        hd_dump("manual (ZL+ZR+L+R)");
      }
    }
  }
}

void hangdump_init(uintptr_t il2cpp_base, uintptr_t unity_base) {
  if (g_started) return;
  g_il2cpp_base = il2cpp_base;
  g_unity_base = unity_base;
  hangdump_thread_register();            /* the calling (main) thread */
  if (R_FAILED(threadCreate(&g_thr, hd_thread_main, NULL, NULL, 0x10000, 0x2C, -2))) return;
  if (R_FAILED(threadStart(&g_thr))) { threadClose(&g_thr); return; }
  g_started = 1;
}
