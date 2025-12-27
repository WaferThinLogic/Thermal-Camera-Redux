#pragma once

#include <stddef.h>
#include <cstdio>

#ifndef P1_CAMERA
#define P1_CAMERA 0
#endif

#define P1_VENDOR_ID 0x3474
#define P1_PRODUCT_ID 0x45c2

#define P1_FRAME_SIZE 77452
#define P1_FRAME_SKIP 12
#define P1_DROP_SMALL 12
#define P1_WIDTH 160
#define P1_HEIGHT 120
#define P1_INFO_HEIGHT 2

#if P1_CAMERA

#ifdef __cplusplus
extern "C" {
#endif

typedef struct P1Camera P1Camera;

typedef struct {
	unsigned long total;
	unsigned long short_packets;
	unsigned long counts[513];
	unsigned long good_frames;
	unsigned long bad_frames;
	unsigned long last_frame_bytes;
	unsigned long last_short_size;
	unsigned long short_match;
	unsigned long short_mismatch;
	unsigned char last_short_bytes[24];
	unsigned long last_short_len;
	unsigned long buffer_bytes;
	unsigned long buffer_max;
	unsigned long drop_events;
	unsigned long drop_small;
	unsigned long full_packets;
	unsigned long partial_packets;
	unsigned long last_transfer_size;
	unsigned long last_frame_transfers;
} P1TransferStats;

P1Camera *p1_open(int quiet);
void p1_close(P1Camera *cam);
void p1_free(P1Camera *cam);
int p1_read_frame(P1Camera *cam, unsigned char *out, size_t out_len, int timeout_ms, size_t *bytes_read);
void p1_set_range_mode(P1Camera *cam, int mode, unsigned long interval_ms);
void p1_set_raw_dump(P1Camera *cam, FILE *fp, size_t max_bytes);
void p1_set_size_log(P1Camera *cam, FILE *fp, size_t max_packets);
void p1_get_transfer_stats(P1Camera *cam, P1TransferStats *out);
int p1_device_present(void);

#ifdef __cplusplus
}
#endif

#else

typedef struct P1Camera {
	int unused;
} P1Camera;

static inline P1Camera *p1_open(int quiet) {
	(void)quiet;
	return NULL;
}

static inline void p1_close(P1Camera *cam) {
	(void)cam;
}

static inline void p1_free(P1Camera *cam) {
	(void)cam;
}

static inline int p1_read_frame(P1Camera *cam, unsigned char *out, size_t out_len, int timeout_ms) {
	(void)cam;
	(void)out;
	(void)out_len;
	(void)timeout_ms;
	return 0;
}

#endif
