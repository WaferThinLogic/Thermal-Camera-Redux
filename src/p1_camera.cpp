// p1_camera.h included here for P1_CAMERA constants and types.
#include "p1_camera.h"

#if P1_CAMERA

#include <libusb-1.0/libusb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <stddef.h>
#include <vector>

#define P1_IFACE_CMD 0
#define P1_IFACE_STREAM 1
#define P1_ALT_STREAM 1

#define P1_EP_CMD_OUT 0x05
#define P1_EP_CMD_IN 0x84
#define P1_EP_STREAM_IN 0x81
#define P1_EP_STREAM_OUT 0x02

#define P1_CTRL_TIMEOUT_MS 1000
#define P1_BULK_CHUNK (1024 * 128)

#define P1_SCAN_MAX 4096
#define P1_BUF_MAX (P1_FRAME_SIZE * 2 + P1_SCAN_MAX)

static int p1_frame_looks_valid(const unsigned char *frame, size_t frame_len) {
	if (frame_len < P1_FRAME_SIZE) {
		return 0;
	}
	const size_t ir_len = P1_WIDTH * P1_HEIGHT * 2;
	const size_t info_len = P1_WIDTH * P1_INFO_HEIGHT * 2;
	const size_t temp_len = P1_WIDTH * P1_HEIGHT * 2;
	const size_t payload_len = P1_FRAME_SIZE - P1_FRAME_SKIP;
	if (payload_len < ir_len + info_len + temp_len) {
		return 0;
	}
	const unsigned char *payload = frame + P1_FRAME_SKIP;
	const unsigned char *temp_ptr = payload + ir_len + info_len;

	int temp_samples = 0;
	int temp_good = 0;
	for (size_t i = 0; i + 1 < temp_len; i += 512) {
		unsigned short v = (unsigned short)(temp_ptr[i] | (temp_ptr[i + 1] << 8));
		temp_samples++;
		if (v >= 8000 && v <= 45000) {
			temp_good++;
		}
	}
	if (temp_samples == 0) {
		return 0;
	}
	double temp_ratio = (double)temp_good / (double)temp_samples;
	if (temp_ratio < 0.6) {
		return 0;
	}

	int uv_samples = 0;
	int uv_sum = 0;
	for (size_t i = 1; i < ir_len; i += 256) {
		uv_sum += payload[i];
		uv_samples++;
	}
	if (uv_samples == 0) {
		return 0;
	}
	int uv_mean = uv_sum / uv_samples;
	if (uv_mean < 60 || uv_mean > 200) {
		return 0;
	}
	return 1;
}

static int p1_find_frame_offset(const std::vector<unsigned char> &buf, size_t frame_len) {
	if (buf.size() < frame_len) {
		return -1;
	}
	size_t max_offset = buf.size() - frame_len;
	if (max_offset > P1_SCAN_MAX) {
		max_offset = P1_SCAN_MAX;
	}
	for (size_t off = 0; off <= max_offset; off += 4) {
		if (p1_frame_looks_valid(buf.data() + off, frame_len)) {
			return (int)off;
		}
	}
	return -1;
}

static const unsigned char kUsbmonInitPayloads[][18] = {
	{0x01,0x01,0x81,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x1f,0x63},
	{0x01,0x01,0x81,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x10,0x4c},
	{0x01,0x01,0x81,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x65,0x4f},
	{0x01,0x01,0x81,0x00,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x19,0x59},
	{0x01,0x2f,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x49,0x30},
	{0x10,0x02,0x41,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x3d},
	{0x10,0x04,0x47,0x00,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0xbc},
	{0x10,0x04,0x4a,0x00,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,0xd1},
	{0x10,0x03,0x4b,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2d,0x8e},
};

struct P1Camera {
	libusb_context *ctx;
	libusb_device_handle *handle;
	std::vector<unsigned char> buf;
	std::vector<unsigned char> chunk;
	int quiet;
	unsigned long last_heartbeat_ms;
	unsigned long last_range_ms;
	unsigned long range_interval_ms;
	int range_mode;
	int synced;
	int use_stream_cmds;
	size_t discard_next;
	FILE *raw_fp;
	size_t raw_limit;
	size_t raw_written;
	FILE *sizes_fp;
	size_t sizes_limit;
	size_t sizes_written;
	P1TransferStats stats;
};

static int p1_ctrl_transfer(libusb_device_handle *handle, const unsigned char *data, int len) {
	return libusb_control_transfer(
		handle,
		0x41,
		0x20,
		0x0000,
		0x0000,
		(unsigned char *)data,
		(uint16_t)len,
		P1_CTRL_TIMEOUT_MS
	);
}

static void p1_usbmon_init(libusb_device_handle *handle) {
	libusb_control_transfer(handle, 0x40, 0xEE, 0x0000, 0x0001, NULL, 0, P1_CTRL_TIMEOUT_MS);
	for (size_t i = 0; i < (sizeof(kUsbmonInitPayloads) / sizeof(kUsbmonInitPayloads[0])); i++) {
		p1_ctrl_transfer(handle, kUsbmonInitPayloads[i], (int)sizeof(kUsbmonInitPayloads[i]));
	}
}

static unsigned long p1_time_ms() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (unsigned long)((tv.tv_sec * 1000UL) + (tv.tv_usec / 1000UL));
}

static unsigned short p1_crc16_ccitt(const unsigned char *data, size_t len) {
	unsigned short crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= (unsigned short)(data[i] << 8);
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (unsigned short)((crc << 1) ^ 0x1021);
			} else {
				crc <<= 1;
			}
			crc &= 0xFFFF;
		}
	}
	return crc;
}

static int p1_send_ctrl_header(libusb_device_handle *handle, const unsigned char *raw16) {
	unsigned char pkt[18];
	memcpy(pkt, raw16, 16);
	unsigned short crc = p1_crc16_ccitt(pkt, 16);
	pkt[16] = (unsigned char)(crc & 0xFF);
	pkt[17] = (unsigned char)((crc >> 8) & 0xFF);
	return p1_ctrl_transfer(handle, pkt, (int)sizeof(pkt));
}

static int p1_send_ircmd(libusb_device_handle *handle, const unsigned char *raw16) {
	unsigned char pkt[18];
	memcpy(pkt, raw16, 16);
	unsigned short crc = p1_crc16_ccitt(pkt, 16);
	pkt[16] = (unsigned char)(crc & 0xFF);
	pkt[17] = (unsigned char)((crc >> 8) & 0xFF);

	int transferred = 0;
	int rc = libusb_bulk_transfer(handle, P1_EP_CMD_OUT, pkt, (int)sizeof(pkt), &transferred, P1_CTRL_TIMEOUT_MS);
	return (rc == 0 && transferred == (int)sizeof(pkt)) ? 1 : 0;
}

static int p1_send_ircmd_ctrl(libusb_device_handle *handle, const unsigned char *raw16) {
	unsigned char pkt[18];
	memcpy(pkt, raw16, 16);
	unsigned short crc = p1_crc16_ccitt(pkt, 16);
	pkt[16] = (unsigned char)(crc & 0xFF);
	pkt[17] = (unsigned char)((crc >> 8) & 0xFF);
	return (p1_ctrl_transfer(handle, pkt, (int)sizeof(pkt)) >= 0) ? 1 : 0;
}

static unsigned char p1_map_preview_format(unsigned char fmt) {
	// Match p1_usb_tool preview_start_header mapping.
	if (fmt == 1) return 2;
	if (fmt == 7) return 3;
	if (fmt == 3) return 1;
	return fmt;
}

static void p1_start_stream(libusb_device_handle *handle, unsigned char source, unsigned char fmt, unsigned char status) {
	unsigned char hb_start[16] = {0x10,0x10,0x53,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char preview_start[16] = {0x10,0x10,0x4d,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char stream_continue[16] = {0x10,0x05,0x41,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	preview_start[4] = source;
	preview_start[5] = p1_map_preview_format(fmt);
	preview_start[6] = status;

	p1_send_ircmd_ctrl(handle, hb_start);
	p1_send_ircmd_ctrl(handle, preview_start);
	p1_send_ircmd_ctrl(handle, stream_continue);

	p1_send_ircmd(handle, hb_start);
	p1_send_ircmd(handle, preview_start);
	p1_send_ircmd(handle, stream_continue);
}

static void p1_send_range_mode(P1Camera *cam, int mode) {
	if (!cam || !cam->handle) {
		return;
	}
	unsigned char range_low[16] = {0x01,0x2f,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char range_high[16] = {0x01,0x2f,0x41,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if (mode == 0) {
		p1_send_ctrl_header(cam->handle, range_low);
	} else if (mode == 1) {
		p1_send_ctrl_header(cam->handle, range_high);
	}
}

static int p1_claim_interfaces(libusb_device_handle *handle) {
	for (int iface = 0; iface <= 1; iface++) {
		if (libusb_kernel_driver_active(handle, iface) == 1) {
			libusb_detach_kernel_driver(handle, iface);
		}
	}
	if (libusb_set_configuration(handle, 1) < 0) {
		return 0;
	}
	if (libusb_claim_interface(handle, P1_IFACE_CMD) < 0) {
		return 0;
	}
	if (libusb_claim_interface(handle, P1_IFACE_STREAM) < 0) {
		return 0;
	}
	libusb_set_interface_alt_setting(handle, P1_IFACE_STREAM, P1_ALT_STREAM);
	return 1;
}

P1Camera *p1_open(int quiet) {
	P1Camera *cam = (P1Camera *)calloc(1, sizeof(P1Camera));
	if (!cam) {
		return NULL;
	}
	cam->quiet = quiet;
	if (libusb_init(&cam->ctx) != 0) {
		free(cam);
		return NULL;
	}
	cam->handle = libusb_open_device_with_vid_pid(cam->ctx, P1_VENDOR_ID, P1_PRODUCT_ID);
	if (!cam->handle) {
		libusb_exit(cam->ctx);
		free(cam);
		return NULL;
	}
	if (!p1_claim_interfaces(cam->handle)) {
		libusb_close(cam->handle);
		libusb_exit(cam->ctx);
		free(cam);
		return NULL;
	}
	// Match the Python "working" mode: usbmon init only, no start/heartbeat.
	cam->use_stream_cmds = 0;
	p1_usbmon_init(cam->handle);
	cam->buf.clear();
	cam->buf.reserve(P1_FRAME_SIZE * 2);
	cam->chunk.clear();
	cam->chunk.resize(P1_BULK_CHUNK);
	cam->last_heartbeat_ms = p1_time_ms();
	cam->synced = 0;
	cam->discard_next = 0;
	cam->last_range_ms = 0;
	cam->range_interval_ms = 2000;
	cam->range_mode = 0;
	cam->raw_fp = NULL;
	cam->raw_limit = 0;
	cam->raw_written = 0;
	cam->sizes_fp = NULL;
	cam->sizes_limit = 0;
	cam->sizes_written = 0;
	memset(&cam->stats, 0, sizeof(cam->stats));
	return cam;
}

void p1_close(P1Camera *cam) {
	if (!cam) {
		return;
	}
	if (cam->handle) {
		libusb_release_interface(cam->handle, P1_IFACE_CMD);
		libusb_release_interface(cam->handle, P1_IFACE_STREAM);
		libusb_close(cam->handle);
		cam->handle = NULL;
	}
	if (cam->ctx) {
		libusb_exit(cam->ctx);
		cam->ctx = NULL;
	}
	cam->buf.clear();
}

void p1_free(P1Camera *cam) {
	if (!cam) {
		return;
	}
	p1_close(cam);
	free(cam);
}

int p1_read_frame(P1Camera *cam, unsigned char *out, size_t out_len, int timeout_ms, size_t *bytes_read) {
	if (!cam || !cam->handle || !out || out_len == 0) {
		return -1;
	}

	unsigned long now_ms = p1_time_ms();
	if (cam->use_stream_cmds && (now_ms - cam->last_heartbeat_ms >= 1000)) {
		unsigned char hb_send[16] = {0x10,0x10,0x54,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		p1_send_ircmd(cam->handle, hb_send);
		p1_send_ircmd_ctrl(cam->handle, hb_send);
		cam->last_heartbeat_ms = now_ms;
	}
	if (cam->use_stream_cmds && (now_ms - cam->last_range_ms >= 2000)) {
		unsigned char stream_continue[16] = {0x10,0x05,0x41,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		p1_send_ircmd_ctrl(cam->handle, stream_continue);
	}
	if (cam->range_mode >= 0 && cam->range_interval_ms > 0) {
		if (cam->last_range_ms == 0 || (now_ms - cam->last_range_ms) >= cam->range_interval_ms) {
			p1_send_range_mode(cam, cam->range_mode);
			cam->last_range_ms = now_ms;
		}
	}

	unsigned long start_ms = p1_time_ms();
	size_t bytes_total = 0;

	while (1) {
		int transferred = 0;
		int rc = libusb_bulk_transfer(
			cam->handle,
			P1_EP_STREAM_IN,
			cam->chunk.data(),
			(int)cam->chunk.size(),
			&transferred,
			timeout_ms
		);
		if (rc == LIBUSB_ERROR_TIMEOUT) {
			if (bytes_read) {
				*bytes_read = bytes_total;
			}
			if (p1_time_ms() - start_ms >= (unsigned long)timeout_ms) {
				return 0;
			}
			continue;
		}
		if (rc < 0) {
			if (bytes_read) {
				*bytes_read = bytes_total;
			}
			return -1;
		}
		if (transferred <= 0) {
			if (bytes_read) {
				*bytes_read = bytes_total;
			}
			if (p1_time_ms() - start_ms >= (unsigned long)timeout_ms) {
				return 0;
			}
			continue;
		}
		cam->stats.total++;
		cam->stats.last_transfer_size = (unsigned long)transferred;
		if (transferred == (int)P1_BULK_CHUNK) {
			cam->stats.full_packets++;
		} else {
			cam->stats.partial_packets++;
		}
		if (transferred < (int)P1_BULK_CHUNK) {
			cam->stats.short_packets++;
		}
		if (transferred >= 0 && transferred <= 512) {
			cam->stats.counts[transferred]++;
		}
		if (cam->sizes_fp && cam->sizes_written < cam->sizes_limit) {
			fprintf(cam->sizes_fp, "%d\n", transferred);
			cam->sizes_written++;
			if (cam->sizes_written >= cam->sizes_limit) {
				fflush(cam->sizes_fp);
			}
		}
		bytes_total += (size_t)transferred;
		if (cam->raw_fp && cam->raw_written < cam->raw_limit) {
			size_t remaining = cam->raw_limit - cam->raw_written;
			size_t to_write = (remaining < (size_t)transferred) ? remaining : (size_t)transferred;
			if (to_write > 0) {
				fwrite(cam->chunk.data(), 1, to_write, cam->raw_fp);
				cam->raw_written += to_write;
			}
		}
		if (!cam->synced) {
			cam->synced = 1;
		}
		if (transferred <= P1_DROP_SMALL) {
			cam->stats.last_short_size = (unsigned long)transferred;
			cam->stats.last_short_len = (unsigned long)transferred;
			memset(cam->stats.last_short_bytes, 0, sizeof(cam->stats.last_short_bytes));
			if (transferred > 0) {
				size_t copy_len = (transferred > sizeof(cam->stats.last_short_bytes))
					? sizeof(cam->stats.last_short_bytes)
					: (size_t)transferred;
				memcpy(cam->stats.last_short_bytes, cam->chunk.data(), copy_len);
			}
			if (cam->discard_next > 0) {
				cam->discard_next--;
			}
			cam->stats.short_match++;
			cam->stats.drop_small++;
			cam->buf.clear();
			continue;
		}

		cam->buf.insert(cam->buf.end(), cam->chunk.begin(), cam->chunk.begin() + transferred);

		while (cam->buf.size() >= out_len) {
			memcpy(out, cam->buf.data(), out_len);
			cam->buf.erase(cam->buf.begin(), cam->buf.begin() + (ptrdiff_t)out_len);
			cam->stats.good_frames++;
			cam->stats.last_frame_transfers = cam->stats.total;
			if (bytes_read) {
				*bytes_read = bytes_total;
			}
			return 1;
		}

		if (cam->buf.size() > P1_BUF_MAX) {
			size_t drop = cam->buf.size() - P1_BUF_MAX;
			cam->buf.erase(cam->buf.begin(), cam->buf.begin() + (ptrdiff_t)drop);
			cam->stats.bad_frames++;
			cam->stats.drop_events++;
		}
		cam->stats.buffer_bytes = (unsigned long)cam->buf.size();
		if (cam->stats.buffer_bytes > cam->stats.buffer_max) {
			cam->stats.buffer_max = cam->stats.buffer_bytes;
		}
		if (p1_time_ms() - start_ms >= (unsigned long)timeout_ms) {
			if (bytes_read) {
				*bytes_read = bytes_total;
			}
			return 0;
		}
	}
}

void p1_set_range_mode(P1Camera *cam, int mode, unsigned long interval_ms) {
	if (!cam) {
		return;
	}
	cam->range_mode = mode;
	cam->range_interval_ms = interval_ms;
	cam->last_range_ms = 0;
	if (mode >= 0) {
		p1_send_range_mode(cam, mode);
		cam->last_range_ms = p1_time_ms();
	}
}

void p1_set_raw_dump(P1Camera *cam, FILE *fp, size_t max_bytes) {
	if (!cam) {
		return;
	}
	cam->raw_fp = fp;
	cam->raw_limit = max_bytes;
	cam->raw_written = 0;
}

void p1_set_size_log(P1Camera *cam, FILE *fp, size_t max_packets) {
	if (!cam) {
		return;
	}
	cam->sizes_fp = fp;
	cam->sizes_limit = max_packets;
	cam->sizes_written = 0;
}

void p1_get_transfer_stats(P1Camera *cam, P1TransferStats *out) {
	if (!cam || !out) {
		return;
	}
	*out = cam->stats;
}

int p1_device_present(void) {
	libusb_context *ctx = NULL;
	libusb_device_handle *handle = NULL;
	if (libusb_init(&ctx) != 0) {
		return 0;
	}
	handle = libusb_open_device_with_vid_pid(ctx, P1_VENDOR_ID, P1_PRODUCT_ID);
	if (handle) {
		libusb_close(handle);
		libusb_exit(ctx);
		return 1;
	}
	libusb_exit(ctx);
	return 0;
}

#endif
