/*-
 * Copyright (c) 2022 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * This file implements support for the DMX512 port via USB on Martin
 * Lightning products.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <usb.h>
#include <pthread.h>
#include <alsa/asoundlib.h>

#ifdef HAVE_PICTURE
#include "picture.h"
#endif

#define	USB_VENDOR 0x11be
#define	USB_PRODUCT 0xf808

#define	USB_RX_ENDPOINT 2
#define	USB_TX_ENDPOINT 4

#define	NOTE_START (5 * 12)
#define	NOTE_END (NOTE_START + 26)

#define	FPS 10
#define	LEDS 8

static usb_dev_handle * usb_devh_rx;
static usb_dev_handle * usb_devh_tx;
static snd_seq_t *alsa_seq;
#ifdef HAVE_PICTURE
static const char *image_data;
static uint32_t image_size;
static uint32_t random_value;
#endif
static float global_decay = 3.0;
static float global_led_gain = 0;
static float global_spot_gain = 0;
static float global_pixel_speed = 0;

static const uint8_t midi_map[26] = {
	/* 1st octave */
	 /* [C0] = */ 0,		/* dead */
	 /* [D0B] = */ 0,		/* dead */
	 /* [D0] = */ 1 + 0 + 0x80,
	 /* [E0B] = */ 1 + 2 + 0x80,
	 /* [E0] = */ 1 + 1 + 0x80,

	 /* [F0] = */ 1 + 0,
	 /* [G0B] = */ 1 + 4,
	 /* [G0] = */ 1 + 1,
	 /* [A0B] = */ 1 + 5,
	 /* [A0] = */ 1 + 2,
	 /* [H0B] = */ 1 + 6,
	 /* [H0] = */ 1 + 3,

	/* 2nd octave */
	 /* [C1] = */ 0,		/* dead */
	 /* [D1B] = */ 1 + 7,
	 /* [D1] = */ 1 + 0 + 0x80,
	 /* [E1B] = */ 1 + 2 + 0x80,
	 /* [E1] = */ 1 + 1 + 0x80,

	 /* [F1] = */ 1 + 0,
	 /* [G1B] = */ 1 + 4,
	 /* [G1] = */ 1 + 1,
	 /* [A1B] = */ 1 + 5,
	 /* [A1] = */ 1 + 2,
	 /* [H1B] = */ 1 + 6,
	 /* [H1] = */ 1 + 3,

	/* 3rd octave */
	 /* [C2] = */ 0,		/* dead */
	 /* [D2B] = */ 1 + 7,
};

#define	SPOT_START 0
#define	SPOT_END 20

static struct {
	uint16_t offset_i;
	uint16_t offset_r;
	uint16_t offset_g;
	uint16_t offset_b;

	float	value_i;
	float	value_r;
	float	value_g;
	float	value_b;
}	led_map[LEDS] = {
#define	LED_MAP(offset) { \
    .offset_i = (offset) + 7, \
    .offset_r = (offset) + 0, \
    .offset_g = (offset) + 1, \
    .offset_b = (offset) + 2 \
}

	LED_MAP(99),
	LED_MAP(108),
	LED_MAP(117),
	LED_MAP(126),

	LED_MAP(135),
	LED_MAP(144),
	LED_MAP(153),
	LED_MAP(162),
};

static void
trigger(uint8_t which, uint8_t velocity)
{
	if (which >= LEDS)
		return;
	velocity = 127 - velocity;

	led_map[which].value_i = (led_map[which].value_i * velocity) / 127.0;
	led_map[which].value_r = 1.0;
	led_map[which].value_g = 0.0;
	led_map[which].value_b = 0.0;
}

static void
store(float value, uint8_t *ptr, float max)
{
	if (value > 1.0f)
		value = 1.0f;
	else if (value < 0.0f)
		value = 0.0f;
	*ptr = (int)(max * value);
}

static void
store_led(uint8_t which, uint8_t *ptr)
{
	store(led_map[which].value_i * global_led_gain, ptr + led_map[which].offset_i, 255);
	store(led_map[which].value_r, ptr + led_map[which].offset_r, 255);
	store(led_map[which].value_g, ptr + led_map[which].offset_g, 255);
	store(led_map[which].value_b, ptr + led_map[which].offset_b, 255);
}

static void
update(uint8_t which, float i, float r, float g, float b)
{
	led_map[which].value_i += (i - led_map[which].value_i) / global_decay;
	led_map[which].value_r += (r - led_map[which].value_r) / global_decay;
	led_map[which].value_g += (g - led_map[which].value_g) / global_decay;
	led_map[which].value_b += (b - led_map[which].value_b) / global_decay;
}

static void
update_pixel_speed()
{
#ifdef HAVE_PICTURE
	unsigned w_rand = (arc4random() % width) * global_pixel_speed;
	unsigned h_rand = (arc4random() % height) * global_pixel_speed;
	unsigned value = (w_rand + h_rand * width) % (image_size / 4);

	random_value = 4 * value;
#endif
}

static void *
usb_read_loop(void *arg)
{
	uint8_t buffer[1024];
	uint8_t timeout = 3;

	while (1) {
		if (usb_bulk_read(usb_devh_rx, USB_RX_ENDPOINT, (char *)buffer, sizeof(buffer), 0) < 0) {
			if (timeout-- == 0)
				break;
		} else {
			timeout = 3;
		}
	}
	printf("USB READ FAILED\n");
	return (NULL);
}

static void
convert(const uint8_t *from, uint8_t *to)
{
	uint16_t c;

	for (c = 0; c != 512;) {
		*to++ = c & 0xFF;
		*to++ = c >> 8;

		for (uint8_t x = 0; x != 62 && c != 512; x++, c++)
			*to++ = *from++;
	}
}

static void *
usb_write_loop(void *arg)
{
	uint8_t buffer[512] = {};
	uint8_t martin[530] = {};
	uint8_t pixels[LEDS][3] = {};
	uint32_t counter = 0;
	uint8_t timeout = 3;

	while (1) {
#ifdef HAVE_PICTURE
		const char *image_ptr;
#endif
		convert(buffer, martin);

		if (usb_bulk_write(usb_devh_tx, USB_TX_ENDPOINT, (char *)martin, sizeof(martin), 0) < 0) {
			if (timeout-- == 0)
				break;
		} else {
			timeout = 3;
		}

		usleep(1000000 / FPS);

#ifdef HAVE_PICTURE
		image_data += random_value;
		while ((image_data - header_data) >= image_size)
			image_data -= image_size;
#endif
		for (unsigned x = SPOT_START; x != SPOT_END; x++)
			store(global_spot_gain, buffer + x, 255);

#ifdef HAVE_PICTURE
		image_ptr = image_data;

		for (unsigned x = 0; x != LEDS; x++) {
			while ((image_ptr - header_data) >= image_size)
				image_ptr -= image_size;
			HEADER_PIXEL(image_ptr, pixels[x]);

			update(x, (pixels[x][0] + pixels[x][1] + pixels[x][2]) / (255.0f * 3.0f),
			    pixels[x][0] / 255.0f, pixels[x][1] / 255.0f, pixels[x][2] / 255.0f);
			store_led(x, buffer);
		}
#endif
		if (++counter == 30 * FPS) {
			counter = 0;
			update_pixel_speed();
		}
	}
	printf("USB WRITE FAILED\n");
	return (NULL);
}

static void
usb_martin_setup()
{
#include "martin.h"

	struct setup_request *req = s_setupRequest;
	struct setup_request *end = s_setupRequest + (sizeof(s_setupRequest) / sizeof(*req));

	for (; req != end; req++) {
		char buffer[req->cbData];

		if (usb_control_msg(usb_devh_rx,
		    req->bmRequestType,
		    req->bRequest,
		    req->wValue,
		    req->wIndex,
		    (req->bmRequestType & 0x80) ? buffer : req->pData,
		    req->cbData, 1000) < 0) {
			printf("USB control request failed\n");
			break;
		}
	}
}

static void *
alsa_read_loop(void *arg)
{
	snd_seq_event_t *ev;

	while (snd_seq_event_input(alsa_seq, &ev) >= 0) {
		switch (ev->type) {
		case SND_SEQ_EVENT_NOTEON:
			if (ev->data.note.channel != 0 ||
			    ev->data.note.note < NOTE_START ||
			    ev->data.note.note >= NOTE_END ||
			    midi_map[ev->data.note.note - NOTE_START] == 0)
				break;
			trigger(midi_map[ev->data.note.note - NOTE_START] - 1, ev->data.note.velocity);
			break;
		case SND_SEQ_EVENT_CONTROLLER:
#ifdef HAVE_DEBUG
			printf("CONTROL EVENT %d %d\n", ev->data.control.param,
			    ev->data.control.value);
#endif
			switch (ev->data.control.param) {
			case 114:
				global_led_gain = ev->data.control.value / 127.0;
				break;
			case 117:
				global_spot_gain = ev->data.control.value / 127.0;
				break;
			case 116:
				global_pixel_speed = ev->data.control.value / 127.0;
				update_pixel_speed();
				break;
			case 113:
				global_decay = ev->data.control.value + 1;
				break;
			default:
				break;
			}
			break;

		default:
#ifdef HAVE_DEBUG
			printf("UNKNOWN EVENT %d\n", ev->type);
#endif
			break;
		}
		snd_seq_free_event(ev);
	}
	return (NULL);
}

int
main()
{
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;
	pthread_t thread;
	int err;

#ifdef HAVE_PICTURE
	image_size = width * height * 4;
	image_data = header_data;
#endif
	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();
	for (bus = busses; bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			if (dev->descriptor.idVendor == USB_VENDOR &&
			    dev->descriptor.idProduct == USB_PRODUCT) {
				usb_devh_rx = usb_open(dev);
				if (usb_devh_rx != NULL) {
					if (usb_claim_interface(usb_devh_rx, 0) < 0 ||
					    usb_set_altinterface(usb_devh_rx, 1) < 0)
						goto fail_rx;
					usb_devh_tx = usb_open(dev);
					if (usb_devh_tx != NULL) {
						if (usb_claim_interface(usb_devh_tx, 0) < 0 ||
						    usb_set_altinterface(usb_devh_tx, 1) < 0) {
							usb_close(usb_devh_rx);
							goto fail_rx;
						}
						usb_martin_setup();
						pthread_create(&thread, NULL, &usb_read_loop, NULL);
						pthread_create(&thread, NULL, &usb_write_loop, NULL);
						goto found;
					} else {
				fail_rx:
						usb_close(usb_devh_rx);
					}
				}
			}
		}
	}
	printf("No Martin USB DMX device found\n");
	return (1);
found:
	err = snd_seq_open(&alsa_seq, "default", SND_SEQ_OPEN_INPUT, 0);
	if (err < 0) {
		printf("Failed to open ALSA sequencer\n");
		return (1);
	}
	snd_seq_set_client_name(alsa_seq, "Martin USB DMX");

	snd_seq_create_simple_port(alsa_seq, "port",
	    SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
	    SND_SEQ_PORT_TYPE_MIDI_GENERIC);

	alsa_read_loop(NULL);

	return (0);
}
