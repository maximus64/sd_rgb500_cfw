#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <jack/jack.h>
#include <jack/ringbuffer.h>
#include <libusb.h>
#include <signal.h>

#define USB_VID 0x6464
#define USB_PID 0x0001

#define ISOC_EP_NUM 0x01
#define ISOC_PACKET_SIZE 384U

#define RINGBUFFER_SAMPLE_COUNT 1024

#define UNUSED(x)  (void)(x)

static int _stop;
static struct libusb_device_handle *devh = NULL;

static uint8_t isoc_data_packet[ISOC_PACKET_SIZE];

jack_client_t *client;

typedef jack_default_audio_sample_t sample_t;
typedef jack_nframes_t nframes_t;

jack_port_t *in_x;
jack_port_t *in_y;
jack_port_t *in_r;
jack_port_t *in_g;
jack_port_t *in_b;

jack_ringbuffer_t *sample_pkt_rb = NULL;


typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t resv;
} sample_pkg_t;

static inline uint16_t xy_to_u16(sample_t val)
{
    float fval;
    int ret;

    fval = val * 32768.0f + 32767.0f;

    ret = fval;
    if (ret > 65535)
        ret = 65535;
    if (ret < 0)
        ret = 0;

    return (uint16_t)ret;
}

static inline uint8_t rgb_to_u8(sample_t val)
{
    float fval;
    int ret;

    fval = val * 255.0f;

    ret = fval;
    if (ret > 255)
        ret = 255;
    if (ret < 0)
        ret = 0;

    return (uint8_t)ret;
}

static const char* usb_transfer_strerror(enum libusb_transfer_status err)
{
  switch(err)
  {
    case LIBUSB_TRANSFER_COMPLETED: return "LIBUSB_TRANSFER_COMPLETED";
    case LIBUSB_TRANSFER_ERROR: return "LIBUSB_TRANSFER_ERROR";
    case LIBUSB_TRANSFER_TIMED_OUT: return "LIBUSB_TRANSFER_TIMED_OUT";
    case LIBUSB_TRANSFER_CANCELLED: return "LIBUSB_TRANSFER_CANCELLED";
    case LIBUSB_TRANSFER_STALL: return "LIBUSB_TRANSFER_STALL";
    case LIBUSB_TRANSFER_NO_DEVICE: return "LIBUSB_TRANSFER_NO_DEVICE";
    case LIBUSB_TRANSFER_OVERFLOW: return "LIBUSB_TRANSFER_OVERFLOW";
    default: return "<unknown libusb transfer error code>";
  }
}

static void _write_data_callback(struct libusb_transfer *transfer)
{
    if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        printf("usb transfer failed! ep: 0x%02x status: %s\n", 
            transfer->endpoint, usb_transfer_strerror(transfer->status));
    }
}


static int write_isoc(uint8_t ep, unsigned char *buf, int len)
{
    int rc;
    unsigned char *alloc_buf;
    struct libusb_transfer *transfer;

    alloc_buf = malloc(len);
    if (!alloc_buf)
    {
        printf("Fail to alloc data buffer.\n");
        return 1;
    }
    memcpy(alloc_buf, buf, len);

    transfer = libusb_alloc_transfer(len/ISOC_PACKET_SIZE);
    if (!transfer)
    {
        printf("Could not allocate memory for iso transfer.\n");
        free(alloc_buf);
        return 1;
    }

    transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;
    libusb_fill_iso_transfer(transfer, devh, ep,
                             alloc_buf, len, len/ISOC_PACKET_SIZE, _write_data_callback, NULL, 0);
    libusb_set_iso_packet_lengths(transfer, ISOC_PACKET_SIZE);

    rc = libusb_submit_transfer(transfer);

    if(rc != 0)
    {
        printf("Could not submit transfer: rc=%d\n", rc);
        libusb_free_transfer(transfer);
        return 1;
    }

    return 0;
}

static int process (nframes_t nframes, void *arg)
{
    UNUSED(arg);

    nframes_t n;
    sample_pkg_t pkt;

    sample_t *i_x = (sample_t *) jack_port_get_buffer (in_x, nframes);
    sample_t *i_y = (sample_t *) jack_port_get_buffer (in_y, nframes);
    sample_t *i_r = (sample_t *) jack_port_get_buffer (in_r, nframes);
    sample_t *i_g = (sample_t *) jack_port_get_buffer (in_g, nframes);
    sample_t *i_b = (sample_t *) jack_port_get_buffer (in_b, nframes);

    for (n = 0; n < nframes; n++) {
        pkt.x = xy_to_u16(i_x[n]);
        pkt.y = xy_to_u16(i_y[n]);

        pkt.r = rgb_to_u8(i_r[n]);
        pkt.g = rgb_to_u8(i_g[n]);
        pkt.b = rgb_to_u8(i_b[n]);

        pkt.resv = 0;
        jack_ringbuffer_write(sample_pkt_rb, (const char*)&pkt, sizeof(pkt));
    }

    while (jack_ringbuffer_read_space(sample_pkt_rb) >= ISOC_PACKET_SIZE) {
        // read from the buffer
        size_t j = jack_ringbuffer_read(sample_pkt_rb, (char *)isoc_data_packet, ISOC_PACKET_SIZE);

        if (j != ISOC_PACKET_SIZE) {
            printf("error: read ring failed ret: %zu\n", j);
            break;
        }

        write_isoc(ISOC_EP_NUM, isoc_data_packet, ISOC_PACKET_SIZE);
    }

    return 0;
}

static int bufsize (nframes_t nframes, void *arg)
{
    UNUSED(arg);

    printf ("the maximum buffer size is now %u\n", nframes);
    if (nframes > RINGBUFFER_SAMPLE_COUNT) {
        printf("error: buffer size is too large\n");
        return 1;
    }
    return 0;
}

static int srate (nframes_t nframes, void *arg)
{
    UNUSED(arg);

    if(nframes != 48000) {
        printf("error: the sample rate is not 48Khz\n");
        return 1;
    }
    printf ("Sample rate: %u/sec\n", nframes);
    return 0;
}

static void jack_shutdown (void *arg)
{
    UNUSED(arg);
    _stop = 1;
}

static void sig_handler(int sig)
{
    switch (sig)
    {
    case SIGINT:
        printf("SIGINT: stopping...\n");
        _stop = 1;
        break;
    default:
        printf("warn: unhandle signal %d\n", sig);
        break;
    }
}

int main (int argc, char *argv[])
{
    int rc;
    struct sigaction sigact;
    jack_status_t jack_status;
    struct timeval ev_timeout_tv = {.tv_sec = 1, .tv_usec = 0};

    UNUSED(argc);
    UNUSED(argv);

    sigact.sa_handler = sig_handler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);

    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
        return 1;
    }

    devh = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID);
    if (!devh) {
        fprintf(stderr, "Error finding USB device\n");
        goto out;
    }

    rc = libusb_claim_interface(devh, 0);
    if (rc < 0) {
        fprintf(stderr, "Error claiming interface: %s\n", libusb_error_name(rc));
        goto out;
    }
    printf("Claimed interface\n");


    if ((client = jack_client_open("usb_bridge", JackNullOption, &jack_status)) == 0) {
        fprintf (stderr, "jack server not running?\n");
        rc = 1;
        goto out;
    }

    jack_set_process_callback (client, process, 0);
    jack_set_buffer_size_callback (client, bufsize, 0);
    jack_set_sample_rate_callback (client, srate, 0);
    jack_on_shutdown (client, jack_shutdown, 0);

    in_x = jack_port_register (client, "in_x", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    in_y = jack_port_register (client, "in_y", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    in_r = jack_port_register (client, "in_r", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    in_g = jack_port_register (client, "in_g", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    in_b = jack_port_register (client, "in_b", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);

    sample_pkt_rb = jack_ringbuffer_create(ISOC_PACKET_SIZE * RINGBUFFER_SAMPLE_COUNT);
    if (sample_pkt_rb == NULL)
    {
        printf("error: Could not allocate sample packets ringbuffer\n");
        rc = 1;
        goto out;
    }

    // lock the buffer into memory, this is *NOT* realtime safe, do it before
    // using the buffer!
    rc = jack_ringbuffer_mlock(sample_pkt_rb);
    if (rc)
    {
        printf("error: Could not lock JACK ringbuffer memory\n");
        goto out;
    }

    if (jack_activate (client)) {
        fprintf (stderr, "cannot activate client");
        rc = 1;
        goto out;
    }

    while (!_stop) {
        libusb_handle_events_timeout(0, &ev_timeout_tv);
    }

    printf("Closing jack client\n");
    jack_client_close (client);

    printf("Releasing USB interface\n");
    libusb_release_interface(devh, 0);
out:
    if (devh)
        libusb_close(devh);
    libusb_exit(NULL);
    return rc;
}

