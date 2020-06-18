/*
 * Simple test program that draw a circle and change RGB every second
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
#include <libusb.h>
#include <assert.h>

#define USB_VID 0x6464
#define USB_PID 0x0001

#define NSEC_PER_SEC 1000000000
#define PACKET_SIZE 384U

typedef struct {
  uint16_t x;
  uint16_t y;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t resv;
} sample_pkg_t;

static struct libusb_device_handle *devh = NULL;

static uint16_t get_sample_x(unsigned pos)
{
    float fval;
    int ret;

    fval = sinf(pos * 2.0f * M_PI * (100.0f / 48000.0f));
    fval = fval * 32768.0f + 32767.0f;

    ret = fval;
    if (ret >= 65536)
        ret = 65535;
    if (ret < 0)
        ret = 0;

    return (uint16_t)ret;
}

static uint16_t get_sample_y(unsigned pos)
{
    float fval;
    int ret;

    fval = cosf(pos * 2.0f * M_PI * (100.0f / 48000.0f));
    fval = fval * 32768.0f + 32767.0f;

    ret = fval;
    if (ret >= 65536)
        ret = 65535;
    if (ret < 0)
        ret = 0;

    return (uint16_t)ret;
}

/*
Internal callback for cleaning up async writes.
 */
static void WriteAsyncCallback(struct libusb_transfer *transfer)
{
    if (transfer && (transfer->status != LIBUSB_TRANSFER_COMPLETED/* || transfer->actual_length != transfer->length*/))
    {
        printf("ISO transfer err: %d   bytes transferred: %d\n", transfer->status, transfer->actual_length);
    }
    libusb_free_transfer(transfer);
}


static int write_isoc(uint8_t ep, unsigned char *buf, int len)
{
    int rc;

    struct libusb_transfer *transfer = libusb_alloc_transfer(len/PACKET_SIZE);

    if (!transfer)
    {
        printf("Could not allocate memory for iso transfer.\n");
        return 1;
    }

    libusb_fill_iso_transfer(transfer, devh, ep,
                             buf, len, len/PACKET_SIZE, WriteAsyncCallback, 0, 0);
    libusb_set_iso_packet_lengths(transfer, PACKET_SIZE);

    rc = libusb_submit_transfer(transfer);

    if(rc != 0)
    {
        printf("Could not submit transfer: rc=%d\n", rc);
        return 1;
    }

    return 0;
}

static int do_exit = 0;

static void sig_hdlr(int signum)
{
    switch (signum)
    {
    case SIGINT:
        printf("\nGot request to quit\n");
        do_exit = 1;
        break;
    case SIGUSR1:
        printf("sigusr1 caught\n");
        do_exit = 1;
        break;
    default:
        printf("what\n");
    }
}

static void *libusbThread(void *arg)
{
    struct timeval timeout_tv;
    (void) arg;

    timeout_tv.tv_sec = 1;
    timeout_tv.tv_usec = 0;

    printf("USB Event Thread Running\n");
    do
    {
        libusb_handle_events_timeout(0, &timeout_tv);
    }
    while (!do_exit);

    return NULL;
}

static inline uint64_t get_ns_tick_uint64(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}

int main (int argc, const char *argv[])
{
    int rc;
    struct sigaction sigact;
    unsigned int count;
    uint64_t tstart, tdelta;
    pthread_t ev_thread;

    (void) argc;
    (void) argv;

    sigact.sa_handler = sig_hdlr;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGUSR1, &sigact, NULL);

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

    printf("Starting USB Event Thread\n");
    pthread_create(&ev_thread, NULL, libusbThread, NULL);

    tstart = get_ns_tick_uint64();
    count = 0;

    while (!do_exit)
    {
        tdelta = get_ns_tick_uint64() - tstart;

        if (tdelta >= 1000000) {
            sample_pkg_t samples[48];

            for (unsigned i = 0; i < 48; i++) {
                samples[i].x = get_sample_x(count % 48000);
                samples[i].y = get_sample_y(count % 48000);
                samples[i].r = 0;
                samples[i].g = 0;
                samples[i].b = 0;
                samples[i].resv = 0x00;

                switch((tstart / NSEC_PER_SEC) % 3)
                {
                case 0:
                  samples[i].r = 0xff;
                  break;
                case 1:
                  samples[i].b = 0xff;
                  break;
                case 2:
                  samples[i].g = 0xff;
                  break;
                }

                count++;
            }

            write_isoc(1, (uint8_t*) samples, sizeof(samples));
            tstart += 1000000;
        }
    }
    
    pthread_join(ev_thread, NULL);

    printf("Quitting gracefully\n");
    libusb_release_interface(devh, 0);
out:
    if (devh)
        libusb_close(devh);
    libusb_exit(NULL);
    return rc;
}