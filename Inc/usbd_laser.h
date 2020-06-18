#ifndef __USB_LASER_H
#define __USB_LASER_H

/* 8 bytes per sample. 48 sample / ms */
#define LASER_ISOC_EP_LEN                      (8U * 48U)
#define LASER_RING_BUF_SIZE                           32U

typedef struct {
  uint16_t x;
  uint16_t y;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t resv;
} sample_pkg_t;

#define NUM_SAMPLES_PER_RING (LASER_ISOC_EP_LEN / sizeof(sample_pkg_t))

extern USBD_ClassTypeDef  USBD_LASER;

typedef struct
{
  uint8_t                   alt_setting;
  uint8_t                   buffer[LASER_RING_BUF_SIZE][LASER_ISOC_EP_LEN];
  uint16_t                  rd_ptr;
  uint16_t                  wr_ptr;
  uint8_t                   rd_enable;
  uint16_t                  overflow_cnt;
  uint16_t                  underflow_cnt;
}
USBD_LASER_HandleTypeDef;

#endif /* __USB_LASER_H */