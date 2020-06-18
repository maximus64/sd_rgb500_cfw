#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/cdefs.h>
#include "usbd_ioreq.h"
#include "usbd_laser.h"
#include "utils.h"

#define LASER_ISOC_OUT_EP                            0x01


/* USB LASER device Configuration Descriptor */
#define WBVAL(x) LOBYTE(x),HIBYTE(x)
#define USB_LASER_CONFIG_DESC_SIZE (USB_LEN_CFG_DESC + USB_LEN_IF_DESC + USB_LEN_EP_DESC )
__ALIGN_BEGIN static uint8_t USBD_LASER_CfgDesc[USB_LASER_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* Configuration 1 */
  USB_LEN_CFG_DESC,                   /* bLength */
  USB_DESC_TYPE_CONFIGURATION,        /* bDescriptorType */
  WBVAL(USB_LASER_CONFIG_DESC_SIZE), /* wTotalLength */
  0x01,                               /* bNumInterfaces */
  0x01,                               /* bConfigurationValue: 0x01 is used to select this configuration */
  0x00,                               /* iConfiguration: no string to describe this configuration */
  0xC0,                               /* bmAttributes: self-powered*/
  0x00,                               /* bMaxPower, self-powered does not power from bus*/

  /* Interface 0, Alternate Setting 0 interface descriptor */
  USB_LEN_IF_DESC,                   /* bLength */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType */
  0,			                     /* bInterfaceNumber: Number of Interface */
  0x00,                              /* bAlternateSetting: Alternate setting */
  0x01,                              /* bNumEndpoints: Two endpoints used */
  0xFF, 							 /* bInterfaceClass: Vendor specific */
  0xFF,						         /* bInterfaceSubClass: Vendor specific */
  0x00,                              /* bInterfaceProtocol: no protocol used */
  0x00,                              /* iInterface: */

  /* Endpoint, EP1 Isochronous Out */
  USB_LEN_EP_DESC,                   /* bLength */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
  0x01,                              /* bEndpointAddress */
  USBD_EP_TYPE_ISOC,                 /* bmAttributes */
  WBVAL(LASER_ISOC_EP_LEN),          /* wMaxPacketSize */
  0x01,                              /* bInterval */
};


/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_LASER_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, /* bcdUSB [2 bytes]: USB Specification Release Number in Binary-Coded Decimal*/
  0x02,
  0xff, /* bDeviceClass: 0xFF vendor specific */
  0x00, /* bDeviceSubClass: Subclass code */
  0xff, /* bDeviceProtocol: Protocol code 0xFF vendor specific */
  0x40, /* bMaxPacketSize0: Maximum packet size for other speed. */
  0x01, /* bNumConfigurations: Number of other-speed configurations. */
  0x00, /* bReserved: Reserved for future use, must be zero. */
};


USBD_LASER_HandleTypeDef laserd_handle;

/**
  * @brief  USBD_LASER_Init
  *         Initialize the LASER interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_LASER_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_LASER_HandleTypeDef   *hlaser;
  printf("MAXIMUS64 USBD_LASER_Init() cfgidx=0x%02x\r\n", cfgidx);

  /* Open ISOC EP OUT */
  USBD_LL_OpenEP(pdev, LASER_ISOC_OUT_EP, USBD_EP_TYPE_ISOC, LASER_ISOC_EP_LEN);
  pdev->ep_out[LASER_ISOC_OUT_EP & 0xFU].is_used = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = &laserd_handle;

  hlaser = (USBD_LASER_HandleTypeDef *) pdev->pClassData;

  hlaser->overflow_cnt = 0;
  hlaser->underflow_cnt = 0;
  /* Clear ring buffer */
  hlaser->wr_ptr = 0U;
  hlaser->rd_ptr = 0U;
  hlaser->rd_enable = 0U;

  memset(hlaser->buffer, 0x00, sizeof(hlaser->buffer));

  /* Prepare Out endpoint to receive 1st packet */
  USBD_LL_PrepareReceive(pdev, LASER_ISOC_OUT_EP, hlaser->buffer[hlaser->wr_ptr],
                          LASER_ISOC_EP_LEN);

  return USBD_OK;
}

/**
  * @brief  USBD_LASER_Init
  *         DeInitialize the LASER layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_LASER_DeInit(USBD_HandleTypeDef *pdev,
                                  uint8_t cfgidx __unused)
{
  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, LASER_ISOC_OUT_EP);
  pdev->ep_out[LASER_ISOC_OUT_EP & 0xFU].is_used = 0U;

  return USBD_OK;
}

/**
  * @brief  USBD_LASER_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_LASER_Setup(USBD_HandleTypeDef *pdev,
                                 USBD_SetupReqTypedef *req)
{
  USBD_LASER_HandleTypeDef *hlaser;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  hlaser = (USBD_LASER_HandleTypeDef *)pdev->pClassData;

  printf("MAXIMUS64 inside USBD_LASER_Setup()\r\n");

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;
/*
        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

            USBD_CtlSendData(pdev, pbuf, len);
          }
          break;
*/
        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, &hlaser->alt_setting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
            {
              hlaser->alt_setting = (uint8_t)(req->wValue);
            }
            else
            {
              /* Call the error management function (command will be nacked */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  USBD_LASER_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_LASER_GetCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_LASER_CfgDesc);

  return USBD_LASER_CfgDesc;
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_LASER_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_LASER_DeviceQualifierDesc);

  return USBD_LASER_DeviceQualifierDesc;
}

/**
  * @brief  USBD_LASER_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_LASER_EP0_RxReady(USBD_HandleTypeDef *pdev __unused)
{
  return USBD_OK;
}
/**
  * @brief  USBD_LASER_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_LASER_EP0_TxReady(USBD_HandleTypeDef *pdev __unused)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
  * @brief  USBD_LASER_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_LASER_SOF(USBD_HandleTypeDef *pdev __unused)
{
  return USBD_OK;
}


/**
  * @brief  USBD_LASER_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_LASER_IsoINIncomplete(USBD_HandleTypeDef *pdev __unused, uint8_t epnum __unused)
{
  printf("USBD_LASER_IsoINIncomplete() epnum=%02x\r\n",epnum);
  return USBD_OK;
}
/**
  * @brief  USBD_LASER_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_LASER_IsoOutIncomplete(USBD_HandleTypeDef *pdev __unused, uint8_t epnum __unused)
{
  printf("USBD_LASER_IsoOutIncomplete() epnum=%02x\r\n",epnum);
  return USBD_OK;
}
/**
  * @brief  USBD_LASER_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_LASER_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  // uint32_t recv_sz;
  uint16_t next_wr_ptr;
  USBD_LASER_HandleTypeDef   *hlaser;
  hlaser = (USBD_LASER_HandleTypeDef *) pdev->pClassData;

  switch(epnum) {
  case LASER_ISOC_OUT_EP:
    // recv_sz = USBD_LL_GetRxDataSize(pdev, epnum);
    next_wr_ptr = (hlaser->wr_ptr + 1) % LASER_RING_BUF_SIZE;

    //check if ring is full
    if (next_wr_ptr == hlaser->rd_ptr)
    {
      //ring is full so let just drop current packet
      //report packet drop
      hlaser->overflow_cnt += 1;
    }
    else
    {
      //increment write pointer
      hlaser->wr_ptr = next_wr_ptr;
    }

    //enable output when buffer is half full
    uint16_t ring_cap =  (hlaser->wr_ptr >= hlaser->rd_ptr)
                ? (uint16_t) (hlaser->wr_ptr - hlaser->rd_ptr)
                : LASER_RING_BUF_SIZE - hlaser->rd_ptr + hlaser->wr_ptr;
    if (ring_cap > LASER_RING_BUF_SIZE / 2)
    {
      hlaser->rd_enable = 1;
    }

    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive(pdev, LASER_ISOC_OUT_EP, hlaser->buffer[hlaser->wr_ptr],
                          LASER_ISOC_EP_LEN);
    break;
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LASER_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_LASER_DataIn(USBD_HandleTypeDef *pdev __unused, uint8_t epnum)
{
  printf("USBD_LASER_DataIn() called. EP=%02x\r\n", epnum);
  /* Only OUT data are processed */
  return USBD_OK;
}

USBD_ClassTypeDef  USBD_LASER =
{
  USBD_LASER_Init,
  USBD_LASER_DeInit,
  USBD_LASER_Setup,
  USBD_LASER_EP0_TxReady,
  USBD_LASER_EP0_RxReady,
  USBD_LASER_DataIn,
  USBD_LASER_DataOut,
  USBD_LASER_SOF,
  USBD_LASER_IsoINIncomplete,
  USBD_LASER_IsoOutIncomplete,
  USBD_LASER_GetCfgDesc,
  USBD_LASER_GetCfgDesc,
  USBD_LASER_GetCfgDesc,
  USBD_LASER_GetDeviceQualifierDesc,
};