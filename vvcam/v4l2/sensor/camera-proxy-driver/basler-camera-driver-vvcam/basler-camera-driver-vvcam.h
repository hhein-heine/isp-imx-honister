#ifndef BASLER_CAMERA_DRIVER_VVCAM_H
#define BASLER_CAMERA_DRIVER_VVCAM_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __KERNEL__

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#endif /* __KERNEL__ */

#define V4L2_CID_PROXY_BASE			(V4L2_CTRL_CLASS_USER | 0x1000)
#define V4L2_CID_BASLER_INTERFACE_VERSION	(V4L2_CID_PROXY_BASE+1)
#define V4L2_CID_BASLER_ACCESS_REGISTER		(V4L2_CID_PROXY_BASE+2)
#define V4L2_CID_BASLER_DEVICE_INFORMATION	(V4L2_CID_PROXY_BASE+3)
#define V4L2_CID_BASLER_CSI_INFORMATION		(V4L2_CID_PROXY_BASE+4)
#define V4L2_CID_BASLER_CAPTURE_PROPERTIES  (V4L2_CID_PROXY_BASE+5)
#define I2CREAD		(1)
#define I2CWRITE	(2)

#define GENCP_STRING_BUFFER_SIZE	(64)
#define STRING_TERMINATION		(1)
#define BDI_MAGIC			(84513200)

/*
 * Basler interface Version
 */
#define BASLER_INTERFACE_VERSION_MAJOR	((__u16) 1)
#define BASLER_INTERFACE_VERSION_MINOR	((__u16) 1)

/*
  Write register:
         IOCTL VIDIOC_S_EXT_CTRLS  with command == I2CWRITE  + address, size and value

  Read register:
         IOCTL VIDIOC_S_EXT_CTRLS  with command == I2CREAD   + address and size
    then IOCTL VIDIOC_G_EXT_CTRLS  returns value

  For the maximum buffer size to use in a read or write command please consider the values
  "Maximum Write Transfer Length"  and "Maximum Read Transfer Length" as described in the MCM Interface Specification.
  These values can differ on the different sensor modules.
*/
struct register_access {
	__u16 address;		/* Register address; host endianness*/
	__u8 data[256];		/* Read/Write register value - target endianness  */
	__u16 data_size;	/* Host endianness  */
	__u8 command;		/* On a VIDIOC_S_EXT_CTRLS identifies to store the register address */
};

struct basler_device_information {
	__u32 _magic;
	__u32 gencpVersion;
	__u8 manufacturerName[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 modelName[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 familyName[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 deviceVersion[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 manufacturerInfo[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 serialNumber[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
	__u8 userDefinedName[GENCP_STRING_BUFFER_SIZE + STRING_TERMINATION];
};

/**
 * struct basler_csi_information - sensor specific csi2 bus configuration.
 * The cid to query this structure is V4L2_CID_BASLER_CSI_INFORMATION.
 * The ioctl to query this structure is BASLER_IOC_G_CSI_INFORMATION
 *
 * @max_lanefrequency  Max theoretical CSI frequency per lane in Hertz.
 * @lanecount          Available CSI lane count.
 * @laneassignment     describes the physical CSI-2 connection between host and camera module.
 *                     The index starts with 0 as CSI lane 1 of the host.
 *                     The value starts with 1 as the CSI lane 1 of the sensor.
 */
struct basler_csi_information {
	__u64 max_lane_frequency;
	__u8 lane_count;
	__u8 lane_assignment[4];
};

/**
* struct basler_capture_properties - sensor specific capture path properties
* The cid to query this structure is V4L2_CID_BASLER_CAPTURE_PROPERTIES
* The ioctl to query this structure is BASLER_IOC_G_CAPTURE_PROPERTIES
*
 * @max_lane_frequency  Max supported CSI frequency per lane in Hertz.
* @max_pixel_frequency  Max supported Pixel frequency for the video capture.
* @max_data_rate        Max supported data rate in bytes/second
*/
struct basler_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

enum {
	BASLER_IOC_G_INTERFACE_VERSION = 0x100,
	BASLER_IOC_READ_REGISTER,
	BASLER_IOC_WRITE_REGISTER,
	BASLER_IOC_G_DEVICE_INFORMATION,
	BASLER_IOC_G_CSI_INFORMATION,
	BASLER_IOC_G_CAPTURE_PROPERTIES

};


#ifdef ISP8000NANO_V1802
#define CONFIG_BASLER_CAMERA_VVCAM
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BASLER_CAMERA_DRIVER_VVCAM_H */
