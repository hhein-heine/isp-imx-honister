#ifndef __CAM_MENU_HPP__
#define __CAM_MENU_HPP__


#include <string>
#define OK 0

struct cam_dev_parameters{
    std::string cam_dev_node;
    unsigned int sensor_resolution;
    int pix_format;
    int dev_fd;
    int preview_status;
};

#define CAM_RESOLUTION_1080P 0
#define CAM_RESOLUTION_720P  1
#define CAM_RESOLUTION_VGA   2

enum cam_menu{
    CAM_MENU_EXIT = 0,
    CAM_MENU_CAM_APP = 1,
    CAM_MENU_REG_RW,
    CAM_MENU_I2C_RW,
    CAM_MENU_V4L2,
    CAM_MENU_VIRT_CAM,
    CAM_MENU_CONFIG,
    CAM_MENU_TOOLS,
    CAM_MENU_DISPLAY_IMAGE,
    CAM_MENU_MAX,
    CAM_MENU_SUB_ABORT = 111,
};

enum cam_menu_reg_rw{
    CAM_MENU_REG_RW_READ = 1,
    CAM_MENU_REG_RW_WRITE,
    CAM_MENU_REG_RW_MAX,
};

enum cam_menu_i2c_rw{
    CAM_MENU_I2C_RW_READ = 1,
    CAM_MENU_I2C_RW_WRITE,
    CAM_MENU_I2C_RW_MAX,
};

enum cam_menu_v4l2{
    CAM_MENU_V4L2_OPEN = 1,
    CAM_MENU_V4L2_QUERYCAP,
    CAM_MENU_V4L2_ENUM_FMT,
    CAM_MENU_V4L2_SFMT,
    CAM_MENU_V4L2_REQBUFS,
    //CAM_MENU_V4L2_QUERYBUFS,
    CAM_MENU_V4L2_PROCESS_IMAGE,
    CAM_MENU_V4L2_STREAM_ON,
    CAM_MENU_V4L2_STREAM_OFF,
    CAM_MENU_V4L2_CLOSE,
    CAM_MENU_V4L2_DISPLAY,
    CAM_MENU_V4L2_MAX,
};

enum cam_menu_virt_cam{
    CAM_MENU_VIRT_CAM_OPEN = 1,
    CAM_MENU_VIRT_CAM_SETFORMAT,
    CAM_MENU_VIRT_CAM_DATA_PROCESS,
    CAM_MENU_VIRT_CAM_CONNECT,
    CAM_MENU_VIRT_CAM_START,
    CAM_MENU_VIRT_CAM_IOCTL,
    CAM_MENU_VIRT_CAM_STOP,
    CAM_MENU_VIRT_CAM_DISCONNECT,
    CAM_MENU_VIRT_CAM_CLOSE,
    CAM_MENU_VIRT_CAM_MAX
};

enum cam_menu_config{
    CAM_MENU_CONFIG_VIDEO_NODE = 1,
    CAM_MENU_CONFIG_FB_NODE,
    CAM_MENU_CONFIG_FILE_NAME_PATH,
    CAM_MENU_CONFIG_FILE_NAME_PREFIX,
    CAM_MENU_CONFIG_JSON_PATH,
    CAM_MENU_CONFIG_JSON_PREFIX,
    CAM_MENU_CONFIG_MAX
};

enum cam_menu_tools{
    CAM_MENU_TOOLS_VIDEO_INFO = 1,
    CAM_MENU_TOOLS_FB_INFO,
    CAM_MENU_TOOLS_V4L2_COMPLIANCE,
    CAM_MENU_TOOLS_V4L2_COMPLIANCE_CFG,
    CAM_MENU_TOOLS_MAX
};

#endif  //__CAM_MENU_HPP__