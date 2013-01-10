/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
//#include "gc0339.h"
#define SENSOR_NAME "gc0339"
#define PLATFORM_DRIVER_NAME "msm_camera_gc0339"
#define gc0339_obj gc0339_##obj

DEFINE_MUTEX(gc0339_mut);
static struct msm_sensor_ctrl_t gc0339_s_ctrl;



static struct msm_camera_i2c_reg_conf gc0339_prev_settings[] = {
/*	{0xfc,0x10},
	{0xfe,0x00},
	{0xf6,0x07},
	{0xf7,0x01},
	{0xf7,0x03},
	{0xfc,0x16},
	{0x06,0x00},
	{0x08,0x04},
	{0x09,0x01},
	{0x0a,0xe8},
	{0x0b,0x02},
	{0x0c,0x88},
	{0x0f,0x02},
	{0x14,0x20},
	{0x1a,0x21},
	{0x1b,0x80},
	{0x1c,0x49},
	{0x61,0x2a},
	{0x62,0x8c},
	{0x63,0x02},
	{0x32,0x00},
	{0x3a,0x20},
	{0x3b,0x20},
	{0x69,0x03},
	{0x60,0x84},
	{0x65,0x10},
	{0x6c,0xaa},
	{0x6d,0x00},
	{0x67,0x10},
	{0x4a,0x40},
	{0x4b,0x40},
	{0x4c,0x40},
	{0xe8,0x04},
	{0xe9,0xbb},
	{0x42,0x28},
	{0x47,0x10},
	{0x50,0x40},
	{0xd0,0x00},
	{0xd3,0x50},
	{0xf6,0x05},
	{0x01,0x6a},
	{0x02,0x0c},
	{0x0f,0x00},
	{0x6a,0x13},
	{0x71,0x01},
	{0x72,0x02},
	{0x79,0x02},
	{0x73,0x01},
	{0x7a,0x01},
	{0x2e,0x30},
	{0x2b,0x00},
	{0x2c,0x03},
	{0xd2,0x00},
	{0x20,0xb0},
//    {0x1f,0x1f},
	{0x60,0x94},
*/
/**/
{0xfc,0x10},
    {0xfe,0x00},
    {0xf6,0x07},
    {0xf7,0x01},
    {0xf7,0x03},
    {0xfc,0x16},
    {0x06,0x00},
    {0x08,0x04},
    {0x09,0x01},
    {0x0a,0xe8},
    {0x0b,0x02},
    {0x0c,0x88},
    {0x0f,0x02},
    {0x14,0x20},
    {0x1a,0x21},
    {0x1b,0x80},
    {0x1c,0x49},
    {0x61,0x2a},
    {0x62,0x8c},
    {0x63,0x02},
    {0x32,0x00},
    {0x3a,0x20},
    {0x3b,0x20},
    {0x69,0x03},
    {0x60,0x84},
    {0x65,0x10},
    {0x6c,0xaa},
    {0x6d,0x00},
    {0x67,0x10},
    {0x4a,0x40},
    {0x4b,0x40},
    {0x4c,0x40},
    {0xe8,0x04},
    {0xe9,0xbb},
    {0x42,0x28},
    {0x47,0x10},
    {0x50,0x40},
    {0xd0,0x00},
    {0xd3,0x50},
    {0xf6,0x05},
    {0x01,0x6a},
    {0x02,0x0c},
    {0x0f,0x00},
    {0x6a,0x11},
    {0x71,0x01},
    {0x72,0x01},
    {0x79,0x01},
    {0x73,0x01},
    {0x7a,0x01},
    {0x2e,0x30},
    {0x2b,0x00},
    {0x2c,0x03},
    {0xd2,0x00},
    {0x20,0xb0},
    {0x60,0x94},
/**/
};



static struct msm_camera_i2c_reg_conf gc0339_snap_settings[] = {
/*	{0xfc,0x10},
	{0xfe,0x00},
	{0xf6,0x07},
	{0xf7,0x01},
	{0xf7,0x03},
	{0xfc,0x16},
	{0x06,0x00},
	{0x08,0x04},
	{0x09,0x01},
	{0x0a,0xe8},
	{0x0b,0x02},
	{0x0c,0x88},
	{0x0f,0x02},
	{0x14,0x20},
	{0x1a,0x21},
	{0x1b,0x80},
	{0x1c,0x49},
	{0x61,0x2a},
	{0x62,0x8c},
	{0x63,0x02},
	{0x32,0x00},
	{0x3a,0x20},
	{0x3b,0x20},
	{0x69,0x03},
	{0x60,0x84},
	{0x65,0x10},
	{0x6c,0xaa},
	{0x6d,0x00},
	{0x67,0x10},
	{0x4a,0x40},
	{0x4b,0x40},
	{0x4c,0x40},
	{0xe8,0x04},
	{0xe9,0xbb},
	{0x42,0x28},
	{0x47,0x10},
	{0x50,0x40},
	{0xd0,0x00},
	{0xd3,0x50},
	{0xf6,0x05},
	{0x01,0x6a},
	{0x02,0x0c},
	{0x0f,0x00},
	{0x6a,0x13},
	{0x71,0x01},
	{0x72,0x02},
	{0x79,0x02},
	{0x73,0x01},
	{0x7a,0x01},
	{0x2e,0x30},
	{0x2b,0x00},
	{0x2c,0x03},
	{0xd2,0x00},
	{0x20,0xb0},
//    {0x1f,0x1f},
	{0x60,0x94},*/
/**/
    {0xfc,0x10},
    {0xfe,0x00},
    {0xf6,0x07},
    {0xf7,0x01},
    {0xf7,0x03},
    {0xfc,0x16},
    {0x06,0x00},
    {0x08,0x04},
    {0x09,0x01},
    {0x0a,0xe8},
    {0x0b,0x02},
    {0x0c,0x88},
    {0x0f,0x02},
    {0x14,0x20},
    {0x1a,0x21},
    {0x1b,0x80},
    {0x1c,0x49},
    {0x61,0x2a},
    {0x62,0x8c},
    {0x63,0x02},
    {0x32,0x00},
    {0x3a,0x20},
    {0x3b,0x20},
    {0x69,0x03},
    {0x60,0x84},
    {0x65,0x10},
    {0x6c,0xaa},
    {0x6d,0x00},
    {0x67,0x10},
    {0x4a,0x40},
    {0x4b,0x40},
    {0x4c,0x40},
    {0xe8,0x04},
    {0xe9,0xbb},
    {0x42,0x28},
    {0x47,0x10},
    {0x50,0x40},
    {0xd0,0x00},
    {0xd3,0x50},
    {0xf6,0x05},
    {0x01,0x6a},
    {0x02,0x0c},
    {0x0f,0x00},
    {0x6a,0x11},
    {0x71,0x01},
    {0x72,0x01},
    {0x79,0x01},
    {0x73,0x01},
    {0x7a,0x01},
    {0x2e,0x30},
    {0x2b,0x00},
    {0x2c,0x03},
    {0xd2,0x00},
    {0x20,0xb0},
    {0x60,0x94},
/**/
};

static struct msm_camera_i2c_reg_conf gc0339_recommend_settings[] = {
/*	{0xfc,0x10},
	{0xfe,0x00},
	{0xf6,0x07},
	{0xf7,0x01},
	{0xf7,0x03},
	{0xfc,0x16},
	{0x06,0x00},
	{0x08,0x04},
	{0x09,0x01},
	{0x0a,0xe8},
	{0x0b,0x02},
	{0x0c,0x88},
	{0x0f,0x02},
	{0x14,0x20},
	{0x1a,0x21},
	{0x1b,0x80},
	{0x1c,0x49},
	{0x61,0x2a},
	{0x62,0x8c},
	{0x63,0x02},
	{0x32,0x00},
	{0x3a,0x20},
	{0x3b,0x20},
	{0x69,0x03},
	{0x60,0x84},
	{0x65,0x10},
	{0x6c,0xaa},
	{0x6d,0x00},
	{0x67,0x10},
	{0x4a,0x40},
	{0x4b,0x40},
	{0x4c,0x40},
	{0xe8,0x04},
	{0xe9,0xbb},
	{0x42,0x28},
	{0x47,0x10},
	{0x50,0x40},
	{0xd0,0x00},
	{0xd3,0x50},
	{0xf6,0x05},
	{0x01,0x6a},
	{0x02,0x0c},
	{0x0f,0x00},
	{0x6a,0x13},
	{0x71,0x01},
	{0x72,0x02},
	{0x79,0x02},
	{0x73,0x01},
	{0x7a,0x01},
	{0x2e,0x30},
	{0x2b,0x00},
	{0x2c,0x03},
	{0xd2,0x00},
	{0x20,0xb0},
 //   {0x1f,0x1f},
	{0x60,0x94},*/
    /**/
    {0xfc,0x10},
    {0xfe,0x00},
    {0xf6,0x07},
    {0xf7,0x01},
    {0xf7,0x03},
    {0xfc,0x16},
    {0x06,0x00},
    {0x08,0x04},
    {0x09,0x01},
    {0x0a,0xe8},
    {0x0b,0x02},
    {0x0c,0x88},
    {0x0f,0x02},
    {0x14,0x20},
    {0x1a,0x21},
    {0x1b,0x80},
    {0x1c,0x49},
    {0x61,0x2a},
    {0x62,0x8c},
    {0x63,0x02},
    {0x32,0x00},
    {0x3a,0x20},
    {0x3b,0x20},
    {0x69,0x03},
    {0x60,0x84},
    {0x65,0x10},
    {0x6c,0xaa},
    {0x6d,0x00},
    {0x67,0x10},
    {0x4a,0x40},
    {0x4b,0x40},
    {0x4c,0x40},
    {0xe8,0x04},
    {0xe9,0xbb},
    {0x42,0x28},
    {0x47,0x10},
    {0x50,0x40},
    {0xd0,0x00},
    {0xd3,0x50},
    {0xf6,0x05},
    {0x01,0x6a},
    {0x02,0x0c},
    {0x0f,0x00},
    {0x6a,0x11},
    {0x71,0x01},
    {0x72,0x01},
    {0x79,0x01},
    {0x73,0x01},
    {0x7a,0x01},
    {0x2e,0x30},
    {0x2b,0x00},
    {0x2c,0x03},
    {0xd2,0x00},
    {0x20,0xb0},
    {0x60,0x94},
/**/
};

static struct v4l2_subdev_info gc0339_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array gc0339_init_conf[] = {
	{&gc0339_recommend_settings[0],
	ARRAY_SIZE(gc0339_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array gc0339_confs[] = {
	{&gc0339_snap_settings[0],
	ARRAY_SIZE(gc0339_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&gc0339_prev_settings[0],
	ARRAY_SIZE(gc0339_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t gc0339_dimensions[] = {
	{
		.x_output = 0x28c, /* 640 */
		.y_output = 0x1E6, /* 480 */
		.line_length_pclk = 0x320,
		.frame_length_lines = 0x1f4, 
		.vt_pixel_clk = 12000000,
		.op_pixel_clk = 266000000,
		.binning_factor = 1,
	},
};

static struct msm_sensor_output_reg_addr_t gc0339_reg_addr = {
	.x_output = 0x0b,
	.y_output = 0x09,
	.line_length_pclk = 0x0b,
	.frame_length_lines = 0x09,
};

static struct msm_sensor_id_info_t gc0339_id_info = {
  .sensor_id_reg_addr        = 0x00,
  .sensor_id                 = 0xC8,
};

static struct msm_sensor_exp_gain_info_t gc0339_exp_gain_info = {
	.coarse_int_time_addr = 0x51,
	.global_gain_addr = 0x03,
	.vert_offset = 4,
};

static enum msm_camera_vreg_name_t gc0339_veg_seq[] = {
	CAM_VIO,
	CAM_VDIG,
	CAM_VANA,
};

static int32_t gc0339_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
 int rc = 0;
  unsigned int  intg_time_msb, intg_time_lsb;
  CDBG("gc0339_write_prev_exp_gain,gain=%d, line=%d\n",gain,line);
  intg_time_msb = (unsigned int ) ((line & 0x0F00) >> 8);
  intg_time_lsb = (unsigned int ) (line& 0x00FF);
  if(gain>0xff)
    gain=0xff;
  msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
    0x51,(gain),MSM_CAMERA_I2C_BYTE_DATA);
  msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
    0x03,(intg_time_msb),MSM_CAMERA_I2C_BYTE_DATA);
  msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
    0x04,(intg_time_lsb),MSM_CAMERA_I2C_BYTE_DATA);
  return rc;
}

static const struct i2c_device_id gc0339_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&gc0339_s_ctrl},
	{ }
};

static struct i2c_driver gc0339_i2c_driver = {
	.id_table = gc0339_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc0339_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};


static const struct of_device_id gc0339_dt_match[] = {
	{.compatible = "qcom,gc0339", .data = &gc0339_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc0339_dt_match);

static struct platform_driver gc0339_platform_driver = {
	.driver = {
		.name = "qcom,gc0339",
		.owner = THIS_MODULE,
		.of_match_table = gc0339_dt_match,
	},
};

#include "msm.h"
int32_t gc0339_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
   int update_type, int res)
{
  int32_t rc = 0;
  static int csi_config;
  if (update_type == MSM_SENSOR_REG_INIT) {
    CDBG("Register INIT\n");
   // s_ctrl->curr_csi_params = NULL;
    msm_sensor_enable_debugfs(s_ctrl);
    csi_config = 0;
  } else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
    CDBG("PERIODIC : %d\n", res);
    if (!csi_config) {
     // s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
      CDBG("CSI config in progress\n");
      //v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
       // NOTIFY_CSIC_CFG, s_ctrl->curr_csic_params);
      //CDBG("CSI config is done\n");
     v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
  
      mb();
      msleep(30);
      csi_config = 1;
      msm_sensor_write_conf_array(
        s_ctrl->sensor_i2c_client,
        s_ctrl->msm_sensor_reg->mode_settings, res);
    
    }
    msleep(50);
  }
  return rc;
}
int32_t gc0339_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *info = NULL;
	printk(KERN_ERR "%s IN\r\n", __func__);

	info = s_ctrl->sensordata;

	msleep(40);
	gpio_direction_output(75, 1);
	usleep_range(5000, 5100);
	msm_sensor_power_down(s_ctrl);
	msleep(40);
	//if (s_ctrl->sensordata->pmic_gpio_enable){
		//lcd_camera_power_onoff(0);
	//}
	return 0;
}
int32_t gc0339_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
  int32_t rc = 0;
 // static struct regulator *regulator_lvs1;
//  struct msm_camera_sensor_info *info = s_ctrl->sensordata;
  printk(KERN_ERR "%s  %d IN filanto\r\n", __func__,__LINE__);
// gpio_direction_output(75, 0);
//  msleep(20);


  rc = msm_sensor_power_up(s_ctrl);
  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
  if (rc < 0) {
    CDBG("%s: msm_sensor_power_up failed\n", __func__);
    return rc;
  }
//  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
//  regulator_lvs1= regulator_get(NULL, "8038_lvs1");
 // msleep(10);

//  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
//  regulator_set_voltage(regulator_lvs1, 1800000, 1800000);
//  msleep(10);

  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
  gpio_direction_output(75, 1);
  msleep(10);

//  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
//  regulator_enable(regulator_lvs1);
//  msleep(10);

  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
  gpio_direction_output(75, 0);
  msleep(20);

 gpio_direction_output(76, 1);
  msleep(20);
// gpio_direction_output(76, 0);
//  msleep(20);
// gpio_direction_output(76, 1);
//  msleep(20);


  printk(KERN_ERR "%s   %d\r\n", __func__,__LINE__);
  msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
    0xfc, 0x10, MSM_CAMERA_I2C_BYTE_DATA);


  printk(KERN_ERR "%s OUT\r\n", __func__);
  return rc;
}
static int32_t gc0339_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(gc0339_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init msm_sensor_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_probe(&gc0339_platform_driver,
		gc0339_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&gc0339_i2c_driver);
}

static void __exit msm_sensor_exit_module(void)
{
	if (gc0339_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc0339_s_ctrl);
		platform_driver_unregister(&gc0339_platform_driver);
	} else
		i2c_del_driver(&gc0339_i2c_driver);
	return;
}

static int32_t gc0339_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
  uint16_t id = 0;
  int32_t rc = -1;
  CDBG("gc0339 sensor read id\n");
  rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
    s_ctrl->sensor_id_info->sensor_id_reg_addr,
    &id, MSM_CAMERA_I2C_BYTE_DATA);
  CDBG("gc0339 readid 0x%x : 0x%x\n", 0xC8, id);
  if (rc < 0) {
    CDBG("%s: read id failed\n", __func__);
    return rc;
  }
  if (id != s_ctrl->sensor_id_info->sensor_id) {
    return -ENODEV;
  }
  CDBG("gc0339 readid ok, success\n");
  return rc;
}

static struct v4l2_subdev_core_ops gc0339_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops gc0339_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops gc0339_subdev_ops = {
	.core = &gc0339_subdev_core_ops,
	.video  = &gc0339_subdev_video_ops,
};

static struct msm_sensor_fn_t gc0339_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = gc0339_write_exp_gain,
	.sensor_write_snapshot_exp_gain = gc0339_write_exp_gain,
	.sensor_setting = gc0339_sensor_setting,//msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = gc0339_sensor_power_up,// msm_sensor_power_up,
	.sensor_power_down = gc0339_sensor_power_down,//msm_sensor_power_down,
//	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines1,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id   = gc0339_sensor_match_id,
};

static struct msm_sensor_reg_t gc0339_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = 0,
	.start_stream_conf_size = 0,
	.stop_stream_conf = 0,
	.stop_stream_conf_size = 0,
	.group_hold_on_conf = 0,
	.group_hold_on_conf_size = 0,
	.group_hold_off_conf = 0,
	.group_hold_off_conf_size =0,

	.init_settings = &gc0339_init_conf[0],
	.init_size = ARRAY_SIZE(gc0339_init_conf),
	.mode_settings = &gc0339_confs[0],
	.output_settings = &gc0339_dimensions[0],
	.num_conf = ARRAY_SIZE(gc0339_confs),
};

static struct msm_sensor_ctrl_t gc0339_s_ctrl = {
	.msm_sensor_reg = &gc0339_regs,
	.sensor_i2c_client = &gc0339_sensor_i2c_client,
	.sensor_i2c_addr = 0x42,
	.vreg_seq = gc0339_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(gc0339_veg_seq),
	.sensor_output_reg_addr = &gc0339_reg_addr,
	.sensor_id_info = &gc0339_id_info,
	.sensor_exp_gain_info = &gc0339_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &gc0339_mut,
	.sensor_i2c_driver = &gc0339_i2c_driver,
	.sensor_v4l2_subdev_info = gc0339_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc0339_subdev_info),
	.sensor_v4l2_subdev_ops = &gc0339_subdev_ops,
	.func_tbl = &gc0339_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
module_exit(msm_sensor_exit_module);
MODULE_DESCRIPTION("sensor driver");
MODULE_LICENSE("GPL v2");


