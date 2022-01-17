#include "S_MPU6050_MPL.h"

uint8_t MPU_i2cWrite(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	uint32_t status = HAL_I2C_Mem_Write(&MPU_I2C_ADDR, slave_addr << 1, reg_addr, 1, data, length, 10);
	if (status != HAL_OK)
		UART_printf(&huart1, "[ERROR]I2C_write error\n");
	return status;
}
uint8_t MPU_i2cRead(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	uint32_t status = HAL_I2C_Mem_Read(&MPU_I2C_ADDR, slave_addr << 1, reg_addr, 1, data, length, 10);
	if (status != HAL_OK)
		UART_printf(&huart1, "[ERROR]I2C_write error\n");
	return status;
}
void MPL_getms(uint32_t *num)
{
	*num = Get_Systick();
}
void MPL_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int length;
	char buffer[128];
	length = vsnprintf(buffer, 128, fmt, ap);
	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, 0xffff);
}

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

struct platform_data_s
{
	signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
	.orientation = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1}};

// static void setup_gyro(void)
// {
// 	unsigned char mask = 0;
// 	mask |= INV_XYZ_ACCEL;//设置加速度计
// 	mask |= INV_XYZ_GYRO;//设置陀螺仪

// 	mpu_set_sensors(mask);
// 	mpu_configure_fifo(mask);
// }

static inline void run_self_test(void)
{
	int result;
	int32_t gyro[3],accel[3];
	
	result = mpu_run_self_test(gyro,accel);
	if(result == 0x7)
	{
		UART_printf(&huart1,"self test passed!\n");

	for(uint8_t i = 0;i < 3;i++)
	{
		gyro[i] = (int32_t)(gyro[i] * 32.8f);
		accel[i] *= 2048.0f;
		accel[i] = accel[i] >> 16;
		gyro[i] = (int32_t)(gyro[i]>>16);
	}
	
	/**
	 * 把计算出来的偏置量推给MPL库 
	 */
	uint16_t accel_sens;
	float gyro_sens;
	
	mpu_get_accel_sens(&accel_sens);
	accel[0] *= accel_sens;
	accel[1] *= accel_sens;
	accel[2] *= accel_sens;
	inv_set_accel_bias(accel,3);
	mpu_get_gyro_sens(&gyro_sens);
	gyro[0] = (int32_t)(gyro[0]*gyro_sens);
	gyro[1] = (int32_t)(gyro[1]*gyro_sens);
	gyro[2] = (int32_t)(gyro[2]*gyro_sens);
	inv_set_gyro_bias(gyro,3);
	}
	else{
		if(!(result & 0x1))
		UART_printf(&huart1,"[Error]gyro failed\n");
		if(!(result & 0x2))
		UART_printf(&huart1,"[ERROR]accel failed\n");
	}
}

void MPU6050_Init(void)
{
	inv_error_t result;
	uint8_t accel_fsr;
	uint16_t gyro_rate,gyro_fsr;
	struct int_param_s int_param;

	result = mpu_init(&int_param);
	if(result)
	{
		UART_printf(&huart1,"[ERROR]could not init gyro\n");
	}

	result = inv_init_mpl();
	if(result)
	{
		UART_printf(&huart1,"[ERROR]could not init MPL\n");
	}

	inv_enable_quaternion();//使能六轴融合
	inv_enable_fast_nomot();
	inv_enable_gyro_tc();
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if(result)
	{
		UART_printf(&huart1,"[ERROR]MPL not start\n");
	}
	result = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置需要的传感器
	if(result)
	{
		UART_printf(&huart1,"[ERROR]cannot set sensor\n");
	}

	result = mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置FIFO
	if(result)
	{
		UART_printf(&huart1,"[ERROR]cannot configure fifo\n");
	}

	mpu_set_sample_rate(100);

	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);

	inv_set_gyro_sample_rate(1000000L/gyro_rate);
	inv_set_accel_sample_rate(1000000L/gyro_rate);
	
	inv_set_gyro_orientation_and_scale(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
		(int32_t)gyro_fsr<<15);
	inv_set_accel_orientation_and_scale(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
		(int32_t)accel_fsr<<15);

	dmp_load_motion_driver_firmware();
	dmp_set_orientation(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	
	dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_GYRO_CAL);
	
	dmp_set_fifo_rate(100);
	
	mpu_set_dmp_state(1);
	
	UART_printf(&huart1,"MPU6050InitOK!\n");
}


uint8_t MPUGetData(float *pitch,float *roll,float *yaw)
{
	uint32_t sensor_timestamp,timestamp;
	int16_t gyro[3],accel_short[3],sensors;
	uint8_t more;
	int32_t accel[3],quat[4],temperature;
	int32_t data[9];
	int8_t accuracy;

	if(dmp_read_fifo(gyro,accel_short,quat,&sensor_timestamp,&sensors,&more))
	{
		UART_printf(&huart1,"[ERROR]read dmp info error!\n");
		return 1;
	}
	if(sensors & INV_XYZ_GYRO)
	{
		inv_build_gyro(gyro,sensor_timestamp);			//把陀螺仪数据发送给MPL
		mpu_get_temperature(&temperature,&sensor_timestamp);
		inv_build_temp(temperature,sensor_timestamp);	//把温度值发送给MPL，大概是解决温飘
	}
	if(sensors & INV_XYZ_ACCEL)
	{
		accel[0] = (int32_t)accel_short[0];
		accel[1] = (int32_t)accel_short[1];
		accel[2] = (int32_t)accel_short[2];
		inv_build_accel(accel,0,sensor_timestamp);//把加速度值发送给MPL
	}
	inv_execute_on_data();
	inv_get_sensor_type_euler(data,&accuracy,&timestamp);

	*roll = (data[0]/65536.0f);
	*pitch = (data[1]/65535.0f);
	*yaw = (data[2]/65536.0f);

	return 0;
}
