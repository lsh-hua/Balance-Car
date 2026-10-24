#include "mpu6050.h"
#include "math.h"
#include "iir_filter.h"
#include "mpu6050_calibration.h"
#include "mpu6050_startup.h"

#define PI 3.14159265f

extern I2C_HandleTypeDef hi2c1;

volatile MPU6050_Diagnostics_t g_mpu6050_diag;
volatile MPU6050_StartupDiagnostics_t g_mpu6050_startup_diag;
static uint8_t config_writes_ok;

/* 陀螺仪静止零偏校准值 */
static int16_t gx_zero = 0, gy_zero = 0, gz_zero = 0;
/* 六组静态姿态拟合的固定参数；独立验证姿态未参与拟合。 */
const MPU6050_AccelCalibration_t g_mpu6050_accel_calibration =
{
    1u,
    {533.3405031034827f, -110.3442435040282f, 2695.955247838345f},
    {1.002155420439347f, 0.9908736522241032f, 0.9679022475546378f}
};


float dt = 0.002f;
static float ax, ay, az;
float gx, gy, gz;

/* ============================================================
 * Phase 1：二阶 Butterworth IIR
 *
 * Fs = 500 Hz
 * Fc = 15 Hz
 *
 * 六个通道分别使用独立滤波器。
 * ============================================================ */

static IIR2_Filter_t accel_x_filter;
static IIR2_Filter_t accel_y_filter;
static IIR2_Filter_t accel_z_filter;

static IIR2_Filter_t gyro_x_filter;
static IIR2_Filter_t gyro_y_filter;
static IIR2_Filter_t gyro_z_filter;


float yaw, pitch, roll;
static float Accel_Angle_x, Accel_Angle_y;

/* ==================== 固定加速度标定 ==================== */
static float Calibrate_Accel_G(int16_t chip_raw_lsb, uint8_t axis)
{
    const MPU6050_AccelCalibration_t *calibration = &g_mpu6050_accel_calibration;
    return ((float)chip_raw_lsb - calibration->bias_lsb[axis])
        * calibration->gain[axis] / 16384.0f;
}

static int16_t Encode_Accel_Lsb(float accel_g)
{
    float value = accel_g * 16384.0f;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}





/* ==================== I2C 与采样有效性 ==================== */
static uint8_t Read_Registers(uint8_t address, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, devAddr, address,
                                               I2C_MEMADD_SIZE_8BIT, data, size, 10);
    g_mpu6050_diag.last_read_status = (uint8_t)status;
    g_mpu6050_diag.last_i2c_error = HAL_I2C_GetError(&hi2c1);
    if (status == HAL_OK) return 1u;

    g_mpu6050_diag.read_errors++;
    g_mpu6050_diag.sample_valid = 0u;
    return 0u;
}

static uint8_t Read_Sample(uint8_t data[14])
{
    if (!Read_Registers(MPU6050_RA_ACCEL_XOUT_H, data, 14)) return 0u;
    if ((data[0] | data[1] | data[2] | data[3] | data[4] | data[5]) == 0u)
    {
        g_mpu6050_diag.zero_samples++;
        g_mpu6050_diag.sample_valid = 0u;
        return 0u;
    }

    g_mpu6050_diag.sample_valid = 1u;
    return 1u;
}

void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_data)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, devAddr, REG_Address,
                                                I2C_MEMADD_SIZE_8BIT, &REG_data, 1, 10);
    if (status == HAL_OK) return;
    config_writes_ok = 0u;
    g_mpu6050_diag.config_valid = 0u;
    g_mpu6050_diag.last_i2c_error = HAL_I2C_GetError(&hi2c1);
    g_mpu6050_startup_diag.write_errors++;
    g_mpu6050_startup_diag.last_write_status = (uint8_t)status;
    g_mpu6050_startup_diag.last_write_register = REG_Address;
    g_mpu6050_startup_diag.last_write_error = g_mpu6050_diag.last_i2c_error;
}

uint8_t Single_ReadI2C(uint8_t REG_Address)
{
    uint8_t REG_data = 0u;
    if (!Read_Registers(REG_Address, &REG_data, 1)) return 0u;
    return REG_data;
}

/* 配置值只有一份，同时用于写入和完整寄存器读回校验。 */
typedef struct
{
    uint8_t address;
    uint8_t value;
    volatile uint8_t *snapshot;
} MPU6050_RegisterSetting_t;

static const MPU6050_RegisterSetting_t startup_settings[] =
{
    {MPU6050_RA_SMPLRT_DIV, 0x01u, 0},
    {MPU6050_RA_CONFIG, 0x02u, 0},
    {MPU6050_RA_GYRO_CONFIG, 0x00u, &g_mpu6050_diag.gyro_config},
    {MPU6050_RA_ACCEL_CONFIG, 0x00u, &g_mpu6050_diag.accel_config}
};

static uint8_t Verify_Register(uint8_t address, uint8_t expected, uint8_t mask,
                               volatile uint8_t *snapshot)
{
    uint8_t actual = 0u;
    if (!Read_Registers(address, &actual, 1)) return 0u;
    if (snapshot) *snapshot = actual;
    if ((actual & mask) == (expected & mask)) return 1u;

    g_mpu6050_startup_diag.verify_errors++;
    g_mpu6050_startup_diag.last_verify_register = address;
    g_mpu6050_startup_diag.last_verify_value = actual;
    return 0u;
}

static uint8_t Read_Config_Diagnostics(void)
{
    g_mpu6050_diag.config_valid = 0u;
    if (!Verify_Register(MPU6050_RA_WHO_AM_I, 0x68u, 0xFFu,
                          &g_mpu6050_diag.who_am_i)) return 0u;
    if (!Verify_Register(MPU6050_RA_PWR_MGMT_1, 0u, 0xEFu,
                          &g_mpu6050_diag.pwr_mgmt_1)) return 0u;
    for (unsigned i = 0; i < sizeof(startup_settings) / sizeof(startup_settings[0]); i++)
    {
        const MPU6050_RegisterSetting_t *setting = &startup_settings[i];
        if (!Verify_Register(setting->address, setting->value, 0xFFu, setting->snapshot)) return 0u;
    }
    g_mpu6050_diag.config_valid = config_writes_ok;
    return g_mpu6050_diag.config_valid;
}

static uint8_t Wake_Sensor(void)
{
    Single_WriteI2C(MPU6050_RA_PWR_MGMT_1, 0x00u);
    if (!config_writes_ok) return 0u;
    if (!Verify_Register(MPU6050_RA_PWR_MGMT_1, 0u, 0xEFu,
                          &g_mpu6050_diag.pwr_mgmt_1)) return 0u;
    /* 唤醒读回成功后再保守等待100ms，并非声称这是精确的硬件最短等待。 */
    HAL_Delay(100);
    return 1u;
}

static uint8_t Configure_Sensor(void)
{
    for (unsigned i = 0; i < sizeof(startup_settings) / sizeof(startup_settings[0]); i++)
    {
        Single_WriteI2C(startup_settings[i].address, startup_settings[i].value);
        if (!config_writes_ok) return 0u;
    }
    return Read_Config_Diagnostics();
}

static uint8_t Run_Configuration_Attempt(void)
{
    g_mpu6050_startup_diag.stage = 1u;
    if (!Wake_Sensor()) return 0u;
    g_mpu6050_startup_diag.stage = 2u;
    return Configure_Sensor();
}

void MPU6050_Init(void)
{
    MPU6050_Diagnostics_t initial_diag = {0};
    MPU6050_StartupDiagnostics_t initial_startup_diag = {0};
    g_mpu6050_diag = initial_diag;
    g_mpu6050_startup_diag = initial_startup_diag;
    HAL_Delay(100);
    for (uint8_t attempt = 1u; attempt <= 3u; attempt++)
    {
        g_mpu6050_startup_diag.attempts = attempt;
        config_writes_ok = 1u;
        g_mpu6050_diag.config_valid = 0u;
        g_mpu6050_diag.calibration_valid = 0u;
        g_mpu6050_diag.calibration_samples = 0u;
        g_mpu6050_diag.sample_valid = 0u;
        if (Run_Configuration_Attempt())
        {
            g_mpu6050_startup_diag.stage = 3u;
            MPU6050_Calibrate();
            if (g_mpu6050_diag.calibration_valid) g_mpu6050_startup_diag.stage = 4u;
            return;
        }
        if (attempt < 3u) HAL_Delay(20);
    }
}

/* 上电静止校准仅更新陀螺仪零偏，不把加速度计中的重力分量当作零偏。 */
void MPU6050_Calibrate(void)
{
    int32_t sx = 0, sy = 0, sz = 0;
    const int samples = 1000;
    uint8_t buf[14];

    g_mpu6050_diag.calibration_valid = 0u;
    g_mpu6050_diag.calibration_samples = 0u;
    if (!g_mpu6050_diag.config_valid)
    {
        g_mpu6050_diag.sample_valid = 0u;
        return;
    }

    for (int i = 0; i < samples; i++)
    {
        /* 连续读取 Accel(6B) + Temp(2B) + Gyro(6B) */
        if (!Read_Sample(buf)) return;
        g_mpu6050_diag.calibration_samples++;

        /* buf[6:7] 温度跳过 */
        sx  += (int16_t)((buf[8]  << 8) | buf[9]);
        sy  += (int16_t)((buf[10] << 8) | buf[11]);
        sz  += (int16_t)((buf[12] << 8) | buf[13]);

        HAL_Delay(1);   /* 匹配 1kHz 采样间隔 */
    }

    gx_zero = (int16_t)(sx / samples);
    gy_zero = (int16_t)(sy / samples);
    gz_zero = (int16_t)(sz / samples);
    g_mpu6050_diag.calibration_valid = 1u;
}

int16_t GetData(uint8_t REG_Address)
{
	uint8_t buf[2] = {0};
    if (!Read_Registers(REG_Address, buf, 2)) return 0;
	return (int16_t)((buf[0] << 8) | buf[1]);
}

/* 加速度返回芯片原码；陀螺仪保持已扣静止零偏的 LSB 契约。 */
void MPU6050_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    *ax = *ay = *az = 0;
    *gx = *gy = *gz = 0;
    if (!Read_Sample(buf)) return;

    int16_t ax_r = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_r = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_r = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx_r = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy_r = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_r = (int16_t)((buf[12] << 8) | buf[13]);

    *ax = ax_r;
    *ay = ay_r;
    *az = az_r;
    *gx = gx_r - gx_zero;
    *gy = gy_r - gy_zero;
    *gz = gz_r - gz_zero;
}







uint8_t MPU6050_ReadRawFiltered(
    int16_t *ax_out, int16_t *ay_out, int16_t *az_out,
    int16_t *gx_out, int16_t *gy_out, int16_t *gz_out,
    int16_t *ax_f_out, int16_t *ay_f_out, int16_t *az_f_out,
    int16_t *gx_f_out, int16_t *gy_f_out, int16_t *gz_f_out)
{
    uint8_t buf[14];

    /* 失败帧全部输出为零，保留上一个有效样本的滤波与姿态状态。 */
    *ax_out = *ay_out = *az_out = 0;
    *gx_out = *gy_out = *gz_out = 0;
    *ax_f_out = *ay_f_out = *az_f_out = 0;
    *gx_f_out = *gy_f_out = *gz_f_out = 0;
    if (!Read_Sample(buf)) return 0u;

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    /* =========================================================
     * 1. Raw：加速度保留芯片原码，陀螺仪扣除静止零偏
     * ========================================================= */

    int16_t gx_value = gx_raw - gx_zero;
    int16_t gy_value = gy_raw - gy_zero;
    int16_t gz_value = gz_raw - gz_zero;

    *ax_out = ax_raw;
    *ay_out = ay_raw;
    *az_out = az_raw;

    *gx_out = gx_value;
    *gy_out = gy_value;
    *gz_out = gz_value;


    /* =========================================================
     * 2. 转换成物理量
     *    加速度：固定标定后的 g；减偏和乘增益均使用浮点数
     *    陀螺仪：°/s
     * ========================================================= */

    float ax_float = Calibrate_Accel_G(ax_raw, 0u);
    float ay_float = Calibrate_Accel_G(ay_raw, 1u);
    float az_float = Calibrate_Accel_G(az_raw, 2u);

    float gx_float = gx_value / 131.0f;
    float gy_float = gy_value / 131.0f;
    float gz_float = gz_value / 131.0f;


    /* =========================================================
     * 3. 二阶 Butterworth IIR 15 Hz
     * ========================================================= */

    float ax_filtered = IIR2_Update(&accel_x_filter, ax_float);
    float ay_filtered = IIR2_Update(&accel_y_filter, ay_float);
    float az_filtered = IIR2_Update(&accel_z_filter, az_float);

    float gx_filtered = IIR2_Update(&gyro_x_filter, gx_float);
    float gy_filtered = IIR2_Update(&gyro_y_filter, gy_float);
    float gz_filtered = IIR2_Update(&gyro_z_filter, gz_float);


    /* =========================================================
     * 4. 再转换回 LSB
     *
     * Raw 为芯片原码，Filtered 为标定后结果，不能混作同一标定域。
     * 仅遥测编码做饱和；姿态使用下面保存的完整浮点物理量。
     * ========================================================= */

    *ax_f_out = Encode_Accel_Lsb(ax_filtered);
    *ay_f_out = Encode_Accel_Lsb(ay_filtered);
    *az_f_out = Encode_Accel_Lsb(az_filtered);

    *gx_f_out = (int16_t)(gx_filtered * 131.0f);
    *gy_f_out = (int16_t)(gy_filtered * 131.0f);
    *gz_f_out = (int16_t)(gz_filtered * 131.0f);
		
		
					/* 将滤波后的物理量保存到全局变量，
				 供 GET_Angle() 使用 */
			ax = ax_filtered;
			ay = ay_filtered;
			az = az_filtered;

			gx = gx_filtered;
			gy = gy_filtered;
			gz = gz_filtered;
    return 1u;
}








/* 一次性 Burst 读取加速度(6B)+温度(2B)+陀螺仪(6B)，减少 I2C 开销 */
void MPU6050_Update(void)
{
	uint8_t buf[14];
    if (!g_mpu6050_diag.config_valid || !g_mpu6050_diag.calibration_valid)
    {
        g_mpu6050_diag.sample_valid = 0u;
        return;
    }
    if (!Read_Sample(buf)) return;

	int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
	int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
	int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
	/* buf[6:7] 温度，忽略 */
	int16_t gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
	int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
	int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

	ax = Calibrate_Accel_G(ax_raw, 0u);
	ay = Calibrate_Accel_G(ay_raw, 1u);
	az = Calibrate_Accel_G(az_raw, 2u);

	/* ±250°/s 量程 */
	gx = (gx_raw - gx_zero) / 131.0f;
	gy = (gy_raw - gy_zero) / 131.0f;
	gz = (gz_raw - gz_zero) / 131.0f;
}

/* 一阶互补滤波，tau 为时间常数（秒），可根据响应需求调整
 * tau 越大 -> 陀螺仪信任度越高，曲线越平滑但漂移恢复越慢
 * tau 越小 -> 加速度计信任度越高，响应越快但易受振动干扰 */
#define COMP_FILTER_TAU  0.25f

float Complementary_Filter_x(float angle_m, float gyro_m)
{
	static float angle = 0.0f;
	static uint8_t initialized = 0;
	float alpha = COMP_FILTER_TAU / (COMP_FILTER_TAU + dt);
	 if (!initialized)
    {
        angle = angle_m;
        initialized = 1;
    }
    else
    {
        angle = alpha * (angle + gyro_m * dt)
              + (1.0f - alpha) * angle_m;
    }

    return angle;
}

float Complementary_Filter_y(float angle_m, float gyro_m)
{
		static float angle = 0.0f;
    static uint8_t initialized = 0;

    float alpha = COMP_FILTER_TAU / (COMP_FILTER_TAU + dt);

    if (!initialized)
    {
        angle = angle_m;
        initialized = 1;
    }
    else
    {
        angle = alpha * (angle + gyro_m * dt)
              + (1.0f - alpha) * angle_m;
    }

    return angle;
}

void GET_Angle(void)
{
    if (!g_mpu6050_diag.sample_valid) return;
	Accel_Angle_x = atan2f(ay, az) * 180.0f / PI;
	Accel_Angle_y = atan2f(ax, az) * 180.0f / PI;
	
	pitch = -Complementary_Filter_x(Accel_Angle_x, gx);
	roll  = -Complementary_Filter_y(Accel_Angle_y, gy);
}



void MPU6050_Filter_Init(void)
{
    /*
     * 初始化时先读取一次当前传感器数据，
     * 让滤波器从当前值开始。
     */

    MPU6050_Update();
    if (!g_mpu6050_diag.sample_valid) return;

    IIR2_Init(&accel_x_filter, ax);
    IIR2_Init(&accel_y_filter, ay);
    IIR2_Init(&accel_z_filter, az);

    IIR2_Init(&gyro_x_filter, gx);
    IIR2_Init(&gyro_y_filter, gy);
    IIR2_Init(&gyro_z_filter, gz);
}

void MPU6050_Update_Filtered(void)
{
    /*
     * 先读取一次最新 MPU6050 数据
     */
    MPU6050_Update();
    if (!g_mpu6050_diag.sample_valid) return;

    /*
     * 加速度计低通
     */
    ax = IIR2_Update(&accel_x_filter, ax);
    ay = IIR2_Update(&accel_y_filter, ay);
    az = IIR2_Update(&accel_z_filter, az);

    /*
     * 陀螺仪低通
     */
    gx = IIR2_Update(&gyro_x_filter, gx);
    gy = IIR2_Update(&gyro_y_filter, gy);
    gz = IIR2_Update(&gyro_z_filter, gz);
}


