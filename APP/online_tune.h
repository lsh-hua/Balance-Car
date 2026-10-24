#ifndef ONLINE_TUNE_H
#define ONLINE_TUNE_H

#include <stddef.h>
#include <stdint.h>

#ifndef ONLINE_TUNING_ENABLE
/* 当前使用离线固定参数：校准完成后按main总开关运行，不接收在线命令。 */
#define ONLINE_TUNING_ENABLE 0
#endif

/* trim使用毫度；命令校验与实际应用共用同一允许范围。 */
#define ONLINE_TUNE_TRIM_MIN_MDEG (-2000)
#define ONLINE_TUNE_TRIM_MAX_MDEG 2000

/* PWM命令计数范围；协议入口与控制应用入口共用边界。 */
#define ONLINE_TUNE_OUTPUT_LIMIT_MIN 100
#define ONLINE_TUNE_OUTPUT_LIMIT_MAX 3500

typedef struct
{
    int32_t kp_milli;
    int32_t kd_milli;
    int32_t trim_mdeg;
    int32_t output_limit;
} OnlineTuneParams_t;

#if ONLINE_TUNING_ENABLE
/* 历史可选功能：ISR只提交接收字节/错误，解析和发送留在主循环。 */
void OnlineTune_Init(void);
void OnlineTune_RxByte(uint8_t byte);
void OnlineTune_RxError(void);
void OnlineTune_Process(void);
void OnlineTune_ControlTick(uint32_t now_ms, uint32_t tick,
                           uint8_t healthy, float absolute_pitch);
uint8_t OnlineTune_IsArmed(void);
/* 返回不含字符串终止符的长度；无回执或缓冲不足返回0，不丢弃该回执。 */
size_t OnlineTune_TakeReply(char *out, size_t capacity);
#endif

#endif
