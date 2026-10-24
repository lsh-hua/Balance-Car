#include "online_tune.h"

/* 在线实现保留供历史回归；默认固件不包含接收队列、事务或心跳状态。 */
#if ONLINE_TUNING_ENABLE
#include "balance.h"
#include "main.h"
#include <float.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#define RX_SIZE 256u
#define FRAME_SIZE 26u
#define REPLY_COUNT 8u
#define WATCHDOG_MS 750u
#define OP_GET 1u
#define OP_SET 2u
#define OP_ARM 3u
#define OP_STOP 4u
#define OP_PING 5u

typedef struct
{
    uint8_t version, op;
    uint32_t id;
    int32_t a, b, c, d;
} Command;

typedef struct
{
    uint32_t id, epoch, rev, tick;
    int32_t base_mdeg;
    OnlineTuneParams_t parameters;
    uint8_t op, rc, armed, reason;
} Reply;

static volatile uint16_t rx_head, rx_tail;
static uint8_t rx_bytes[RX_SIZE];
static volatile uint32_t rx_generation;
static volatile uint8_t rx_fault;
static uint32_t parser_generation;
static uint8_t frame[FRAME_SIZE], frame_used;
static Command mailbox, stop_mailbox;
static volatile uint8_t mail_ready, stop_ready, armed;
static uint32_t epoch, revision, current_tick, last_ping;
static uint8_t last_reason, previous_guard;
static Reply reply_queue[REPLY_COUNT], deferred_fault;
static uint8_t reply_head, reply_tail, reply_count, fault_reply_pending;

/* 临界区只复制状态，不解析CRC、不格式化或发送串口，恢复调用前的PRIMASK。 */
static uint32_t Lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static uint16_t Crc(const uint8_t *data, size_t size)
{
    uint16_t value = 0xFFFFu;
    while (size--)
    {
        value ^= (uint16_t)*data++ << 8;
        for (unsigned bit = 0; bit < 8; bit++)
            value = (uint16_t)((value << 1) ^ ((value & 0x8000u) ? 0x1021u : 0u));
    }
    return value;
}

static uint32_t ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
           (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static int32_t ReadI32(const uint8_t *data)
{
    uint32_t value = ReadU32(data);
    return (value & 0x80000000u) ? -(int32_t)(~value) - 1 : (int32_t)value;
}

static void NextEpoch(void)
{
    /* ARM令牌经过有符号i32字段，不能产生无法传回的uint32大值。 */
    epoch = epoch >= (uint32_t)INT32_MAX ? 1u : epoch + 1u;
}

static Reply Snapshot(uint32_t id, uint8_t op, uint8_t rc, uint8_t reason)
{
    Reply reply;
    reply.id = id; reply.op = op; reply.rc = rc; reply.reason = reason;
    reply.epoch = epoch; reply.rev = revision; reply.tick = current_tick;
    reply.armed = armed;
    reply.base_mdeg = Control_GetBasePitchMdeg();
    Control_GetTuning(&reply.parameters);
    return reply;
}

static void QueueReply(Reply reply)
{
    reply_queue[reply_tail] = reply;
    reply_tail = (uint8_t)((reply_tail + 1u) % REPLY_COUNT);
    reply_count++;
}

static void RxFaultLocked(void)
{
    if (!rx_fault) rx_generation++;
    rx_fault = 1u;
    rx_tail = rx_head;
    mail_ready = 0u;
}

static void StopState(uint8_t reason)
{
    armed = 0u;
    last_reason = reason;
    NextEpoch();
}

static void Emit(Reply reply)
{
    if (reply_count < REPLY_COUNT && !fault_reply_pending) { QueueReply(reply); return; }
    /* 主循环也可能触发满载；立即撤销许可，状态变更和事件快照留到控制tick。 */
    RxFaultLocked();
}

static void EmitAutomatic(uint8_t reason)
{
    Reply reply = Snapshot(0u, OP_STOP, 0u, reason);
    if (reply_count < REPLY_COUNT && !fault_reply_pending) { QueueReply(reply); return; }
    /* 满载时冻结最后一次自动停机事件，不递归制造新的接收错误。 */
    deferred_fault = reply;
    fault_reply_pending = 1u;
}

static void CancelDangerousMail(void)
{
    /* 故障发生前排队的SET/ARM不得在故障同一tick重新生效。 */
    if (mail_ready && (mailbox.op == OP_SET || mailbox.op == OP_ARM))
    {
        Reply canceled = Snapshot(mailbox.id, mailbox.op, 7u, 0u);
        mail_ready = 0u;
        Emit(canceled);
    }
}

static uint8_t Validate(const Command *command)
{
    if (command->version != 1u) return 1u;
    if (!command->id) return 9u;
    if (command->op < OP_GET || command->op > OP_PING) return 2u;
    if (command->op == OP_SET)
        return (command->a < 0 || command->a > 600000 || command->b < 0 || command->b > 5000 ||
                command->c < ONLINE_TUNE_TRIM_MIN_MDEG || command->c > ONLINE_TUNE_TRIM_MAX_MDEG ||
                command->d < ONLINE_TUNE_OUTPUT_LIMIT_MIN ||
                command->d > ONLINE_TUNE_OUTPUT_LIMIT_MAX) ? 3u : 0u;
    if (command->op == OP_ARM) return (command->b || command->c || command->d) ? 9u : 0u;
    return (command->a || command->b || command->c || command->d) ? 9u : 0u;
}

static void Dispatch(Command command, uint32_t generation)
{
    uint8_t rc = Validate(&command);
    uint32_t mask = Lock();
    if (generation != rx_generation || rx_fault) { __set_PRIMASK(mask); return; }
    if (rc) { Emit(Snapshot(command.id, command.op, rc, 0u)); __set_PRIMASK(mask); return; }
    if (command.op == OP_STOP)
    {
        /* STOP有独立优先槽，并在同一临界区丢弃尚未应用的SET/ARM。 */
        if (mail_ready) { Reply canceled = Snapshot(mailbox.id, mailbox.op, 7u, 0u); mail_ready = 0u; Emit(canceled); }
        if (stop_ready) Emit(Snapshot(command.id, command.op, 7u, 1u));
        else { stop_mailbox = command; stop_ready = 1u; }
    }
    else if (mail_ready || stop_ready || fault_reply_pending)
        Emit(Snapshot(command.id, command.op, 7u, 0u));
    else if (reply_count >= REPLY_COUNT)
        RxFaultLocked();
    else { mailbox = command; mail_ready = 1u; }
    __set_PRIMASK(mask);
}

void OnlineTune_Init(void)
{
    uint32_t mask = Lock();
    rx_head = rx_tail = 0u;
    rx_generation = parser_generation = 0u;
    rx_fault = frame_used = mail_ready = stop_ready = armed = 0u;
    reply_head = reply_tail = reply_count = fault_reply_pending = 0u;
    epoch = 1u; revision = current_tick = last_ping = 0u;
    last_reason = previous_guard = 0u;
    __set_PRIMASK(mask);
}

void OnlineTune_RxByte(uint8_t byte)
{
    uint32_t mask = Lock();
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_SIZE);
    if (rx_fault) { __set_PRIMASK(mask); return; }
    if (next == rx_tail) RxFaultLocked();
    else { rx_bytes[rx_head] = byte; rx_head = next; }
    __set_PRIMASK(mask);
}

void OnlineTune_RxError(void)
{
    uint32_t mask = Lock();
    RxFaultLocked();
    __set_PRIMASK(mask);
}

static void ParseByte(uint8_t byte, uint32_t generation)
{
    if (parser_generation != generation) { frame_used = 0u; parser_generation = generation; }
    if (!frame_used && byte != 0xC3u) return;
    frame[frame_used++] = byte;
    while (frame_used >= 2u)
    {
        if (frame[0] != 0xC3u || frame[1] != 0x3Cu)
        { memmove(frame, frame + 1, --frame_used); continue; }
        if (frame_used < FRAME_SIZE) return;
        if (Crc(frame + 2, 22u) != (uint16_t)(frame[24] | (uint16_t)frame[25] << 8))
        { memmove(frame, frame + 1, --frame_used); continue; }
        Command command;
        command.version = frame[2]; command.op = frame[3]; command.id = ReadU32(frame + 4);
        command.a = ReadI32(frame + 8); command.b = ReadI32(frame + 12);
        command.c = ReadI32(frame + 16); command.d = ReadI32(frame + 20);
        frame_used = 0u;
        Dispatch(command, generation);
    }
}

void OnlineTune_Process(void)
{
    for (unsigned budget = 0; budget < 128u; budget++)
    {
        uint32_t mask = Lock();
        if (rx_tail == rx_head || rx_fault) { __set_PRIMASK(mask); break; }
        uint32_t generation = rx_generation;
        uint8_t byte = rx_bytes[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_SIZE);
        __set_PRIMASK(mask);
        ParseByte(byte, generation);
    }
}

static uint8_t GuardReason(uint8_t healthy, float pitch, float relative)
{
    if (!healthy || pitch != pitch || pitch > FLT_MAX || pitch < -FLT_MAX) return 3u;
    return (relative > 45.0f || relative < -45.0f) ? 4u : 0u;
}

static uint8_t ApplyCommand(Command command, uint32_t now_ms, uint8_t healthy, float relative)
{
    if (command.op == OP_SET)
    {
        if (armed) return 4u;
        OnlineTuneParams_t parameters = {command.a, command.b, command.c, command.d};
        Control_ApplyTuning(&parameters);
        revision++; NextEpoch();
    }
    else if (command.op == OP_ARM)
    {
        if (!healthy) return 5u;
        if (relative > 8.0f || relative < -8.0f) return 6u;
        if (command.a < 1 || (uint32_t)command.a != epoch) return 8u;
        Control_ResetTuningState();
        armed = 1u; last_ping = now_ms; last_reason = 0u; NextEpoch();
    }
    else if (command.op == OP_PING) last_ping = now_ms;
    return 0u;
}

void OnlineTune_ControlTick(uint32_t now_ms, uint32_t tick, uint8_t healthy, float absolute_pitch)
{
    uint32_t mask = Lock();
    current_tick = tick;
    OnlineTuneParams_t parameters;
    Control_GetTuning(&parameters);
    float relative = absolute_pitch - (Control_GetBasePitchMdeg() / 1000.0f + parameters.trim_mdeg / 1000.0f);
    uint8_t guard = GuardReason(healthy, absolute_pitch, relative);
    if (rx_fault)
    {
        rx_fault = 0u;
        StopState(5u);
        EmitAutomatic(5u);
    }
    if (guard && (armed || previous_guard != guard))
    {
        StopState(guard);
        EmitAutomatic(guard);
    }
    previous_guard = guard;
    if (armed && (uint32_t)(now_ms - last_ping) >= WATCHDOG_MS)
    {
        CancelDangerousMail();
        StopState(2u);
        EmitAutomatic(2u);
    }
    if (stop_ready)
    {
        Command command = stop_mailbox;
        stop_ready = mail_ready = 0u;
        StopState(1u);
        Emit(Snapshot(command.id, OP_STOP, 0u, 1u));
    }
    else if (mail_ready)
    {
        Command command = mailbox;
        mail_ready = 0u;
        if (reply_count >= REPLY_COUNT || fault_reply_pending) RxFaultLocked();
        else
        {
            uint8_t rc = ApplyCommand(command, now_ms, (uint8_t)(guard != 3u), relative);
            Emit(Snapshot(command.id, command.op, rc,
                          rc == 5u ? 3u : rc == 6u ? 4u : last_reason));
        }
    }
    __set_PRIMASK(mask);
}

uint8_t OnlineTune_IsArmed(void)
{
    uint32_t mask = Lock();
    uint8_t permitted = (uint8_t)(armed && !rx_fault && !stop_ready && !fault_reply_pending);
    __set_PRIMASK(mask);
    return permitted;
}

size_t OnlineTune_TakeReply(char *out, size_t capacity)
{
    if (!out || !capacity) return 0u;
    uint32_t mask = Lock();
    if (!reply_count && fault_reply_pending)
    { QueueReply(deferred_fault); fault_reply_pending = 0u; }
    if (!reply_count) { __set_PRIMASK(mask); return 0u; }
    Reply reply = reply_queue[reply_head];
    __set_PRIMASK(mask);

    /* 只在主循环格式化已冻结的应用快照，输出期间ISR不会改这组字段。 */
    char text[240];
    int length = snprintf(text, sizeof text,
        "[TUNE] v=1 id=%lu op=%u rc=%u epoch=%lu rev=%lu tick=%lu armed=%u kp=%ld kd=%ld trim=%ld limit=%ld base=%ld reason=%u",
        (unsigned long)reply.id, (unsigned)reply.op, (unsigned)reply.rc,
        (unsigned long)reply.epoch, (unsigned long)reply.rev, (unsigned long)reply.tick,
        (unsigned)reply.armed, (long)reply.parameters.kp_milli, (long)reply.parameters.kd_milli,
        (long)reply.parameters.trim_mdeg, (long)reply.parameters.output_limit, (long)reply.base_mdeg, (unsigned)reply.reason);
    if (length < 0 || length > (int)sizeof(text) - 8) return 0u;
    uint16_t crc = Crc((const uint8_t *)text, (size_t)length);
    int suffix = snprintf(text + length, sizeof(text) - (size_t)length, "*%04X\r\n", (unsigned)crc);
    if (suffix != 7 || capacity <= (size_t)length + 7u) return 0u;
    size_t size = (size_t)length + 7u;
    memcpy(out, text, size + 1u);

    mask = Lock();
    reply_head = (uint8_t)((reply_head + 1u) % REPLY_COUNT);
    reply_count--;
    if (fault_reply_pending && reply_count < REPLY_COUNT)
    { QueueReply(deferred_fault); fault_reply_pending = 0u; }
    __set_PRIMASK(mask);
    return size;
}
#endif
