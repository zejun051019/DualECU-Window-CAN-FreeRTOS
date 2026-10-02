#include "can_port_mspm0.h"

#include "ti_msp_dl_config.h"
#include "can_test_frame.h"
#include "zdt_motor_can.h"

#include <stddef.h>
#include <string.h>

#define CAN_PORT_MSPM0_COMMAND_TX_BUFFER      (0U)
#define CAN_PORT_MSPM0_STATUS_TX_BUFFER       (1U)
#define CAN_PORT_MSPM0_TEST_TX_BUFFER         (2U)
#define CAN_PORT_MSPM0_MODE_POLL_LIMIT         (100000U)
#define CAN_PORT_MSPM0_TIMESTAMP_PRESCALER     (15U)
#define CAN_PORT_MSPM0_SECOND_FILTER_INDEX     (1U)
#define CAN_PORT_MSPM0_MOTOR_FILTER_INDEX      (0U)
#define CAN_PORT_MSPM0_RECOVERY_RETRY_MS       (250U)

volatile uint32_t g_can_port_bus_off_count;
volatile uint32_t g_can_port_recovery_attempt_count;
volatile uint32_t g_can_port_recovery_complete_count;
volatile uint32_t g_can_port_last_error_code;
static bool s_busOffActive;
static bool s_hasRecoveryAttempt;
static uint32_t s_lastRecoveryAttemptMs;

#if MCAN0_INST_MCAN_STD_ID_FILTER_NUM < 2
#error "SysConfig must reserve two standard CAN filter slots."
#endif

#if MCAN0_INST_MCAN_TX_BUFF_SIZE < 3
#error "SysConfig must reserve separate command, status, and probe TX buffers."
#endif

#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK) && \
    (MCAN0_INST_MCAN_EXT_ID_FILTER_NUM < 1)
#error "SysConfig must reserve one extended CAN filter for the ZDT motor."
#endif

static void canPortMspm0AddSecondFilter(void)
{
    const DL_MCAN_StdMsgIDFilterElement secondFilter = {
        .sfid2 = 0x7FFU,
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
        .sfid1 = CAN_PROTOCOL_STATUS_ID,
#else
        .sfid1 = CAN_TEST_FRAME_F407_TO_G3507_ID,
#endif
        .sfec = 0x1U,
        .sft = 0x2U};

    DL_MCAN_addStdMsgIDFilter(
        MCAN0_INST, CAN_PORT_MSPM0_SECOND_FILTER_INDEX, &secondFilter);
}

#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
static void canPortMspm0AddMotorFilter(void)
{
    const DL_MCAN_ExtMsgIDFilterElement motorFilter = {
        .efid1 = ((uint32_t)ZDT_MOTOR_CAN_DEFAULT_ADDRESS) << 8U,
        .efec = 0x1U,
        .efid2 = 0x1FFFFFFFU,
        .eft = 0x2U};

    DL_MCAN_addExtMsgIDFilter(
        MCAN0_INST, CAN_PORT_MSPM0_MOTOR_FILTER_INDEX, &motorFilter);
}
#endif

static bool canPortMspm0WaitForMode(uint32_t expectedMode)
{
    uint32_t poll;

    for (poll = 0U; poll < CAN_PORT_MSPM0_MODE_POLL_LIMIT; poll++)
    {
        if (DL_MCAN_getOpMode(MCAN0_INST) == expectedMode)
        {
            return true;
        }
    }

    return false;
}

static bool canPortMspm0ClearClockStopRequest(void)
{
    uint32_t poll;

    DL_MCAN_disableClockStopGateRequest(MCAN0_INST);
    DL_MCAN_addClockStopRequest(MCAN0_INST, false);

    for (poll = 0U; poll < CAN_PORT_MSPM0_MODE_POLL_LIMIT; poll++)
    {
        if (DL_MCAN_getClockStopAck(MCAN0_INST) == 0U)
        {
            return true;
        }
    }

    return false;
}

bool can_port_mspm0_init(void)
{
    DL_MCAN_ConfigParams config = {0};

    config.monEnable = 0U;
    config.asmEnable = 0U;
    config.tsPrescalar = CAN_PORT_MSPM0_TIMESTAMP_PRESCALER;
    config.tsSelect = 1U; /* Internal timestamp counter. */
    config.timeoutSelect = DL_MCAN_TIMEOUT_SELECT_CONT;
    config.timeoutPreload = 0xFFFFU;
    config.timeoutCntEnable = 0U;
    config.filterConfig.rrfe = 1U;
    config.filterConfig.rrfs = 1U;
    config.filterConfig.anfe = 2U; /* Reject non-matching extended frames. */
    config.filterConfig.anfs = 2U; /* Reject non-matching standard frames. */

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_SW_INIT);
    if (!canPortMspm0WaitForMode(DL_MCAN_OPERATION_MODE_SW_INIT))
    {
        return false;
    }

    if (!canPortMspm0ClearClockStopRequest())
    {
        return false;
    }

    if (DL_MCAN_config(MCAN0_INST, &config) != 0)
    {
        return false;
    }

    canPortMspm0AddSecondFilter();
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
    canPortMspm0AddMotorFilter();
#endif

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
    return canPortMspm0WaitForMode(DL_MCAN_OPERATION_MODE_NORMAL);
}

bool can_port_mspm0_service_bus_off(uint32_t now_ms)
{
    DL_MCAN_ProtocolStatus status = {0};
    DL_MCAN_getProtocolStatus(MCAN0_INST, &status);
    if ((status.lastErrCode > 0U) && (status.lastErrCode < 7U))
    {
        g_can_port_last_error_code = status.lastErrCode;
    }
    if (status.busOffStatus == 0U)
    {
        if (s_busOffActive)
        {
            s_busOffActive = false;
            ++g_can_port_recovery_complete_count;
        }
        return false;
    }
    if (!s_busOffActive)
    {
        s_busOffActive = true;
        ++g_can_port_bus_off_count;
        /* Discard pending pre-fault transmissions before enabling the link. */
        for (uint32_t buffer = 0U; buffer < 3U; ++buffer)
        {
            (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, buffer);
        }
    }
    if ((DL_MCAN_getOpMode(MCAN0_INST) == DL_MCAN_OPERATION_MODE_SW_INIT) &&
        (!s_hasRecoveryAttempt ||
         ((uint32_t)(now_ms - s_lastRecoveryAttemptMs) >=
          CAN_PORT_MSPM0_RECOVERY_RETRY_MS)))
    {
        s_lastRecoveryAttemptMs = now_ms;
        s_hasRecoveryAttempt = true;
        ++g_can_port_recovery_attempt_count;
        /* Hardware observes the recessive-bit recovery sequence asynchronously. */
        DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
    }
    return true;
}

bool can_port_mspm0_send_frame(const can_protocol_frame_t *frame)
{
    DL_MCAN_TxBufElement txMessage = {0};
    uint32_t txBuffer;

    if (s_busOffActive || (frame == NULL) ||
        (frame->id > (frame->is_extended ? 0x1FFFFFFFU : 0x7FFU)) ||
        (frame->dlc > CAN_PROTOCOL_MAX_DATA_LEN))
    {
        return false;
    }

    if (frame->is_extended)
    {
        txBuffer = CAN_PORT_MSPM0_TEST_TX_BUFFER;
    }
    else if (frame->id == CAN_PROTOCOL_COMMAND_ID)
    {
        txBuffer = CAN_PORT_MSPM0_COMMAND_TX_BUFFER;
    }
    else if (frame->id == CAN_PROTOCOL_STATUS_ID)
    {
        txBuffer = CAN_PORT_MSPM0_STATUS_TX_BUFFER;
    }
    else if (frame->id == CAN_TEST_FRAME_G3507_TO_F407_ID)
    {
        txBuffer = CAN_PORT_MSPM0_TEST_TX_BUFFER;
    }
    else
    {
        return false;
    }

    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & (1U << txBuffer)) != 0U)
    {
        return false;
    }

    txMessage.id = frame->is_extended ? frame->id : frame->id << 18U;
    txMessage.rtr = frame->is_remote ? 1U : 0U;
    txMessage.xtd = frame->is_extended ? 1U : 0U;
    txMessage.dlc = frame->dlc;
    txMessage.brs = 0U;
    txMessage.fdf = 0U;
    (void)memcpy(txMessage.data, frame->data, frame->dlc);

    DL_MCAN_writeMsgRam(
        MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, txBuffer, &txMessage);

    return DL_MCAN_TXBufAddReq(MCAN0_INST, txBuffer) == 0;
}

can_port_mspm0_rx_result_t can_port_mspm0_receive(
    can_protocol_frame_t *frame_out,
    uint16_t *timestamp_out)
{
    DL_MCAN_RxFIFOStatus fifoStatus = {0};
    DL_MCAN_RxBufElement rxMessage = {0};
    uint32_t byteIndex;

    if ((frame_out == NULL) || (timestamp_out == NULL))
    {
        return CAN_PORT_MSPM0_RX_ACK_ERROR;
    }

    fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
    if (fifoStatus.fillLvl == 0U)
    {
        return CAN_PORT_MSPM0_RX_EMPTY;
    }

    DL_MCAN_readMsgRam(
        MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U,
        DL_MCAN_RX_FIFO_NUM_0, &rxMessage);

    if (DL_MCAN_writeRxFIFOAck(
            MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0, fifoStatus.getIdx) != 0)
    {
        return CAN_PORT_MSPM0_RX_ACK_ERROR;
    }

    frame_out->id = rxMessage.xtd != 0U ?
                        (rxMessage.id & 0x1FFFFFFFU) :
                        ((rxMessage.id >> 18U) & 0x7FFU);
    frame_out->dlc = (uint8_t)rxMessage.dlc;
    frame_out->is_extended = (rxMessage.xtd != 0U);
    frame_out->is_remote = (rxMessage.rtr != 0U);
    for (byteIndex = 0U; byteIndex < CAN_PROTOCOL_MAX_DATA_LEN; byteIndex++)
    {
        frame_out->data[byteIndex] = rxMessage.data[byteIndex];
    }
    *timestamp_out = (uint16_t)rxMessage.rxts;

    return CAN_PORT_MSPM0_RX_FRAME;
}

uint16_t can_port_mspm0_timestamp_now(void)
{
    return (uint16_t)DL_MCAN_getTSCounterVal(MCAN0_INST);
}

bool can_port_mspm0_rx_overflow_pending(void)
{
    DL_MCAN_RxFIFOStatus fifoStatus = {0};

    fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
    return fifoStatus.msgLost;
}

bool can_port_mspm0_clear_rx_overflow_if_drained(void)
{
    DL_MCAN_RxFIFOStatus fifoStatus = {0};

    fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
    if (fifoStatus.fillLvl != 0U)
    {
        return false;
    }

    if (fifoStatus.msgLost)
    {
        DL_MCAN_clearIntrStatus(
            MCAN0_INST, DL_MCAN_INTERRUPT_RF0L,
            DL_MCAN_INTR_SRC_MCAN_LINE_0);
    }

    fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
    return (fifoStatus.fillLvl == 0U) && !fifoStatus.msgLost;
}
