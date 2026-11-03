/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           UART <-> I2C (Master) Bridge
 *
 *           Host (PC) sends binary command frames over UART0, firmware
 *           relays the payload to an external I2C slave device (I2C0 as
 *           Master). Read-back data from the I2C slave is relayed back to
 *           the host over UART0, so the link is fully bidirectional.
 *           The default I2C slave address is 0x5A (7-bit).
 *
 *           Transparent framing: 10 ms UART silence terminates a packet.
 *           Every received byte (including 0x49) is I2C payload; no CMD,
 *           length prefix, checksum or terminator is added or removed.
 *           Maximum packet size: 64 bytes. Oversized/corrupt packets are
 *           discarded, not partially transmitted. Separate packets by at
 *           least 10 ms (20 ms recommended with USB-UART adapters).
 *
 *           UART writes return 00 on success, EE on failure.
 *           Independently, every 20 ms the Master polls the paired bridge
 *           Slave: SLA+R -> length (0..64) -> payload -> STOP. Only payload
 *           is forwarded to UART. Empty polls produce no UART output.
 *           The header is ACKed so hardware can receive it; for length 0
 *           one padding byte is received with NACK and discarded.
 *           Both boards must use this same length-prefixed read protocol.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Bridge protocol definitions                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_SLAVE_ADDR_DEFAULT  0x5A   /* Default 7-bit I2C slave address */
#define I2C_BUF_SIZE            64u    /* Max bytes per I2C transaction   */
#define I2C_READ_FRAME_SIZE     (I2C_BUF_SIZE + 1u) /* Length + payload */
#define UART_RX_BUF_SIZE        256u   /* UART RX ring buffer size        */

#define UART_PACKET_IDLE_MS     10u
#define I2C_TRANSFER_TIMEOUT_MS 100u
#define I2C_READ_POLL_MS         20u

#define BRIDGE_STATUS_OK        0x00u
#define BRIDGE_STATUS_ERR       0xEEu

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t  g_u8I2CSlaveAddr = I2C_SLAVE_ADDR_DEFAULT;

static volatile uint8_t g_au8I2CTxBuf[I2C_BUF_SIZE];
static volatile uint8_t g_au8I2CRxBuf[I2C_READ_FRAME_SIZE];
volatile uint8_t  g_u8I2CTxLen, g_u8I2CTxIdx;
volatile uint8_t  g_u8I2CRxLen, g_u8I2CRxIdx;
volatile uint8_t  g_u8I2CEndFlag = 0;
volatile uint8_t  g_u8I2CErrFlag = 0;
volatile uint8_t  g_u8TimeoutFlag = 0;
static volatile uint8_t g_u8LengthPrefixedRead = 0;
/* Debugger counters, not bytes injected into the UART data stream. */
volatile uint32_t g_u32I2CReadErrorCount = 0;
volatile uint32_t g_u32I2CReadByteCount = 0;
volatile uint8_t g_u8LastI2CStatus = 0;

static uint8_t g_au8BridgeTxStage[I2C_BUF_SIZE];
static uint8_t g_au8BridgeRxStage[I2C_READ_FRAME_SIZE];

/* UART0 RX ring buffer (single-producer ISR / single-consumer main loop) */
static volatile uint8_t  g_au8UartRxBuf[UART_RX_BUF_SIZE];
/* Preserve packet gaps even while main is busy with an I2C transaction. */
static volatile uint32_t g_au32UartRxTime[UART_RX_BUF_SIZE];
static volatile uint16_t g_u16UartRxHead = 0;
static volatile uint16_t g_u16UartRxTail = 0;
static volatile uint32_t g_u32TickMs = 0;
static volatile uint32_t g_u32LastUartRxMs = 0;
static volatile uint8_t g_u8UartRxError = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_pfnI2C0Handler = NULL;

void I2C0_Init(void);

void SysTick_Handler(void)
{
    g_u32TickMs++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART0 IRQ Handler - drains RX FIFO into the ring buffer                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32Errors = UART0->FIFOSTS & (UART_FIFOSTS_RXOVIF_Msk |
                         UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_FEF_Msk);

    if (u32Errors != 0U)
    {
        UART0->FIFOSTS = u32Errors;
        g_u8UartRxError = 1;
        g_u32LastUartRxMs = g_u32TickMs;
    }

    while (!UART_GET_RX_EMPTY(UART0))
    {
        uint16_t u16Next = (uint16_t)((g_u16UartRxHead + 1U) % UART_RX_BUF_SIZE);
        uint8_t  u8Data  = (uint8_t)UART_READ(UART0);

        g_u32LastUartRxMs = g_u32TickMs;

        if (u16Next != g_u16UartRxTail)
        {
            g_au8UartRxBuf[g_u16UartRxHead] = u8Data;
            g_au32UartRxTime[g_u16UartRxHead] = g_u32LastUartRxMs;
            g_u16UartRxHead = u16Next;
        }
        else
        {
            g_u8UartRxError = 1;
        }
    }
}

static void UART_SendBytes(const uint8_t *pu8Buf, uint32_t u32Len)
{
    uint32_t i;

    for (i = 0; i < u32Len; i++)
    {
        uint32_t u32StartMs = g_u32TickMs;

        while (UART_IS_TX_FULL(UART0))
        {
            if ((uint32_t)(g_u32TickMs - u32StartMs) >= I2C_TRANSFER_TIMEOUT_MS)
            {
                return;
            }
        }

        UART_WRITE(UART0, pu8Buf[i]);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if (s_pfnI2C0Handler != NULL)
        {
            s_pfnI2C0Handler(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master state machine - handles Write / Read / Write+Read(repeated START) transactions              */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterHandler(uint32_t u32Status)
{
    g_u8LastI2CStatus = (uint8_t)u32Status;

    switch (u32Status)
    {
        case 0x08:  /* START has been transmitted */
            if (g_u8I2CTxLen > 0)
            {
                I2C_SET_DATA(I2C0, (uint32_t)(g_u8I2CSlaveAddr << 1));             /* SLA+W */
            }
            else
            {
                I2C_SET_DATA(I2C0, (uint32_t)((g_u8I2CSlaveAddr << 1) | 0x01));    /* SLA+R */
            }

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            break;

        case 0x18:  /* SLA+W has been transmitted and ACK has been received */
            I2C_SET_DATA(I2C0, g_au8I2CTxBuf[g_u8I2CTxIdx++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            break;

        case 0x28:  /* DATA has been transmitted and ACK has been received */
            if (g_u8I2CTxIdx < g_u8I2CTxLen)
            {
                I2C_SET_DATA(I2C0, g_au8I2CTxBuf[g_u8I2CTxIdx++]);
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else if (g_u8I2CRxLen > 0)
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);   /* Repeated START -> read phase */
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8I2CEndFlag = 1;
            }

            break;

        case 0x10:  /* Repeat START has been transmitted and prepare SLA+R */
            I2C_SET_DATA(I2C0, (uint32_t)((g_u8I2CSlaveAddr << 1) | 0x01));
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            break;

        case 0x40:  /* SLA+R has been transmitted and ACK has been received */
            if (g_u8I2CRxLen > 1)
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);   /* Will NACK the only/last byte */
            }

            break;

        case 0x50:  /* DATA has been received and ACK has been returned */
            g_au8I2CRxBuf[g_u8I2CRxIdx++] = (uint8_t)I2C_GET_DATA(I2C0);

            if ((g_u8LengthPrefixedRead != 0U) && (g_u8I2CRxIdx == 1U))
            {
                uint8_t u8PayloadLen = g_au8I2CRxBuf[0];

                /* The first ACKed byte determines this transaction's size.
                   Length 0 still needs one NACKed padding byte to finish. */
                if (u8PayloadLen > I2C_BUF_SIZE)
                {
                    g_u8I2CErrFlag = 1;
                    g_u8I2CRxLen = 2;
                }
                else
                {
                    g_u8I2CRxLen = (u8PayloadLen == 0U) ? 2U : (uint8_t)(u8PayloadLen + 1U);
                }
            }

            if ((g_u8I2CRxLen - g_u8I2CRxIdx) > 1)
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);   /* Will NACK the last byte */
            }

            break;

        case 0x58:  /* DATA has been received and NACK has been returned (last byte) */
            g_au8I2CRxBuf[g_u8I2CRxIdx++] = (uint8_t)I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8I2CEndFlag = 1;
            break;

        case 0x38:  /* Arbitration lost: release bus without STOP */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            g_u8I2CErrFlag = 1;
            g_u8I2CEndFlag = 1;
            break;

        case 0x20:  /* SLA+W transmitted, NACK received */
        case 0x30:  /* DATA transmitted, NACK received */
        case 0x48:  /* SLA+R transmitted, NACK received             */
        default:    /* Bus error or other unexpected status         */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8I2CErrFlag = 1;
            g_u8I2CEndFlag = 1;
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Blocking I2C0 Master transaction: write u8TxLen bytes, optionally followed by a repeated START          */
/*  and a read of u8RxLen bytes. Pass u8TxLen = 0 for a pure read, u8RxLen = 0 for a pure write.             */
/*  u8RxLen = I2C_READ_FRAME_SIZE enables the paired Slave's length-prefixed read protocol.                */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t I2C_Master_Transfer(uint8_t u8SlaveAddr, const uint8_t *pu8TxBuf, uint8_t u8TxLen,
                                    uint8_t *pu8RxBuf, uint8_t u8RxLen)
{
    uint32_t u32StartMs, i;

    if ((u8TxLen == 0U) && (u8RxLen == 0U))
    {
        return 0;   /* Nothing to do */
    }

    if ((u8TxLen > I2C_BUF_SIZE) || (u8RxLen > I2C_READ_FRAME_SIZE) ||
        ((u8TxLen > 0U) && (pu8TxBuf == NULL)) ||
        ((u8RxLen > 0U) && (pu8RxBuf == NULL)))
    {
        return -1;
    }

    g_u8I2CSlaveAddr = u8SlaveAddr;
    g_u8I2CTxLen = u8TxLen;
    g_u8I2CRxLen = u8RxLen;
    g_u8LengthPrefixedRead = (uint8_t)(u8RxLen == I2C_READ_FRAME_SIZE);
    g_u8I2CTxIdx = 0;
    g_u8I2CRxIdx = 0;
    g_u8I2CEndFlag = 0;
    g_u8I2CErrFlag = 0;

    for (i = 0; i < u8TxLen; i++)
    {
        g_au8I2CTxBuf[i] = pu8TxBuf[i];
    }

    g_u8TimeoutFlag = 0;
    I2C_EnableTimeout(I2C0, 0);
    s_pfnI2C0Handler = I2C_MasterHandler;
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    u32StartMs = g_u32TickMs;

    while ((g_u8I2CEndFlag == 0U) && (g_u8TimeoutFlag == 0U))
    {
        if ((uint32_t)(g_u32TickMs - u32StartMs) >= I2C_TRANSFER_TIMEOUT_MS)
        {
            g_u8I2CErrFlag = 1;
            break;
        }
    }

    /* STOP is asynchronous. Do not start the next transfer before it completes. */
    while ((I2C0->CTL0 & I2C_CTL0_STO_Msk) != 0U)
    {
        if ((uint32_t)(g_u32TickMs - u32StartMs) >= I2C_TRANSFER_TIMEOUT_MS)
        {
            g_u8I2CErrFlag = 1;
            break;
        }
    }

    NVIC_DisableIRQ(I2C0_IRQn);
    I2C_DisableInt(I2C0);
    I2C_DisableTimeout(I2C0);
    s_pfnI2C0Handler = NULL;

    if ((g_u8TimeoutFlag != 0U) || (g_u8I2CErrFlag != 0U))
    {
        /* Bus stuck - reset the I2C0 IP and re-initialize it */
        SYS_UnlockReg();
        SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
        SYS_LockReg();
        I2C0_Init();
        g_u8I2CErrFlag = 1;
    }
    else
    {
        NVIC_ClearPendingIRQ(I2C0_IRQn);
        I2C_EnableInt(I2C0);
        NVIC_EnableIRQ(I2C0_IRQn);
    }

    if ((g_u8I2CErrFlag == 0U) && (u8RxLen > 0U) && (pu8RxBuf != NULL))
    {
        for (i = 0; i < g_u8I2CRxIdx; i++)
        {
            pu8RxBuf[i] = g_au8I2CRxBuf[i];
        }
    }

    return (g_u8I2CErrFlag != 0U) ? -1 : 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Init System Clock and Multi-function Pins                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);
    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Update System Core Clock */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);
    /* I2C0 pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}

void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);
    /* Enable I2C0 interrupt */
    NVIC_ClearPendingIRQ(I2C0_IRQn);
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/* A complete transparent UART packet maps to exactly one I2C transaction. */
static void Bridge_SendPacket(uint8_t u8Len, uint8_t u8Invalid)
{
    uint8_t u8Status = BRIDGE_STATUS_ERR;

    if ((u8Invalid == 0U) &&
        (I2C_Master_Transfer(g_u8I2CSlaveAddr, g_au8BridgeTxStage, u8Len,
                             NULL, 0) == 0))
    {
        u8Status = BRIDGE_STATUS_OK;
    }

    //UART_SendBytes(&u8Status, 1);
}

/* Slave UART data can arrive long after a write, or with no preceding write.
   Poll independently; never print EE for every empty/offline background poll. */
static void Bridge_PollSlave(void)
{
    uint8_t u8Len;

    if (I2C_Master_Transfer(g_u8I2CSlaveAddr, NULL, 0,
                          g_au8BridgeRxStage, I2C_READ_FRAME_SIZE) != 0)
    {
        g_u32I2CReadErrorCount++;
        return;
    }

    u8Len = g_au8BridgeRxStage[0];

    if ((u8Len > I2C_BUF_SIZE) ||
        (g_u8I2CRxIdx != ((u8Len == 0U) ? 2U : (uint8_t)(u8Len + 1U))))
    {
        g_u32I2CReadErrorCount++;
        return;
    }

    if (u8Len != 0U)
    {
        UART_SendBytes(&g_au8BridgeRxStage[1], u8Len);
        g_u32I2CReadByteCount += u8Len;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Len = 0, u8Invalid = 0;
    uint8_t u8Status = BRIDGE_STATUS_ERR;
    uint32_t u32LastByteMs = 0;
    uint32_t u32LastPollMs = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* No printf on the binary data port. SysTick supplies a real 1 ms clock. */
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        while (1) {}
    }

    /* Init I2C0 as Master only */
    I2C0_Init();

    /* Trigger at one byte; RX timeout also drains any residual FIFO bytes. */
    UART0->FIFO = (UART0->FIFO & ~UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_1BYTE;
    UART_SetTimeoutCnt(UART0, 40);
    UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk |
                          UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

    for (;;)
    {
        if ((uint32_t)(g_u32TickMs - u32LastPollMs) >= I2C_READ_POLL_MS)
        {
            Bridge_PollSlave();
            u32LastPollMs = g_u32TickMs;
        }

        /* Lost/corrupt bytes invalidate framing. Resync only after silence. */
        if (g_u8UartRxError != 0U)
        {
            uint32_t u32Primask = __get_PRIMASK();
            uint8_t u8Recovered = 0;

            __disable_irq();
            g_u16UartRxTail = g_u16UartRxHead;
            u8Len = 0;
            u8Invalid = 0;

            if (((uint32_t)(g_u32TickMs - g_u32LastUartRxMs) >= UART_PACKET_IDLE_MS) &&
                UART_GET_RX_EMPTY(UART0))
            {
                g_u8UartRxError = 0;
                u8Recovered = 1;
            }

            __set_PRIMASK(u32Primask);

            if (u8Recovered != 0U)
            {
                UART_SendBytes(&u8Status, 1);
            }

            continue;
        }

        if (g_u16UartRxTail != g_u16UartRxHead)
        {
            uint16_t u16Tail = g_u16UartRxTail;
            uint32_t u32ByteMs = g_au32UartRxTime[u16Tail];

            if ((u8Len != 0U) &&
                ((uint32_t)(u32ByteMs - u32LastByteMs) >= UART_PACKET_IDLE_MS))
            {
                /* Leave the first byte of the next packet in the queue. */
                Bridge_SendPacket(u8Len, u8Invalid);
                u8Len = 0;
                u8Invalid = 0;
                continue;
            }

            if (u8Len < I2C_BUF_SIZE)
            {
                g_au8BridgeTxStage[u8Len++] = g_au8UartRxBuf[u16Tail];
            }
            else
            {
                /* Keep consuming to the gap, but never send a truncated packet. */
                u8Invalid = 1;
            }

            u32LastByteMs = u32ByteMs;
            g_u16UartRxTail = (uint16_t)((u16Tail + 1U) % UART_RX_BUF_SIZE);
        }
        else if (u8Len != 0U)
        {
            uint32_t u32Primask = __get_PRIMASK();
            uint8_t u8Ready;

            /* Recheck queue and hardware atomically before declaring silence. */
            __disable_irq();
            u8Ready = (uint8_t)((g_u16UartRxTail == g_u16UartRxHead) &&
                               (g_u8UartRxError == 0U) && UART_GET_RX_EMPTY(UART0) &&
                               ((uint32_t)(g_u32TickMs - u32LastByteMs) >= UART_PACKET_IDLE_MS));
            __set_PRIMASK(u32Primask);

            if (u8Ready != 0U)
            {
                Bridge_SendPacket(u8Len, u8Invalid);
                u8Len = 0;
                u8Invalid = 0;
            }
        }
    }
}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
