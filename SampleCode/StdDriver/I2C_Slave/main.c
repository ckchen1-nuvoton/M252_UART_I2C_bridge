/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           I2C <-> UART bidirectional bridge.
 *           I2C0 works as a Slave (address 0x5A). Bytes written by an external
 *           I2C master are forwarded out through UART0, and bytes received on
 *           UART0 are returned to the I2C master on the next I2C read.
 *           All data is passed through as raw/HEX bytes (no ASCII conversion).
 *           I2C read format: one length byte (0..64), followed by that many
 *           UART payload bytes. The Master consumes this header internally.
 *           An empty queue returns length 0, then padding 0 if clocked again.
 *           Bytes arriving after the length snapshot wait for the next read.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/* I2C slave address of this bridge (7-bit) */
#define I2C_SLAVE_ADDR      0x5A

/* Ring buffer size used to bridge data between I2C and UART, must be a power of 2 */
#define BRIDGE_BUF_SIZE      256
#define BRIDGE_BUF_MASK      (BRIDGE_BUF_SIZE - 1)
#define BRIDGE_READ_MAX      64u

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* I2C(master write) -> UART(Tx) queue: raw/HEX bytes written by the external I2C master,
   forwarded out through UART0 as-is */
static volatile uint8_t  g_au8I2cToUartBuf[BRIDGE_BUF_SIZE];
static volatile uint16_t g_u16I2cToUartHead = 0;
static volatile uint16_t g_u16I2cToUartTail = 0;

/* UART(Rx) -> I2C(master read) queue: raw/HEX bytes received on UART0,
   returned to the external I2C master on the next I2C read */
static volatile uint8_t  g_au8UartToI2cBuf[BRIDGE_BUF_SIZE];
static volatile uint16_t g_u16UartToI2cHead = 0;
static volatile uint16_t g_u16UartToI2cTail = 0;
static volatile uint8_t g_u8ReadRemaining = 0;
static volatile uint8_t g_u8ReadPayloadLoaded = 0;
volatile uint32_t g_u32UartRxErrorCount = 0;
volatile uint32_t g_u32UartRxDropCount = 0;
volatile uint32_t g_u32I2CErrorCount = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_pfnI2C0Handler = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  Ring buffer helpers used to bridge data between the I2C and UART interrupt handlers                    */
/*---------------------------------------------------------------------------------------------------------*/
static void BridgeBufPush(volatile uint8_t *pu8Buf, volatile uint16_t *pu16Head, volatile uint16_t *pu16Tail, uint8_t u8Data)
{
    uint16_t u16Next = (uint16_t)((*pu16Head + 1) & BRIDGE_BUF_MASK);

    if (u16Next != *pu16Tail)   /* silently drop the byte if the queue is full */
    {
        pu8Buf[*pu16Head] = u8Data;
        *pu16Head = u16Next;
    }
}

static int32_t BridgeBufPop(volatile uint8_t *pu8Buf, volatile uint16_t *pu16Head, volatile uint16_t *pu16Tail, uint8_t *pu8Data)
{
    if (*pu16Tail == *pu16Head)   /* empty */
    {
        return -1;
    }

    *pu8Data = pu8Buf[*pu16Tail];
    *pu16Tail = (uint16_t)((*pu16Tail + 1) & BRIDGE_BUF_MASK);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART0 IRQ Handler - receives raw/HEX bytes from UART0 and queues them for the I2C master to read        */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntSts = UART0->INTSTS;
    uint32_t u32Errors = UART0->FIFOSTS & (UART_FIFOSTS_RXOVIF_Msk |
                         UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_FEF_Msk);

    if (u32Errors != 0U)
    {
        UART0->FIFOSTS = u32Errors;
        g_u32UartRxErrorCount++;
    }

    if (u32IntSts & (UART_INTSTS_RDAIF_Msk | UART_INTSTS_RXTOIF_Msk))
    {
        /* Read all available bytes out of the RX FIFO and queue them for the I2C side */
        while (UART_GET_RX_EMPTY(UART0) == 0)
        {
            uint8_t u8InChar = (uint8_t)UART_READ(UART0);
            uint16_t u16Next = (uint16_t)((g_u16UartToI2cHead + 1U) & BRIDGE_BUF_MASK);

            if (u16Next == g_u16UartToI2cTail)
            {
                g_u32UartRxDropCount++;
            }
            else
            {
                g_au8UartToI2cBuf[g_u16UartToI2cHead] = u8InChar;
                g_u16UartToI2cHead = u16Next;
            }
        }
    }
}

/* Commit only a payload byte that has actually finished transmission.
   Merely loading DAT must not discard a byte if the Master stops early. */
static void SlaveRead_CommitByte(void)
{
    if (g_u8ReadPayloadLoaded != 0U)
    {
        g_u16UartToI2cTail = (uint16_t)((g_u16UartToI2cTail + 1U) & BRIDGE_BUF_MASK);
        g_u8ReadRemaining--;
        g_u8ReadPayloadLoaded = 0;
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
/*  I2C0 Slave TRx Callback Function - bridges I2C <-> UART (raw/HEX bytes, bidirectional)                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;

    if (u32Status == 0x60)                      /* Own SLA+W has been received; ACK has been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously addressed with own SLA address;
                                                   Data has been received; ACK has been returned */
    {
        /* Byte written by the I2C master -> queue it to be forwarded out through UART0 */
        u8Data = (uint8_t)I2C_GET_DATA(I2C0);
        BridgeBufPush(g_au8I2cToUartBuf, &g_u16I2cToUartHead, &g_u16I2cToUartTail, u8Data);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been received; ACK has been returned */
    {
        uint16_t u16Count = (uint16_t)((g_u16UartToI2cHead - g_u16UartToI2cTail) & BRIDGE_BUF_MASK);

        if (u16Count > BRIDGE_READ_MAX)
        {
            u16Count = BRIDGE_READ_MAX;
        }

        g_u8ReadRemaining = (uint8_t)u16Count;
        g_u8ReadPayloadLoaded = 0;
        I2C_SET_DATA(I2C0, g_u8ReadRemaining);  /* First byte is length, NOT payload. */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)                 /* Data byte in I2CDAT has been transmitted; ACK has been received */
    {
        SlaveRead_CommitByte();
        u8Data = 0;                            /* Padding is never removed from the queue. */

        if (g_u8ReadRemaining != 0U)
        {
            u8Data = g_au8UartToI2cBuf[g_u16UartToI2cTail];
            g_u8ReadPayloadLoaded = 1;
        }

        I2C_SET_DATA(I2C0, u8Data);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        /* A Master NACK on the final byte is normal completion, not failure. */
        SlaveRead_CommitByte();
        g_u8ReadRemaining = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8ReadRemaining = 0;
        g_u8ReadPayloadLoaded = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        g_u32I2CErrorCount++;
        g_u8ReadRemaining = 0;
        g_u8ReadPayloadLoaded = 0;

        if (u32Status == 0x68)              /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else if (u32Status == 0xB0)         /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
    }
}

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
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Update System Core Clock */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();
    /* Set I2C0 multi-function pins: PB.4 = SDA, PB.5 = SCL */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);
    /* I2C pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}


void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);
    /* Set I2C0 Slave Address (7-bit), no address mask so only the exact address is matched */
    I2C_SetSlaveAddr(I2C0, 0, I2C_SLAVE_ADDR, I2C_GCMODE_DISABLE);
    I2C_SetSlaveAddrMask(I2C0, 0, 0x0);
    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Data;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
    /* Enable UART0 RX interrupts so bytes sent from the PC are queued for the I2C side */
    UART_SetTimeoutCnt(UART0, 40);
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk |
                           UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk));
    NVIC_EnableIRQ(UART0_IRQn);

    /* Set callback before enabling the I2C0 interrupt and slave address ACK. */
    s_pfnI2C0Handler = I2C_SlaveTRx;
    /* Init I2C0 as Slave, address 0x5A */
    I2C0_Init();
    /* I2C0 enter non address SLV mode, ready to be addressed by an external master */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    while (1)
    {
        /* Forward bytes received from the I2C master out through UART0 (raw/HEX bytes) */
        while (UART_IS_TX_FULL(UART0) == 0)
        {
            if (BridgeBufPop(g_au8I2cToUartBuf, &g_u16I2cToUartHead, &g_u16I2cToUartTail, &u8Data) != 0)
            {
                break;
            }

            UART_WRITE(UART0, u8Data);
        }
    }
}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/

