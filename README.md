# M252_UART_I2C_bridge

Bidirectional UART / I2C bridge using Nuvoton M252 microcontrollers.

## Features

- **Master:** Forwards PC UART commands to I2C and polls for responses.
- **Slave:** Forwards I2C commands to UART and buffers UART responses.
- UART: **115200 baud, 8-N-1**.
- I2C: **100 kHz**, slave address **0x5A (7-bit)**.
- Maximum payload: **64 bytes**.

## Hardware Connections

| Interface | Pin | Connection |
| --- | --- | --- |
| UART0 RX | PB.12 | External UART TX |
| UART0 TX | PB.13 | External UART RX |
| I2C0 SDA | PB.4 | Other board SDA |
| I2C0 SCL | PB.5 | Other board SCL |
| GND | GND | Common ground |

Connect the PC UART to the Master and the target device UART to the Slave.
Provide I2C pull-up resistors and use compatible logic levels.

## Build and Run

1. Open the Keil projects:
   - `SampleCode/StdDriver/I2C_Master/Keil/I2C_Master.uvprojx`
   - `SampleCode/StdDriver/I2C_Slave/Keil/I2C_Slave.uvprojx`
2. Build and program the corresponding boards.
3. Open a serial terminal at **115200, 8-N-1**.
4. Use **HEX/binary mode**, disable automatic CR/LF, and leave at least **20 ms between commands**.

## Write / Read Example

### Write Command

Send four binary bytes from the PC:

```text
49 01 10 A6
```

After 10 ms of UART inactivity, the Master sends:

```text
START → 0x5A + Write → 49 → 01 → 10 → A6 → STOP
```

The Slave forwards `49 01 10 A6` to the target device through UART.

### Read Response

If the target replies with `49 01 1F 97` and all four bytes are queued, the Master reads:

```text
START → 0x5A + Read → 04 → 49 → 01 → 1F → 97 → STOP
```

`04` indicates the payload length. The Master removes this byte, so the PC receives:

```text
49 01 1F 97
```

The Master polls automatically with a 20 ms interval setting.
The target device must generate the response.

## Notes

- I2C writes contain only payload; I2C reads include a one-byte length prefix.
- Fully queued responses of 4 / 7 / 15 / 19 bytes use length values `04` / `07` / `0F` / `13`.
- Responses may span multiple polls. PC software must assemble frames using the target-device protocol.
- Normal write-status replies are disabled; do not wait for `00` after writing.
