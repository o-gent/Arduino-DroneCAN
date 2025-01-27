#include "Arduino.h"
#include "can.h"

#define DEBUG 0
#define AF3 0x03
#define AF8 0x08
#define AF9 0x09
#define AF10 0x0A

CAN_bit_timing_config_t can_configs[6] = {{2, 13, 100}, {2, 13, 50}, {2, 13, 40}, {2, 13, 20}, {2, 13, 10}, {1, 8, 8}};

/**
 * Print registers.
 */
void printRegister(char *buf, uint32_t reg)
{
    if (DEBUG == 0)
        return;
    Serial.print(buf);
    Serial.print("0x");
    Serial.print(reg, HEX);
    Serial.println();
}

/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: afry    - Specified Alternative function selection AF0-AF15.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3)
{
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7)
    {
        _index4 = (index - 8) * 4;
        ofs = 1;
    }

    uint32_t mask;
    // printRegister("GPIO_AFR(b)=", addr->AFR[1]);
    mask = 0xF << _index4;
    addr->AFR[ofs] &= ~mask; // Reset alternate function
    // setting = 0x9;                    // AF9
    setting = afry; // Alternative function selection
    mask = setting << _index4;
    addr->AFR[ofs] |= mask; // Set alternate function
    // printRegister("GPIO_AFR(a)=", addr->AFR[1]);

    // printRegister("GPIO_MODER(b)=", addr->MODER);
    mask = 0x3 << _index2;
    addr->MODER &= ~mask; // Reset mode
    setting = 0x2;        // Alternate function mode
    mask = setting << _index2;
    addr->MODER |= mask; // Set mode
    // printRegister("GPIO_MODER(a)=", addr->MODER);

    // printRegister("GPIO_OSPEEDR(b)=", addr->OSPEEDR);
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask; // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask; // Set speed
    // printRegister("GPIO_OSPEEDR(a)=", addr->OSPEEDR);

    // printRegister("GPIO_OTYPER(b)=", addr->OTYPER);
    mask = 0x1 << index;
    addr->OTYPER &= ~mask; // Reset Output push-pull
    // printRegister("GPIO_OTYPER(a)=", addr->OTYPER);

    // printRegister("GPIO_PUPDR(b)=", addr->PUPDR);
    mask = 0x3 << _index2;
    addr->PUPDR &= ~mask; // Reset port pull-up/pull-down
    // printRegister("GPIO_PUPDR(a)=", addr->PUPDR);
}

/**
 * Initializes the CAN filter registers.
 *
 * The bxCAN provides up to 14 scalable/configurable identifier filter banks, for selecting the incoming messages, that the software needs and discarding the others.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index > 13)
        return;

    CAN1->FA1R &= ~(0x1UL << index); // Deactivate filter

    if (scale == 0)
    {
        CAN1->FS1R &= ~(0x1UL << index); // Set filter to Dual 16-bit scale configuration
    }
    else
    {
        CAN1->FS1R |= (0x1UL << index); // Set filter to single 32 bit configuration
    }
    if (mode == 0)
    {
        CAN1->FM1R &= ~(0x1UL << index); // Set filter to Mask mode
    }
    else
    {
        CAN1->FM1R |= (0x1UL << index); // Set filter to List mode
    }

    if (fifo == 0)
    {
        CAN1->FFA1R &= ~(0x1UL << index); // Set filter assigned to FIFO 0
    }
    else
    {
        CAN1->FFA1R |= (0x1UL << index); // Set filter assigned to FIFO 1
    }

    CAN1->sFilterRegister[index].FR1 = bank1; // Set filter bank registers1
    CAN1->sFilterRegister[index].FR2 = bank2; // Set filter bank registers2

    CAN1->FA1R |= (0x1UL << index); // Activate filter
}

/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 * @params: remap   - Select CAN port.
 *                    =0:CAN_RX mapped to PA11, CAN_TX mapped to PA12
 *                    =1:CAN_RX mapped to PB5 , CAN_TX mapped to PB6
 *                    =2:CAN_RX mapped to PB8 , CAN_TX mapped to PB9
 *                    =3:CAN_RX mapped to PB12, CAN_TX mapped to PB13
 *
 */
bool CANInit(BITRATE bitrate, int remap)
{
    // Reference manual
    // https://www.st.com/content/ccc/resource/technical/document/reference_manual/4a/19/6e/18/9d/92/43/32/DM00043574.pdf/files/DM00043574.pdf/jcr:content/translations/en.DM00043574.pdf

    RCC->APB1ENR1 |= 0x2000000UL; // Enable CAN clock

    if (remap == 0)
    {
        RCC->AHB2ENR |= 0x1UL;      // Enable GPIOA clock
        CANSetGpio(GPIOA, 11, AF9); // Set PA11 to AF9
        CANSetGpio(GPIOA, 12, AF9); // Set PA12 to AF9
    }

    if (remap == 1)
    {
        RCC->AHB2ENR |= 0x2UL;     // Enable GPIOB clock
        CANSetGpio(GPIOB, 5, AF3); // Set PB5 to AF3
        CANSetGpio(GPIOB, 6, AF8); // Set PB6 to AF8
    }

    if (remap == 2)
    {
        RCC->AHB2ENR |= 0x2UL;     // Enable GPIOB clock
        CANSetGpio(GPIOB, 8, AF9); // Set PB8 to AF9
        CANSetGpio(GPIOB, 9, AF9); // Set PB9 to AF9
    }

    if (remap == 3)
    {
        RCC->AHB2ENR |= 0x2UL;       // Enable GPIOB clock
        CANSetGpio(GPIOB, 12, AF10); // Set PB12 to AF10
        CANSetGpio(GPIOB, 13, AF10); // Set PB13 to AF10
    }

    CAN1->MCR |= 0x1UL; // Set CAN to Initialization mode
    while (!(CAN1->MSR & 0x1UL))
        ; // Wait for Initialization mode

    // CAN1->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
    CAN1->MCR = 0x41UL; // Hardware initialization(With automatic retransmission)

    // Set bit rates
    // CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
    // CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF));
    CAN1->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) | ((can_configs[bitrate].BRP - 1) & 0x3FF);
    //   printRegister("CAN1->BTR=", CAN1->BTR);

    // Configure Filters to default values
    CAN1->FMR |= 0x1UL; // Set to filter initialization mode

    // Set fileter 0
    // Single 32-bit scale configuration
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0
    // Filter bank register to all 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    CAN1->FMR &= ~(0x1UL); // Deactivate initialization mode

    uint16_t TimeoutMilliseconds = 1000;
    bool can1 = false;
    CAN1->MCR &= ~(0x1UL); // Require CAN1 to normal mode

    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        if ((CAN1->MSR & 0x1UL) == 0)
        {
            can1 = true;
            break;
        }
        delayMicroseconds(1000);
    }
    // Serial.print("can1=");
    // Serial.println(can1);
    if (can1)
    {
        Serial.println("CAN1 initialize ok");
    }
    else
    {
        Serial.println("CAN1 initialize fail!!");
        return false;
    }
    return true;
}

#define STM32_CAN_TIR_TXRQ (1U << 0U) // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR (1U << 1)   // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE (1U << 2)   // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_STD_ID_MASK 0x000007FFU

uint8_t dlcToDataLength(uint8_t dlc)
{
    /*
    Data Length Code      9  10  11  12  13  14  15
    Number of data bytes 12  16  20  24  32  48  64
    */
    if (dlc <= 8)
    {
        return dlc;
    }
    else if (dlc == 9)
    {
        return 12;
    }
    else if (dlc == 10)
    {
        return 16;
    }
    else if (dlc == 11)
    {
        return 20;
    }
    else if (dlc == 12)
    {
        return 24;
    }
    else if (dlc == 13)
    {
        return 32;
    }
    else if (dlc == 14)
    {
        return 48;
    }
    return 64;
}

/**
 * Decodes CAN messages from the data registers and populates a
 * CAN message struct with the data fields.
 *
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 *
 */
void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    uint32_t id = CAN1->sFIFOMailBox[0].RIR;

    // if ((id & STM32_CAN_RIR_IDE) == 0)
    // { // Standard frame format
    //     CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21));
    // }
    // else
    // { // Extended frame format
    CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3));
    CAN_rx_msg->id |= 1U << 31; // https://github.com/ArduPilot/ardupilot/blob/4d31a7320a1d2c38e2d742ae63c34f914febaa8f/libraries/AP_HAL_ChibiOS/CanIface.cpp#L570
    // }

    CAN_rx_msg->data_len = dlcToDataLength((CAN1->sFIFOMailBox[0].RDTR) & 0xFUL);

    CAN_rx_msg->data[0] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 0));
    CAN_rx_msg->data[1] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8));
    CAN_rx_msg->data[2] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16));
    CAN_rx_msg->data[3] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24));
    CAN_rx_msg->data[4] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 0));
    CAN_rx_msg->data[5] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8));
    CAN_rx_msg->data[6] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16));
    CAN_rx_msg->data[7] = uint8_t(0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24));

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    CAN1->RF0R |= 0x20UL;
}

/**
 * Encodes CAN messages using the CAN message struct and populates the
 * data registers with the sent.
 *
 * @params CAN_tx_msg - CAN message structure for transmission
 *
 */
void CANSend(const CanardCANFrame *CAN_tx_msg)
{
    volatile int count = 0;

    uint32_t out = 0;
    // if ((CAN_tx_msg->id & STM32_CAN_RIR_IDE) == 0)
    // {
    //     // standard frame format
    //     out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
    // }
    // else
    // {
    // extended frame format
    // force extended frame format
    out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
    // }

    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->data_len & 0xFUL;

    CAN1->sTxMailBox[0].TDLR = (((uint32_t)CAN_tx_msg->data[3] << 24) |
                                ((uint32_t)CAN_tx_msg->data[2] << 16) |
                                ((uint32_t)CAN_tx_msg->data[1] << 8) |
                                ((uint32_t)CAN_tx_msg->data[0]));
    CAN1->sTxMailBox[0].TDHR = (((uint32_t)CAN_tx_msg->data[7] << 24) |
                                ((uint32_t)CAN_tx_msg->data[6] << 16) |
                                ((uint32_t)CAN_tx_msg->data[5] << 8) |
                                ((uint32_t)CAN_tx_msg->data[4]));

    // Send Go
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    // Wait until the mailbox is empty
    while (CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000)
        ;

    // The mailbox don't becomes empty while loop
    if (CAN1->sTxMailBox[0].TIR & 0x1UL)
    {
        Serial.println("Send Fail");
        Serial.println(CAN1->ESR);
        Serial.println(CAN1->MSR);
        Serial.println(CAN1->TSR);
    }
}

/**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
uint8_t CANMsgAvail(void)
{
    // Check for pending FIFO 0 messages
    return CAN1->RF0R & 0x3UL;
}