/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>

// USB device number.
#define USBFS_DEVICE  (0u)

// Packet size of USBUART
#define     UART_TX_QUEUE_SIZE      (64)
#define     UART_RX_QUEUE_SIZE      (64)

// BULK-IN/OUT parameters
#define     IN_EP                   (0x01u)
#define     OUT_EP                  (0x02u)
#define     MAX_PACKET_SIZE         (0x40u)
#define     TARGET_SIZE             (2052)
#define     OUT_NAKTIME             (100)

// TX Queue buffer for USBUART
uint8       uartTxQueue[UART_TX_QUEUE_SIZE];    // TX Queue buffer
uint8       uartTxCount = 0;                    // Data count in TX Queue
CYBIT       uartZlpRequired = 0;                // ZLP Request Flag
uint8       uartTxReject = 0;                   // TX Rejecting counter

// RX Queue buffer for USBUART
uint8       uartRxQueue[UART_RX_QUEUE_SIZE];    // RX Queue buffer
uint8       uartRxCount = 0;                    // Data count in RX Queue
uint8       uartRxIndex = 0;                    // Read index pointer

// TX Buffer for BULK-IN
uint8       buffer_in[MAX_PACKET_SIZE * 2] = "ABCDEFGIHJKLMNOPQRSTUVWXYZ";

// RX Buffer for BULK-OUT
uint8       buffer_out[MAX_PACKET_SIZE * 2];
uint32      rxCount = 0;
uint32      rxSize = 0;

// Periodically check the USBUART
CY_ISR(int_uartQueue_isr) {
    // TX control
    if ((uartTxCount > 0) || uartZlpRequired) {
        // Put TX buffer data
        if (USBFS_CDCIsReady()) {
            USBFS_PutData(uartTxQueue, uartTxCount);
            uartZlpRequired = (uartTxCount == UART_TX_QUEUE_SIZE);
            uartTxCount = 0;
            uartTxReject = 0;
        } else if (++uartTxReject > 4) {
            // Discard TX buffer
            uartTxCount = 0;
            uartTxReject = 0;
        }
    }
    // RX control
    if (uartRxIndex >= uartRxCount) {
        // Get data to empty RX buffer
        if (USBFS_DataIsReady()) {
            uartRxCount = USBFS_GetAll(uartRxQueue);
            uartRxIndex = 0;
        }
    }
}

static void putch_sub(const int16 ch) {
    for (;;) {                                  // 送信キューが空くまで待つ
        int_uartQueue_Disable();
        if (uartTxCount < UART_TX_QUEUE_SIZE) break;
        int_uartQueue_Enable();
    }
    uartTxQueue[uartTxCount++] = ch;            // 送信キューに一文字入れる
    int_uartQueue_Enable();
}

// USBUARTに一文字送る
void putch(const int16 ch) {
    if (ch == '\n') {
        putch_sub('\r');
    }
    putch_sub(ch);
}

// USBUARTから一文字受け取る
int16 getch(void) {
    int16 ch = -1;
    
    int_uartQueue_Disable();
    if (uartRxIndex < uartRxCount) {            // 受信キューに文字があるか確認
        ch = uartRxQueue[uartRxIndex++];        // 受信キューから一文字取り出す
        if (ch == '\r') {                       // 行末文字の変換処理
            ch = '\n';
        }
    }
    int_uartQueue_Enable();
    return ch;
}

//
//  The host application program is updated to generate packets filled
//  by a sequential number packet by packet.  Following validation
//  algorithm is planned to be added.
//
void bulkOutValidation(uint16 length) {
    static uint8    seqNoExpected = 0;
    uint8           seqNoEstimated;
    char            numBuffer[64];
    uint16          nInvalid = 0;
    uint32          i;

    
    //  1) Estimate the sequence number of this packet.
    //     Specify the estimated sequence number of this packet by a
    //     majority voting of the first three bytes.  If the packet
    //     size is ZERO, the expected sequence number is used.  If
    //     the packet size is one or two, the first byte is used.
    switch (length) {
        case 0:
            seqNoEstimated = seqNoExpected;
            break;
        case 1:
        case 2:
            seqNoEstimated = buffer_out[0];
            break;
        default:
            if (buffer_out[1] == buffer_out[3]) {
                seqNoEstimated = buffer_out[2];
            } else {
                seqNoEstimated = buffer_out[0];
            }
    }

    //  2) Validate the sequence number
    //     Check if the estimated sequence number is the expected
    //     sequence number.  INVALID sequence number is notified by
    //     CDC and UART.
    if (seqNoEstimated != seqNoExpected) {
        putch('#');
        sprintf(numBuffer,
            "%ld - Invalid SEQ# %02x->%02x\r\n",
            rxCount, seqNoExpected, seqNoEstimated
        );
        UART_UartPutString(numBuffer);
    }

    //  3) Validate the packet content
    //     Check if all data in the packet are same as the estimated
    //     sequence number.  If the packet size is ZERO, the packet
    //     is always VALID.  INVALID data count is notified by CDC
    //     and UART.
    nInvalid = 0;
    for (i = 0; i < length; i++) {
        if (buffer_out[i] != seqNoEstimated) {
            nInvalid++;
        }
    }
    if (nInvalid > 0) {
        putch('C');
        sprintf(numBuffer,
            "%ld - SEQ# %02x Corrupted %d/%d\r\n",
            rxCount, seqNoEstimated, nInvalid, length
        );
        UART_UartPutString(numBuffer);
    }

    //  4) Update the expected sequence number
    //     Revise the expected sequence number as the estimated
    //     sequence number plus one for the validation of the next
    //     packet.
    seqNoExpected = seqNoEstimated + 1;
}

void bulkOutDispatch(void) {
    char    numBuffer[64];
    uint16  length;

    if (USBFS_GetEPState(OUT_EP) & USBFS_OUT_BUFFER_FULL) {
        // Respond with NAK for a while
        CyDelayUs(OUT_NAKTIME);
        
        // Read received bytes count
        length = USBFS_GetEPCount(OUT_EP);

        // Unload the OUT buffer
        USBFS_ReadOutEP(OUT_EP, &buffer_out[0], length);
        
        // Validate the received packet
        bulkOutValidation(length);
        
        if (length > MAX_PACKET_SIZE) {
            // Illegal packet size
            putch('B');
            sprintf(numBuffer, "%ld - Too big %dB\r\n", rxCount, length);
            UART_UartPutString(numBuffer);
            rxSize = 0;
            rxCount++;
        } else if (length == MAX_PACKET_SIZE) {
            // Max size packet
            rxSize += length;
        } else {
            // Short packet
            rxSize += length;
            if (rxSize != TARGET_SIZE) {
                putch('*');
                sprintf(numBuffer, "%ld - Invalid Size %ldB\r\n", rxCount, rxSize);
                UART_UartPutString(numBuffer);
            }
            rxSize = 0;
            rxCount++;
        }
    }
}

void echoBackDispatch(void) {
    int16   ch;

    ch = getch();
    if (ch >= 0) {
        putch(ch);
        if (ch == '\n') {
            putch('*');
        }
    }
}

int main()
{

    CyGlobalIntEnable; /* Enable global interrupts. */
    
    UART_Start();

    // Start USBFS operation with 5V operation.
    USBFS_Start(USBFS_DEVICE, USBFS_5V_OPERATION);
    
    int_uartQueue_StartEx(int_uartQueue_isr);
    
    for(;;) {
        // Wait for Device to enumerate
        while (USBFS_GetConfiguration() == 0);
        UART_UartPutString("C\r\n");

        // Drop CHANGE flag
        USBFS_IsConfigurationChanged();
        USBFS_CDC_Init();

        // Enable OUT endpoint for receive data from Host */
        USBFS_EnableOutEP(OUT_EP);
        
        rxSize = 0;

        for (;;) {
            // Check configuration changes
            if (USBFS_IsConfigurationChanged()) {
                UART_UartPutString("c\r\n");
                break;
            }

            // BULK-OUT : Check for OUT data received
            bulkOutDispatch();
            
            // BULK-IN : Check for IN buffer is empty
            if (USBFS_GetEPState(IN_EP) & USBFS_IN_BUFFER_EMPTY) {
                // Load the IN buffer
                buffer_in[0]++;
                USBFS_LoadInEP(IN_EP, &buffer_in[0], MAX_PACKET_SIZE);
                UART_UartPutString("p\r\n");
            }
            
            // CDC-OUT : Check for input data from host.
            echoBackDispatch();
            
            // CDC-Control : Drop Line Change events.
            (void)USBFS_IsLineChanged();
        }
        
        // UNCONFIGURED
    }
}

/* [] END OF FILE */
