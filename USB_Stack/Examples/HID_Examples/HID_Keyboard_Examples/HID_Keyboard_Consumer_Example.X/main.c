/**
 * @file main.c
 * @brief Main C file.
 * @author John Izzard
 * @date 2024-11-14
 * 
 * HID Keyboard + Consumer Example.
 * Copyright (C) 2017-2024 John Izzard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * USB uC BOOTLOADER INSTRUCTIONS
 * 
 * 1. SETUP PROJECT
 * Right click on your MPLABX project, and select Properties. 
 * Under XC8 global options, click XC8 linker. In the Option categories dropdown, 
 * select Additional options. In the Codeoffset input, you need to put an 
 * offset of 0x2000. (For PIC16F145X offset is in words, therefore 0x1000).
 * 
 * If you are using the a J Series bootloader:
 * In the Option categories dropdown, select Memory Model. In the ROM ranges 
 * input, you need to put a range starting from the Codeoffset (0x2000) to 1KB from last 
 * byte in flash. e.g. For X7J53, 2000-1FBFF is used. This makes sure your code 
 * isn't placed in the same Flash Page as the Config Words. That area is write 
 * protected.
 * 
 * PIC18FX4J50: 2000-03BFF
 * PIC18FX5J50: 2000-07BFF
 * PIC18FX6J50: 2000-0FBFF
 * PIC18FX6J53: 2000-0FBFF
 * PIC18FX7J53: 2000-1FBFF
 * 
 * 2. DOWNLOAD FROM MPLABX
 * You can get MPLABX to download your code every time you press build. 
 * To set this up, right click on your MPLABX project, and select Properties. 
 * Under Conf: "PROCESSOR", click Building. Check the "Execute this line after 
 * build" box and place in this line of code (use the drive letter or name of 
 * your device depending on OS):
 * 
 * Windows Example: cp ${ImagePath} E:\ 
 *                  **Needs a space following "\".
 * 
 * OSX Example: cp ${ImagePath} /Volumes/PIC18FX7J53
 * 
 * Linux Example: cp ${ImagePath} /media/PIC18FX7J53
 * 
 * 3. START BOOTLOADER
 * If you have previously loaded a program, reset your device or insert the USB 
 * cable whilst holding down the bootloader button. The bootloader LED will 
 * turn on to indicate "bootloader mode" is active. If no program is present, 
 * just insert the USB cable.. Your PIC will now appear as a thumb drive.
 * 
 * 4. READ/ERASE
 * If you've previously loaded a program, PROG_MEM.BIN file will exist on the 
 * drive. You can use this file to view the raw binary of your program using a 
 * hex editor. If you wish to erase your program, just delete this file. After 
 * the erase completes, the bootloader will restart and you can load a new program.
 * 
 * 5. EEPROM READ/WRITE/ERASE
 * For PICs that have EEPROM, a EEPROM.BIN file will also exist on the drive. 
 * This file can be used to view your EEPROM and modify it's values. Open the 
 * file in a hex editor, and modify any values and save the file. You can also 
 * erase all the EEPROM values by deleting this file (the bootloader will restart, 
 * and the file will reappear with blank EEPROM).
 * 
 * 6. DOWNLOAD
 * To program, simply drag and drop your hex file or right click your hex file 
 * and select send to PIC18F25K50 (for example). The bootloader will close and 
 * instantly start running your code. Alternatively, as seen in step two, you 
 * can get MPLABX to download the file automatically after a build.
 * 
 */

#include <xc.h>
#include <stdint.h>
#include "fuses.h"
#include "config.h"
#include "usb.h"
#include "usb_hid.h"
#include "usb_hid_reports.h"
#include "ascii_2_key.h"

// RB6 and RB7 button definitions
#define BUTTON_A_PORT_BIT  6
#define BUTTON_A_PORT      PORTB
#define BUTTON_A_PRESSED   (~BUTTON_A_PORT & (1 << BUTTON_A_PORT_BIT))
#define BUTTON_A_RELEASED  (BUTTON_A_PORT & (1 << BUTTON_A_PORT_BIT))

#define BUTTON_B_PORT_BIT  7
#define BUTTON_B_PORT      PORTB
#define BUTTON_B_PRESSED   (~BUTTON_B_PORT & (1 << BUTTON_B_PORT_BIT))
#define BUTTON_B_RELEASED  (BUTTON_B_PORT & (1 << BUTTON_B_PORT_BIT))

#define BUTTON_A_WAS_PRESSED  (BUTTON_A_PRESSED && m_released_a)
#define BUTTON_A_WAS_RELEASED (BUTTON_A_RELEASED && !m_released_a)
#define BUTTON_B_WAS_PRESSED  (BUTTON_B_PRESSED && m_released_b)
#define BUTTON_B_WAS_RELEASED (BUTTON_B_RELEASED && !m_released_b)

// Status LED on RA5 definitions
#define STATUS_LED_BIT 5
#define STATUS_LED_LAT LATA
#define STATUS_LED_TRIS TRISA
#define STATUS_LED_ON()  STATUS_LED_LAT |= (1 << STATUS_LED_BIT)
#define STATUS_LED_OFF() STATUS_LED_LAT &= ~(1 << STATUS_LED_BIT)
#define STATUS_LED_OUTPUT() STATUS_LED_TRIS &= ~(1 << STATUS_LED_BIT)

static void example_init(void);
#ifdef USE_BOOT_LED
static void flash_led(void);
#endif
static void __interrupt() isr(void);
static void send_key(uint8_t modifier, uint8_t key_code);
static void send_consumer(uint8_t consumer_val);
static void service_reports_to_send(void);

static bool m_released_a = true;
static bool m_released_b = true;
static bool m_send_report0 = false, m_send_report1 = false;

// Sleep mode management
static uint16_t m_idle_counter = 0;  // Incremented in SOF (every 1ms)
#define IDLE_TIMEOUT_MS 30000  // 30 seconds of USB inactivity
static bool m_is_sleeping = false;

void main(void)
{
    example_init();
    #ifdef USE_BOOT_LED
	LED_OFF();
    LED_OUPUT();
    flash_led();
	#endif
    
    usb_init();

    // Initialize status LED (active mode at startup)
    STATUS_LED_ON();

    // Comment out the following for polling method.
    INTCONbits.PEIE = 1;
    USB_INTERRUPT_FLAG = 0;
    USB_INTERRUPT_ENABLE = 1;
    INTCONbits.GIE = 1;
    
    send_key(0,0);
    send_consumer(0x00);
    
    uint8_t i = 0;
    while(1)
    {
        // Uncomment out the following for polling method.
        //usb_tasks();

        if(usb_get_state() != STATE_CONFIGURED) {
            m_idle_counter = 0;
            continue;
        }

        service_reports_to_send();

        bool button_activity = false;

        // Keyboard Example - RB6 button sends "A"
        if(g_hid_sent_report[0] == true && g_hid_report_sent)
        {
            if(BUTTON_A_WAS_PRESSED)
            {
                button_activity = true;
                m_released_a = false;
                ascii_2_key('A');
                send_key(g_key_result.Modifier, g_key_result.KeyCode);
            }
            else if(BUTTON_A_WAS_RELEASED)
            {
                button_activity = true;
                m_released_a = true;
                send_key(0,0);
            }

            // RB7 button sends "B"
            if(BUTTON_B_WAS_PRESSED)
            {
                button_activity = true;
                m_released_b = false;
                ascii_2_key('B');
                send_key(g_key_result.Modifier, g_key_result.KeyCode);
            }
            else if(BUTTON_B_WAS_RELEASED)
            {
                button_activity = true;
                m_released_b = true;
                send_key(0,0);
            }
        }

        // Sleep mode management - based on USB SOF inactivity
        if(button_activity) {
            m_idle_counter = 0;  // Reset on any button activity
            if(m_is_sleeping) {
                m_is_sleeping = false;  // Wake up
            }
        }

        // Update LED based on sleep state
        if(m_is_sleeping) {
            STATUS_LED_OFF();
        } else {
            STATUS_LED_ON();
        }
    }
}

static void example_init(void)
{
    // Oscillator Settings.
    // PIC16F145X.
    #if defined(_PIC14E)
    #if XTAL_USED == NO_XTAL
    OSCCONbits.IRCF = 0xF;
    #endif
    #if XTAL_USED != MHz_12
    OSCCONbits.SPLLMULT = 1;
    #endif
    OSCCONbits.SPLLEN = 1;
    PLL_STARTUP_DELAY();
    #if XTAL_USED == NO_XTAL
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN = 1;
    #endif

    // PIC18FX450, PIC18FX550, and PIC18FX455.
    #elif defined(_18F4450_FAMILY_) || defined(_18F4550_FAMILY_)
    PLL_STARTUP_DELAY();
    
    // PIC18F14K50.
    #elif defined(_18F13K50) || defined(_18F14K50)
    OSCTUNEbits.SPLLEN = 1;
    PLL_STARTUP_DELAY();
    
    // PIC18F2XK50.
    #elif defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    #if XTAL_USED == NO_XTAL
    OSCCONbits.IRCF = 7;
    #endif
    #if (XTAL_USED != MHz_12)
    OSCTUNEbits.SPLLMULT = 1;
    #endif
    OSCCON2bits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #if XTAL_USED == NO_XTAL
    ACTCONbits.ACTSRC = 1;
    ACTCONbits.ACTEN = 1;
    #endif

    // PIC18F2XJ53 and PIC18F4XJ53.
    #elif defined(__J_PART)
    OSCTUNEbits.PLLEN = 1;
    PLL_STARTUP_DELAY();
    #endif

    
    // Configure status LED on RA5
    #if defined(_PIC14E)
    ANSELA &= ~(1 << 5);  // RA5 as digital pin
    #endif
    STATUS_LED_OUTPUT();
    STATUS_LED_OFF();

    // Make boot pin digital.
    #if defined(BUTTON_ANSEL)
    BUTTON_ANSEL &= ~(1<<BUTTON_ANSEL_BIT);
    #elif defined(BUTTON_ANCON)
    BUTTON_ANCON |= (1<<BUTTON_ANCON_BIT);
    #endif


    // Apply pull-up.
    #ifdef BUTTON_WPU
    #if defined(_PIC14E)
    WPUA = 0;
    #if defined(_16F1459)
    WPUB = 0;
    // Enable weak pull-ups for RB6 and RB7 (custom buttons)
    WPUB |= (1 << 6) | (1 << 7);
    #endif
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    OPTION_REGbits.nWPUEN = 0;
    
    #elif defined(_18F4450_FAMILY_) || defined(_18F4550_FAMILY_)
    LATB = 0;
    LATD = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    #if BUTTON_RXPU_REG == INTCON2
    INTCON2 &= 7F;
    #else
    PORTE |= 80;
    #endif
    
    #elif defined(_18F13K50) || defined(_18F14K50)
    WPUA = 0;
    WPUB = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    INTCON2bits.nRABPU = 0;
    
    #elif defined(_18F24K50) || defined(_18F25K50) || defined(_18F45K50)
    WPUB = 0;
    TRISE &= 0x7F;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    INTCON2bits.nRBPU = 0;
    
    #elif defined(_18F24J50) || defined(_18F25J50) || defined(_18F26J50) || defined(_18F26J53) || defined(_18F27J53)
    LATB = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    BUTTON_RXPU_REG &= ~(1 << BUTTON_RXPU_BIT);
    
    #elif defined(_18F44J50) || defined(_18F45J50) || defined(_18F46J50) || defined(_18F46J53) || defined(_18F47J53)
    LATB = 0;
    LATD = 0;
    LATE = 0;
    BUTTON_WPU |= (1 << BUTTON_WPU_BIT);
    BUTTON_RXPU_REG &= ~(1 << BUTTON_RXPU_BIT);
    #endif
    #endif

    // Configure Interrupt-on-Change for buttons RB6, RB7 (wakeup from sleep)
    #if defined(_16F1459)
    IOCBPbits.IOCBP6 = 1;  // IOC on positive edge (button release) RB6
    IOCBPbits.IOCBP7 = 1;  // IOC on positive edge (button release) RB7
    IOCBNbits.IOCBN6 = 1;  // IOC on negative edge (button press) RB6
    IOCBNbits.IOCBN7 = 1;  // IOC on negative edge (button press) RB7
    INTCONbits.IOCIE = 1;  // Enable IOC interrupt
    #endif
}

#ifdef USE_BOOT_LED
static void flash_led(void)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        LED_ON();
        __delay_ms(500);
        LED_OFF();
        __delay_ms(500);
    }
}
#endif

static void service_reports_to_send(void)
{
    if((g_hid_in_report_settings[0].Idle_Count_Overflow || m_send_report0) && g_hid_report_sent)
    {
        m_send_report0 = false;
        hid_send_report(0);
    }
    if((g_hid_in_report_settings[1].Idle_Count_Overflow || m_send_report1) && g_hid_report_sent)
    {
        m_send_report1 = false;
        hid_send_report(1);
    }
}

static void send_key(uint8_t modifier, uint8_t key_code)
{
    g_hid_in_report1.Modifiers = modifier;
    g_hid_in_report1.Keycode   = key_code;
    m_send_report0 = true;
}

static void send_consumer(uint8_t consumer_val)
{
    g_hid_in_report2.Consumer_Byte = consumer_val;
    m_send_report1 = true;
}

void usb_sof(void)
{
    hid_service_sof();

    // Increment idle counter (called every 1ms by USB host)
    if(!m_is_sleeping) {
        if(m_idle_counter < IDLE_TIMEOUT_MS) {
            m_idle_counter++;
        }

        // Enter sleep mode after timeout
        if(m_idle_counter >= IDLE_TIMEOUT_MS && !m_is_sleeping) {
            m_is_sleeping = true;
        }
    }
}

void hid_out(uint8_t report_num)
{
    #ifdef USE_BOOT_LED
    if(g_hid_out_report1.CAPS_LOCK) LED_ON();
    else LED_OFF();
    #endif
    #if PINGPONG_MODE == PINGPONG_1_15 || PINGPONG_MODE == PINGPONG_ALL_EP
    if(HID_EP_OUT_LAST_PPB == ODD) hid_arm_ep_out(HID_BD_OUT_EVEN);
    else hid_arm_ep_out(HID_BD_OUT_ODD);
    #else
    hid_arm_ep_out();
    #endif
}

static void __interrupt() isr(void)
{
    if(USB_INTERRUPT_ENABLE && USB_INTERRUPT_FLAG){
        usb_tasks();
        USB_INTERRUPT_FLAG = 0;
    }

    // IOC interrupt for button wakeup from sleep
    if(INTCONbits.IOCIE && INTCONbits.IOCIF) {
        // Clear IOC flags on PORTB
        if(IOCBFbits.IOCBF6) IOCBFbits.IOCBF6 = 0;
        if(IOCBFbits.IOCBF7) IOCBFbits.IOCBF7 = 0;
        INTCONbits.IOCIF = 0;

        // Wake up from sleep on button press
        if(m_is_sleeping) {
            m_is_sleeping = false;
            m_idle_counter = 0;
        }
    }
}