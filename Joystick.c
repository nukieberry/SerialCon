/*
Nintendo Switch Fightstick - Proof-of-

Based on the LUFA library's Low-Level Joystick 
    (C) Dean 
Based on the HORI's Pokken Tournament Pro Pad 
    (C) 

This project implements a modified version of HORI's Pokken Tournament Pro 
USB descriptors to allow for the creation of custom controllers for 
Nintendo Switch. This also works to a limited degree on the PS3.

Since System Update v3.0.0, the Nintendo Switch recognizes the 
Tournament Pro Pad as a Pro Controller. Physical design limitations 
the Pokken Controller from functioning at the same level as the 
Controller. However, by default most of the descriptors are there, with 
exception of Home and Capture. Descriptor modification allows us to 
these buttons for our use.
*/

#include "Joystick.h"

// constants
#define VERSION 0x43
#define ECHO_TIMES 3
#define ECHO_INTERVAL 5
#define LED_DURATION 50
#define SERIAL_BUFFER_SIZE 20
#define DIRECTION_OFFSET 20
#define VARSPACE_OFFSET 90
#define KEYCODE_OFFSET 90
#define KEYCODE_MAX 33
#define REGISTER_OFFSET 130
#define STACK_OFFSET 150
#define CALLSTACK_OFFSET 190
#define FORSTACK_OFFSET 250
#define INS_OFFSET 370

// serial protocal control bytes and replies
#define CMD_READY 0xA5
#define CMD_DEBUG 0x80
#define CMD_HELLO 0x81
#define CMD_FLASH 0x82
#define CMD_SCRIPTSTART 0x83
#define CMD_SCRIPTSTOP 0x84
#define CMD_VERSION 0x85
#define REPLY_ERROR 0x00
#define REPLY_ACK 0xFF
#define REPLY_BUSY 0xFE
#define REPLY_HELLO 0x80
#define REPLY_FLASHSTART 0x81
#define REPLY_FLASHEND 0x82
#define REPLY_SCRIPTACK 0x83

// indexed variables and inline functions
#define Max(a, b) ((a > b) ? (a) : (b))
#define Min(a, b) ((a < b) ? (a) : (b))
#define SERIAL_BUFFER(i) mem[(i)]
#define KEY(keycode) mem[KEYCODE_OFFSET+(keycode)]
#define REG(i) *(int16_t*)(mem+REGISTER_OFFSET+((i)<<1))
#define STACK(i) *(int16_t*)(mem+STACK_OFFSET+((i)<<1))
#define CALLSTACK(i) *(uint16_t*)(mem+CALLSTACK_OFFSET+((i)<<1))
#define DX(i) mem[DIRECTION_OFFSET+((i)<<1)]
#define DY(i) mem[DIRECTION_OFFSET+((i)<<1)+1]
#define FOR_I(i) *(int32_t*)(mem+FORSTACK_OFFSET+(i)*12)
#define FOR_C(i) *(int32_t*)(mem+FORSTACK_OFFSET+(i)*12+4)
#define FOR_ADDR(i) *(uint16_t*)(mem+FORSTACK_OFFSET+(i)*12+8)
#define FOR_NEXT(i) *(uint16_t*)(mem+FORSTACK_OFFSET+(i)*12+10)
#define SETWAIT(time) wait_ms=(time)
#define RESETAFTER(keycode,n) KEY(keycode)=n
#define JUMP(addr) script_addr = (uint8_t*)(addr)
#define JUMPNEAR(addr) script_addr = (uint8_t*)((uint16_t)script_addr + (addr))
#define E(val) _e_set = 1, _e_val = (val)
#define E_SET ((_e_set_t = _e_set),(_e_set = 0),_e_set_t)

// single-byte variables
#define _ins3 mem[INS_OFFSET]
#define _ins2 mem[INS_OFFSET+1]
#define _ins1 mem[INS_OFFSET+2]
#define _ins0 mem[INS_OFFSET+3]
#define _ins *(uint16_t*)(mem+INS_OFFSET+2)
#define _insEx *(uint32_t*)(mem+INS_OFFSET)
#define _code mem[INS_OFFSET+4]
#define _keycode mem[INS_OFFSET+5]
#define _lr mem[INS_OFFSET+6]
#define _direction mem[INS_OFFSET+7]
#define _addr *(uint16_t*)(mem+INS_OFFSET+8)
#define _stackindex mem[INS_OFFSET+10]
#define _callstackindex mem[INS_OFFSET+11]
#define _forstackindex mem[INS_OFFSET+12]
#define _report_echo mem[INS_OFFSET+13]
#define _e_set mem[INS_OFFSET+14]
#define _e_set_t mem[INS_OFFSET+15]
#define _e_val *(uint16_t*)(mem+INS_OFFSET+16)    // external argument for next instruction (used for dynamic for-loop, wait etc.)
#define _script_running mem[INS_OFFSET+18]
#define _ri0 mem[INS_OFFSET+19]
#define _ri1 mem[INS_OFFSET+20]
#define _v mem[INS_OFFSET+21]
#define _flag mem[INS_OFFSET+22]

// All variables other than mem must have default values. Otherwise EasyCon cannot locate the data segment.
uint8_t mem[MEM_SIZE] = {0xFF, 0xFF, VERSION};   // preallocated memory for all purposes, as well as static instruction carrier
size_t serial_buffer_length = 0;                      // current length of serial buffer
bool serial_command_ready = false;             // CMD_READY acknowledged, ready to receive command byte
uint8_t* flash_addr = 0;                                // start location for EEPROM flashing
uint16_t flash_index = 0;                               // current buffer index
uint16_t flash_count = 0;                               // number of bytes expected for this time
uint8_t* script_addr = 0;                               // address of next instruction
uint8_t* script_eof = 0;                                 // address of EOF
uint16_t tail_wait = 0;                                   // insert an extra wait before next instruction (used by compressed instruction)
uint32_t timer_elapsed = 0;                          // previous execution time

// Main entry point.
int main(void)
{
    // We'll start by performing hardware and peripheral setup.
    SetupHardware();
    // We'll then enable global interrupts for our use.
    GlobalInterruptEnable();
    // Once that's done, we'll enter an infinite loop.
    for (;;)
    {
        // We need to run our task to process and deliver data for our IN and OUT endpoints.
        HID_Task();
        // We also need to run the main USB management task.
        USB_USBTask();
        // Manage data from/to serial port.
        Serial_Task();
        // Process local script instructions.
        Script_Task();
    }
}

// Configures hardware and peripherals, such as the USB peripherals.
void SetupHardware(void)
{
    // We need to disable watchdog if enabled by bootloader/fuses.
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    // We need to disable clock division before initializing the USB hardware.
    clock_prescale_set(clock_div_1);
    
    // Initialize report.
    ResetReport();
    
    // Initialize LEDs.
    LEDs_Init();
    
    // Initialize script.
    Script_Init();
    
    // Initialize timer interrupt
    TIMSK0 |= (1 << TOIE0);
    sei();
    //enable interrupts
    TCCR0B |= (1 << CS01) | (1 << CS00);
    // set prescaler to 64 and start the timer
    
    // We can then initialize our hardware and peripherals, including the USB stack.
    
    // The USB stack should be initialized last.
    USB_Init();
    
    // Initialize serial port.
    Serial_Init(9600, false);
    
    // Start script.
    Script_AutoStart();
}

volatile uint32_t timer_ms = 0;        // script timer
volatile uint8_t echo_ms = 0;          // echo counter
volatile uint32_t wait_ms = 0;         // waiting counter
volatile uint8_t led_ms = 0;            // transmission LED countdown
ISR (TIMER0_OVF_vect) // timer0 overflow interrupt
{
    TCNT0 += 6;
    // add 6 to the register (our work around)
    timer_ms++;
    // decrement echo counter
    if (echo_ms != 0)
        echo_ms--;
    // decrement waiting counter
    if (wait_ms != 0 && (_report_echo == 0 || wait_ms > 5))
        wait_ms--;
    // decrement LED counter
    if (led_ms > 0)
    {
        led_ms--;
        if (led_ms == 0)
            LEDs_TurnOffLEDs(LEDMASK_TX);
    }
}

// Fired to indicate that the device is enumerating.
void EVENT_USB_Device_Connect(void)
{
    // We can indicate that we're enumerating here (via status LEDs, sound, etc.).
}

// Fired to indicate that the device is no longer connected to a host.
void EVENT_USB_Device_Disconnect(void)
{
    // We can indicate that our device is not ready (via status LEDs, sound, etc.).
}

// Fired when the host set the current configuration of the USB device after enumeration.
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    // We setup the HID report endpoints.
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_OUT_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_IN_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

    // We can read ConfigSuccess to indicate a success or failure at this point.
}

// Process control requests sent to the device from the USB host.
void EVENT_USB_Device_ControlRequest(void)
{
    // We can handle two control requests: a GetReport and a SetReport.

    // Not used here, it looks like we don't receive control request from the Switch.
}
USB_JoystickReport_Input_t next_report;

// Process and deliver data from IN and OUT endpoints.
void HID_Task(void)
{
    // If the device isn't connected and properly configured, we can't do anything here.
    if (USB_DeviceState != DEVICE_STATE_Configured)
        return;

    // [Optimized] We don't need to receive data at all.
    if (true)
    {
        // We'll start with the OUT endpoint.
        Endpoint_SelectEndpoint(JOYSTICK_OUT_EPADDR);
        // We'll check to see if we received something on the OUT endpoint.
        if (Endpoint_IsOUTReceived())
        {
            // If we did, and the packet has data, we'll react to it.
            if (false && Endpoint_IsReadWriteAllowed())
            {
                // We'll create a place to store our data received from the host.
                USB_JoystickReport_Output_t JoystickOutputData;
                // We'll then take in that data, setting it up in our storage.
                while(Endpoint_Read_Stream_LE(&JoystickOutputData, sizeof(JoystickOutputData), NULL) != ENDPOINT_RWSTREAM_NoError);
                // At this point, we can react to this data.

                // However, since we're not doing anything with this data, we abandon it.
            }
            // Regardless of whether we reacted to the data, we acknowledge an OUT packet on this endpoint.
            Endpoint_ClearOUT();
        }
    }
    
    // [Optimized] Only send data when changed.
    if (echo_ms == 0 && _report_echo)
    {
        // We'll then move on to the IN endpoint.
        Endpoint_SelectEndpoint(JOYSTICK_IN_EPADDR);
        // We first check to see if the host is ready to accept data.
        if (Endpoint_IsINReady())
        {
            // Once populated, we can output this data to the host. We do this by first writing the data to the control stream.
            if(Endpoint_Write_Stream_LE(&next_report, sizeof(next_report), NULL) == ENDPOINT_RWSTREAM_NoError)
            {
                // We then send an IN packet on this endpoint.
                Endpoint_ClearIN();
                // decrement echo counter
                _report_echo--;
                // set interval
                echo_ms = ECHO_INTERVAL;
            }
        }
    }
}

// Reset report to default.
void ResetReport(void)
{
    memset(&next_report, 0, sizeof(USB_JoystickReport_Input_t));
    next_report.LX = STICK_CENTER;
    next_report.LY = STICK_CENTER;
    next_report.RX = STICK_CENTER;
    next_report.RY = STICK_CENTER;
    next_report.HAT = HAT_CENTER;
    _report_echo = ECHO_TIMES;
}

// Process data from serial port.
void Serial_Task(void)
{
    // read all incoming bytes
    while (true)
    {
        // read next byte
        int16_t byte = Serial_ReceiveByte();
        if (byte < 0)
            break;
        BlinkLED();
        if (flash_index < flash_count)
        {
            // flashing
            SERIAL_BUFFER(flash_index) = byte;
            flash_index++;
            if (flash_index == flash_count)
            {
                // all bytes received
                for (flash_index = 0; flash_index < flash_count; flash_index++, flash_addr++)
                    eeprom_write_byte(flash_addr, SERIAL_BUFFER(flash_index));
                Serial_Send(REPLY_FLASHEND);
            }
        }
        else
        {
            // regular
            SERIAL_BUFFER(serial_buffer_length++) = byte;
            // check control byte
            if ((byte & 0x80) != 0)
            {
                if (serial_buffer_length == 1 && !serial_command_ready && byte == CMD_READY)
                {
                    // comand ready
                    serial_command_ready = true;
                }
                else if (serial_buffer_length == 8)
                {
                    // report data
                    if (_script_running)
                    {
                        // script running, send BUSY
                        Serial_Send(REPLY_BUSY);
                    }
                    else
                    {
                        //memset(&next_report, 0, sizeof(USB_JoystickReport_Input_t));
                        next_report.Button = (SERIAL_BUFFER(0) << 9) | (SERIAL_BUFFER(1) << 2) | (SERIAL_BUFFER(2) >> 5);
                        next_report.HAT = (uint8_t)((SERIAL_BUFFER(2) << 3) | (SERIAL_BUFFER(3) >> 4));
                        next_report.LX = (uint8_t)((SERIAL_BUFFER(3) << 4) | (SERIAL_BUFFER(4) >> 3));
                        next_report.LY = (uint8_t)((SERIAL_BUFFER(4) << 5) | (SERIAL_BUFFER(5) >> 2));
                        next_report.RX = (uint8_t)((SERIAL_BUFFER(5) << 6) | (SERIAL_BUFFER(6) >> 1));
                        next_report.RY = (uint8_t)((SERIAL_BUFFER(6) << 7) | (SERIAL_BUFFER(7) & 0x7f));
                        // set flag
                        _report_echo = ECHO_TIMES;
                        // send ACK
                        Serial_Send(REPLY_ACK);
                    }
                    //Serial_Send(next_report.Button >> 8);
                    //Serial_Send(next_report.Button);
                    //Serial_Send(next_report.HAT);
                    //Serial_Send(next_report.LX);
                    //Serial_Send(next_report.LY);
                    //Serial_Send(next_report.RX);
                    //Serial_Send(next_report.RY);
                    serial_command_ready = false;
                }
                else if (serial_command_ready)
                {
                    serial_command_ready = false;
                    // command
                    switch (byte)
                    {
                        case CMD_DEBUG:
                            ;uint32_t n;
                            // instruction count
                            //uint8_t* count = (uint8_t*)(eeprom_read_byte((uint8_t*)0) | (eeprom_read_byte((uint8_t*)1) << 8));
                            //for (uint8_t* i = 0; i < count; i++)
                                //Serial_Send(eeprom_read_byte(i));
                            
                            // current loop variable
                            n = FOR_I(_forstackindex - 1);
                            for (int i = 0; i < 4; i++)
                            {
                                Serial_Send(n);
                                n >>= 8;
                            }
                            
                            // time elapsed
                            n = timer_elapsed;
                            for (int i = 0; i < 4; i++)
                            {
                                Serial_Send(n);
                                n >>= 8;
                            }
                            
                            // PC
                            n = (uint16_t)script_addr;
                            for (int i = 0; i < 2; i++)
                            {
                                Serial_Send(n);
                                n >>= 8;
                            }
                            break;
                        case CMD_VERSION:
                            Serial_Send(VERSION);
                            break;
                        case CMD_READY:
                            serial_command_ready = true;
                            break;
                        case CMD_HELLO:
                            Serial_Send(REPLY_HELLO);
                            break;
                        case CMD_FLASH:
                            if (serial_buffer_length != 5)
                            {
                                Serial_Send(REPLY_ERROR);
                                break;
                            }
                            Script_Stop();
                            flash_addr = (uint8_t*)(SERIAL_BUFFER(0) | (SERIAL_BUFFER(1) << 7));
                            flash_count = (SERIAL_BUFFER(2) | (SERIAL_BUFFER(3) << 7));
                            flash_index = 0;
                            Serial_Send(REPLY_FLASHSTART);
                            break;
                        case CMD_SCRIPTSTART:
                            Script_Start();
                            Serial_Send(REPLY_SCRIPTACK);
                            break;
                        case CMD_SCRIPTSTOP:
                            Script_Stop();
                            Serial_Send(REPLY_SCRIPTACK);
                            break;
                        default:
                            // error
                            Serial_Send(REPLY_ERROR);
                            break;
                    }
                }
                else
                {
                    // error
                    Serial_Send(serial_buffer_length);
                }
                // clear buffer
                serial_buffer_length = 0;
            }
            // overflow protection
            if (serial_buffer_length >= SERIAL_BUFFER_SIZE)
                serial_buffer_length = 0;
        }
    }
}

// Initialize script. Load static script into EEPROM if exists.
void Script_Init(void)
{
    if (mem[0] != 0xFF || mem[1] != 0xFF)
    {
        // flash instructions from firmware
        int len = mem[0] | ((mem[1] & 0b01111111) << 8);
        for (int i = 0; i < len; i++)
            eeprom_write_byte((uint8_t*)i, mem[i]);
    }
    memset(mem, 0, sizeof(mem));
    // calculate direction presets
    for (int i = 0; i < 16; i++)
    {
        // part 1
        int x = (Min(i, 8)) << 5;
        int y = (Max(i, 8) - 8) << 5;
        x = Min(x, 255);
        y = Min(y, 255);
        DX(i) = x;
        DY(i) = y;
    }
    for (int i = 16; i < 32; i++)
    {
        // part 2
        int x = (24 - Min(i, 24)) << 5;
        int y = (32 - Max(i, 24)) << 5;
        x = Min(x, 255);
        y = Min(y, 255);
        DX(i) = x;
        DY(i) = y;
    }
}

// Run script on startup.
void Script_AutoStart(void)
{
    // only if highest bit is 0
    if ((eeprom_read_byte((uint8_t*)1) >> 7) == 0)
        Script_Start();
}

// Run script.
void Script_Start(void)
{
    script_addr = (uint8_t*)2;
    uint16_t eof = eeprom_read_byte((uint8_t*)0) | (eeprom_read_byte((uint8_t*)1) << 8);
    if (eof == 0xFFFF)
        eof = 0;
    script_eof = (uint8_t*)(eof & 0x7FFF);
    // reset variables
    wait_ms = 0;
    echo_ms = 0;
    timer_ms = 0;
    tail_wait = 0;
    memset(mem + VARSPACE_OFFSET, 0, sizeof(mem) - VARSPACE_OFFSET);
    _script_running = 1;
    LEDs_TurnOnLEDs(LEDMASK_RX);
}

// Stop script.
void Script_Stop(void)
{
    _script_running = 0;
    timer_elapsed = timer_ms;
    ResetReport();
    LEDs_TurnOffLEDs(LEDMASK_RX);
}

// Process script instructions.
void Script_Task(void)
{
    while (true)
    {
        // status check
        if (!_script_running)
            return;
        // timer check
        if (wait_ms > 0)
            return;
        // release keys
        for (int i = 0; i <= KEYCODE_MAX; i++)
        {
            if (KEY(i) != 0)
            {
                KEY(i)--;
                if (KEY(i) == 0)
                {
                    if (i == 32)
                    {
                        // LS
                        next_report.LX = STICK_CENTER;
                        next_report.LY = STICK_CENTER;
                        _report_echo = ECHO_TIMES;
                    }
                    else if (i == 33)
                    {
                        // RS
                        next_report.RX = STICK_CENTER;
                        next_report.RY = STICK_CENTER;
                        _report_echo = ECHO_TIMES;
                    }
                    else if ((i & 0x10) == 0)
                    {
                        // Button
                        next_report.Button &= ~(1 << i);
                        _report_echo = ECHO_TIMES;
                    }
                    else
                    {
                        // HAT
                        next_report.HAT = HAT_CENTER;
                        _report_echo = ECHO_TIMES;
                    }
                }
            }
        }
        if (tail_wait != 0)
        {
            // wait after compressed instruction
            SETWAIT(tail_wait);
            tail_wait = 0;
            return;
        }
        if (script_addr >= script_eof)
        {
            // reaches EOF, end script
            Script_Stop();
            return;
        }
        _addr = (uint16_t)script_addr;
        _ins0 = eeprom_read_byte(script_addr++);
        _ins1 = eeprom_read_byte(script_addr++);
        int32_t n;
        int16_t reg;
        if (_ins0 & 0b10000000)
        {
            // key/stick actions
            if ((_ins0 & 0b01000000) == 0)
            {
                // Instruction : Key
                _keycode = (_ins0 >> 1) & 0b11111;
                // modify report
                if ((_keycode & 0x10) == 0)
                {
                    // Button
                    next_report.Button |= 1 << _keycode;
                    _report_echo = ECHO_TIMES;
                }
                else
                {
                    // HAT
                    next_report.HAT = _keycode & 0xF;
                    _report_echo = ECHO_TIMES;
                }
                // post effect
                if (E_SET)
                {
                    // pre-loaded duration
                    SETWAIT(REG(_e_val));
                    RESETAFTER(_keycode, 1);
                }
                else if ((_ins0 & 0b00000001) == 0)
                {
                    // standard
                    n = _ins1;
                    // unscale
                    n *= 10;
                    SETWAIT(n);
                    RESETAFTER(_keycode, 1);
                }
                else if ((_ins1 & 0b10000000) == 0)
                {
                    // compressed
                    tail_wait = _ins1 & 0b01111111;
                    // unscale
                    tail_wait *= 50;
                    SETWAIT(50);
                    RESETAFTER(_keycode, 1);
                }
                else
                {
                    // hold
                    n = _ins1 & 0b01111111;
                    RESETAFTER(_keycode, n);
                }
            }
            else
            {
                // Instruction : Stick
                _lr = (_ins0 >> 5) & 1;
                _keycode = 32 | _lr;
                _direction = _ins0 & 0b11111;
                // modify report
                if (_lr)
                {
                    // RS
                    next_report.RX = DX(_direction);
                    next_report.RY = DY(_direction);
                    _report_echo = ECHO_TIMES;
                }
                else
                {
                    // LS
                    next_report.LX = DX(_direction);
                    next_report.LY = DY(_direction);
                    _report_echo = ECHO_TIMES;
                }
                // post effect
                if (E_SET)
                {
                    // pre-loaded duration
                    SETWAIT(REG(_e_val));
                    RESETAFTER(_keycode, 1);
                }
                else if ((_ins1 & 0b10000000) == 0)
                {
                    // standard
                    n = _ins1 & 0b01111111;
                    // unscale
                    n *= 50;
                    SETWAIT(n);
                    RESETAFTER(_keycode, 1);
                }
                else
                {
                    // hold
                    n = _ins1 & 0b01111111;
                    RESETAFTER(_keycode, n);
                }
            }
        }
        else
        {
            // flow control
            switch ((_ins0 >> 3) & 0b1111)
            {
                case 0b0000:
                    if ((_ins0 & 0b100) == 0)
                    {
                        // empty
                    }
                    else
                    {
                        // Instruction : SerialPrint
                        reg = _ins & ((1 << 9) - 1);
                        if ((_ins0 & 0b10) == 0)
                        {
                            Serial_Send(reg);
                            Serial_Send(reg >> 8);
                        }
                        else
                        {
                            Serial_Send(mem[reg]);
                            Serial_Send(mem[reg+1]);
                        }
                        break;
                    }
                    break;
                case 0b0001:
                    // Instruction : Wait
                    if (E_SET)
                    {
                        // pre-loaded duration
                        n = REG(_e_val);
                    }
                    else if ((_ins0 & 0b100) == 0)
                    {
                        // standard
                        n = _ins & ((1 << 10) - 1);
                        // unscale
                        n *= 10;
                    }
                    else if ((_ins0 & 0b10) == 0)
                    {
                        // extended
                        _ins2 = eeprom_read_byte(script_addr++);
                        _ins3 = eeprom_read_byte(script_addr++);
                        n = _insEx & ((1L << 25) - 1);
                        // unscale
                        n *= 10;
                    }
                    else
                    {
                        // high precision
                        n = _ins & ((1 << 9) - 1);
                    }
                    SETWAIT(n);
                    break;
                case 0b0010:
                    // Instruction : For
                    if (_forstackindex == 0 || FOR_ADDR(_forstackindex - 1) != _addr)
                    {
                        // loop initialize
                        _forstackindex++;
                        FOR_I(_forstackindex - 1) = 0;
                        FOR_ADDR(_forstackindex - 1) = _addr;
                        FOR_NEXT(_forstackindex - 1) = _ins & ((1 << 11) - 1);
                        // pre-loaded arguments
                        if (E_SET)
                        {
                            // iterator
                            _ri0 = REG(_e_val) & 0xF;
                            if (_ri0 != 0)
                            {
                                // store iterator
                                FOR_I(_forstackindex - 1) = _ri0 | 0x80000000;
                            }
                            // count
                            _ri1 = (REG(_e_val) >> 4) & 0xF;
                            if (_ri1 != 0)
                            {
                                // write loop count
                                FOR_C(_forstackindex - 1) = REG(_ri1);
                                // Mode 2 : loop count overwritten
                                E(2);
                                // jump to next (for condition checking)
                                JUMP(FOR_NEXT(_forstackindex - 1));
                                break;
                            }
                        }
                        // Mode 0 : init
                        E(0);
                        // jump to next (for further initialization)
                        JUMP(FOR_NEXT(_forstackindex - 1));
                        break;
                    }
                    break;
                case 0b0011:
                    // Instruction : Next
                    if (_ins0 & 0b100)
                    {
                        // extended
                        _ins2 = eeprom_read_byte(script_addr++);
                        _ins3 = eeprom_read_byte(script_addr++);
                    }
                    if (E_SET)
                    {
                        if (_e_val == 1)
                        {
                            // Mode 1 : break
                            _forstackindex--;
                            break;
                        }
                        else if (_e_val == 0)
                        {
                            // Mode 0 : init
                            if (_e_val == 0)
                            {
                                // initialize loop count
                                if ((_ins0 & 0b100) == 0)
                                {
                                    // small number
                                    FOR_C(_forstackindex - 1) = _ins & ((1 << 10) - 1);
                                    if (FOR_C(_forstackindex - 1) == 0)
                                    {
                                        // infinite loop
                                        FOR_C(_forstackindex - 1) = 0x80000000;
                                    }
                                }
                                else
                                {
                                    // large number
                                    FOR_C(_forstackindex - 1) = _insEx & ((1L << 26) - 1);
                                }
                            }
                        }
                        else if (_e_val == 2)
                        {
                            // Mode 2 : loop count overwritten
                            // do nothing here
                        }
                    }
                    else
                    {
                        // normal loop step
                        if (FOR_I(_forstackindex - 1) & 0x80000000)
                        {
                            // iterator
                            REG(FOR_I(_forstackindex - 1) & 0xF) += 1;
                        }
                        else
                        {
                            // loop variable
                            FOR_I(_forstackindex - 1) += 1;
                        }
                    }
                    // check condition
                    if (FOR_I(_forstackindex - 1) & 0x80000000)
                        n = REG(FOR_I(_forstackindex - 1) & 0xF);
                    else
                        n = FOR_I(_forstackindex - 1);
                    if (FOR_C(_forstackindex - 1) != 0x80000000 && n >= FOR_C(_forstackindex - 1))
                    {
                        // end for
                        _forstackindex--;
                    }
                    else
                    {
                        // jump back
                        JUMP(FOR_ADDR(_forstackindex - 1));
                    }
                    break;
                case 0b0100:
                    if (_ins0 & 0b100)
                    {
                        // comparisons
                        _ri0 = (_ins1 >>3) & 0b111;
                        _ri1 = _ins1 & 0b111;
                        switch (_ins0 & 0b11)
                        {
                            case 0b00:
                                // Instruction : Equal
                                _v = REG(_ri0) == REG(_ri1);
                                break;
                            case 0b01:
                                // Instruction : NotEqual
                                _v = REG(_ri0) != REG(_ri1);
                                break;
                            case 0b10:
                                // Instruction : LessThan
                                _v = REG(_ri0) < REG(_ri1);
                                break;
                            case 0b11:
                                // Instruction : LessOrEqual
                                _v = REG(_ri0) <= REG(_ri1);
                                break;
                        }
                        _v = (bool)_v;
                        _flag = (bool)_flag;
                        switch (_ins1 >> 6)
                        {
                            case 0b00:
                                // assign
                                _flag = _v;
                                break;
                            case 0b01:
                                // and
                                _flag &= _v;
                                break;
                            case 0b10:
                                // or
                                _flag |= _v;
                                break;
                            case 0b11:
                                // xor
                                _flag ^= _v;
                                break;
                        }
                    }
                    else
                    {
                        switch (_ins1 >> 5)
                        {
                            case 0b000:
                                // Instruction : Break
                                if ((_ins1 & 0b10000) && !_flag)
                                    break;
                                _v = _ins1 & 0b1111;
                                _forstackindex -= _v;
                                E(1);
                                JUMP(FOR_NEXT(_forstackindex - 1));
                                break;
                            case 0b001:
                                // Instruction : Continue
                                if ((_ins1 & 0b10000) && !_flag)
                                    break;
                                _v = _ins1 & 0b1111;
                                _forstackindex -= _v;
                                JUMP(FOR_NEXT(_forstackindex - 1));
                                break;
                            case 0b111:
                                // Instruction : Return
                                if ((_ins1 & 0b10000) && !_flag)
                                    break;
                                if (_callstackindex)
                                {
                                    // sub function
                                    // pop return address
                                    JUMP(CALLSTACK(_callstackindex - 1));
                                    _callstackindex--;
                                    break;
                                }
                                else
                                {
                                    // main function
                                    Script_Stop();
                                    break;
                                }
                                break;
                        }
                    }
                    break;
                case 0b0101:
                    if ((_ins0 & 0b100) == 0)
                    {
                        _ri0 = (_ins >> 7) & 0b111;
                        if (_ri0 == 0)
                        {
                            if ((_ins1 & (1 << 6)) == 0)
                            {
                                // binary operations on instant
                                _ins2 = eeprom_read_byte(script_addr++);
                                _ins3 = eeprom_read_byte(script_addr++);
                                _v =  (_ins >> 3) & 0b111;
                                _ri0 = _ins & 0b111;
                                reg = _insEx;
                                BinaryOp(_v, _ri0, reg);
                            }
                            else
                            {
                                // preserved
                            }
                        }
                        else
                        {
                            // Instruction : Mov
                            reg = _ins1 & 0b01111111;
                            // fill sign bit
                            reg <<= 9;
                            reg >>= 9;
                            REG(_ri0) = reg;
                        }
                    }
                    else if ((_ins0 & 0b110) == 0b100)
                    {
                        // binary operations on register
                        _v =  (_ins >> 6) & 0b111;
                        _ri0 = (_ins >> 3) & 0b111;
                        _ri1 = _ins & 0b111;
                        BinaryOp(_v, _ri0, REG(_ri1));
                    }
                    else if ((_ins0 & 0b111) == 0b110)
                    {
                        // bitwise shift
                        _ri0 = (_ins1 >> 4) & 0b111;
                        _v = _ins1 & 0b1111;
                        if (_ri0 == 0)
                            break;
                        if ((_ins1 & 0b10000000) == 0)
                        {
                            // Instruction : ShL
                            REG(_ri0) <<= _v;
                        }
                        else
                        {
                            // Instruction : ShR
                            REG(_ri0) >>= _v;
                        }
                    }
                    else
                    {
                        // unary operations
                        _ri0 = _ins1 & 0b111;
                        switch ((_ins1 >> 3) & 0b1111)
                        {
                            case 0b0010:
                                // Instruction : Negative
                                if (_ri0 == 0)
                                    break;
                                REG(_ri0) = -REG(_ri0);
                                break;
                            case 0b0011:
                                // Instruction : Not
                                if (_ri0 == 0)
                                    break;
                                REG(_ri0) = ~REG(_ri0);
                                break;
                            case 0b0100:
                                // Instruction : Push
                                _stackindex++;
                                STACK(_stackindex - 1) = REG(_ri0);
                                break;
                            case 0b0101:
                                // Instruction : Pop
                                if (_ri0 == 0)
                                    break;
                                REG(_ri0) = STACK(_stackindex - 1);
                                _stackindex--;
                                break;
                            case 0b0111:
                                // Instruction : StoreOp
                                E(_ri0);
                                break;
                            case 0b1000:
                                // Instruction : Bool
                                if (_ri0 == 0)
                                    break;
                                REG(_ri0) = (bool)REG(_ri0);
                                break;
                        }
                    }
                    break;
                case 0b0110:
                    // branches
                    reg = _ins & ((1 << 9) - 1);
                    reg = reg << 7 >> 6;
                    switch ((_ins0 >> 1) & 0b11)
                    {
                        case 0b00:
                            // Instruction : Branch
                            JUMPNEAR(reg);
                            break;
                        case 0b01:
                            // Instruction : BranchTrue
                            if (_flag)
                                JUMPNEAR(reg);
                            break;
                        case 0b10:
                            // Instruction : BranchFalse
                            if (!_flag)
                                JUMPNEAR(reg);
                            break;
                        case 0b11:
                            // Instruction : Call
                            _callstackindex++;
                            STACK(_callstackindex - 1) = (int16_t)script_addr;
                            JUMPNEAR(reg);
                            break;
                    }
                    break;
            }
        }
    }
}

// Perform binary operations by operator code
void BinaryOp(uint8_t op, uint8_t reg, int16_t value)
{
    if (reg == 0)
        return;
    switch (op)
    {
        case 0b000:
            // Mov
            REG(reg) = value;
            break;
        case 0b001:
            // Add
            REG(reg) += value;
            break;
        case 0b010:
            // Mul
            REG(reg) *= value;
            break;
        case 0b011:
            // Div
            REG(reg) /= value;
            break;
        case 0b100:
            // Mod
            REG(reg) %= value;
            break;
        case 0b101:
            // And
            REG(reg) &= value;
            break;
        case 0b110:
            // Or
            REG(reg) |= value;
            break;
        case 0b111:
            // Xor
            REG(reg) ^= value;
            break;
    }
}

void BlinkLED(void)
{
    led_ms = LED_DURATION;
    LEDs_TurnOnLEDs(LEDMASK_TX);
}

void Serial_Send(const char DataByte)
{
    Serial_SendByte(DataByte);
    BlinkLED();
}