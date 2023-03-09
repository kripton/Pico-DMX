
/*
 * Copyright (c) 2021 Jostein Løwer 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DMX_INPUT_H
#define DMX_INPUT_H

#define PIN_DEBUG_INPUT_TIMER_RESET 16
#define PIN_DEBUG_INPUT_BREAK_DETECT_ALARM_FIRED 17
#define PIN_DEBUG_INPUT_END_OF_FRAME_COMMON_HANDLER 19
#define PIN_DEBUG_INPUT_DMA_HANDLER 20
#define PIN_DEBUG_INPUT_DMA_RUNNING 22

#if defined(ARDUINO_ARCH_MBED)
  #include <dma.h>
  #include <pio.h>
#else
  #ifdef ARDUINO
    #include <Arduino.h>
  #endif
  #include "hardware/dma.h"
  #include "hardware/pio.h"
  #include "pico/time.h"
#endif

#define DMX_SM_FREQ 1000000

#define DMXINPUT_BUFFER_SIZE(num_channels) (num_channels+1)
class DmxInput
{
    uint _pin;

public:
    /*
    private properties that are declared public so the interrupt handler has access
    */
    volatile int32_t _num_channels;
    volatile uint8_t *_buf;
    volatile PIO _pio;
    volatile uint _sm;
    volatile uint _dma_chan;
    volatile bool _frame_finished = 0;
    volatile alarm_id_t _alarm_id;      // ID of the BREAK-detect alarm
    volatile uint32_t _channels_captured;
    volatile bool _dma_running;
    void (*_cb)(DmxInput*);

    /*
        All different return codes for the DMX class. Only the SUCCESS
        Return code guarantees that the DMX output instance was properly configured
        and is ready to run
    */
    enum return_code
    {
        SUCCESS = 0,

        // There were no available state machines left in the
        // pio instance.
        ERR_NO_SM_AVAILABLE = -1,

        // There is not enough program memory left in the PIO to fit
        // The DMX PIO program
        ERR_INSUFFICIENT_PRGM_MEM = -2
    };

    /*
       Starts a new DMX input instance. 
       
       Param: pin
       Any valid GPIO pin on the Pico

       Param: pio
       defaults to pio0. pio0 can run up to 3
       DMX input instances. If you really need more, you can
       run 3 more on pio1  
    */

    return_code begin(uint pin, uint start_channel, uint num_channels, PIO pio = pio0, bool inverted = false);

    /*
        Read the selected channels from .begin(...) into a buffer.
        Method call blocks until the selected channels have been received

        Param: buffer
        A pointer to the location where the channels should be received 
        The buffer should have a max length of
        513 bytes (1 byte start code + 512 bytes frame). For ordinary
        DMX data frames, the start code should be 0x00.
    */
    void read(volatile uint8_t *buffer);

    /*
        Start async read process. This should only be called once.
        From then on, the buffer will always contain the latest DMX data.
        If you want to be notified whenever a new DMX frame has been received,
        provide a callback function that will be called without arguments.
    */
    void read_async(volatile uint8_t *buffer, void (*inputUpdatedCallback)(DmxInput* instance) = nullptr);

    /*
        Get the pin this instance is listening on
    */
    uint pin();


    /*
        De-inits the DMX input instance. Releases PIO resources. 
        The instance can safely be destroyed after this method is called
    */
    void end();
};

#endif
