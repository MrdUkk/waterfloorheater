#include "OneWireHub.h"
#include "OneWireItem.h"

#include "platform.h"

OneWireHub::OneWireHub(const uint8_t pin)
{
    _error = Error::NO_ERROR;

    slave_list = nullptr;

    // prepare pin
    pin_bitMask = PIN_TO_BITMASK(pin);
    pin_baseReg = PIN_TO_BASEREG(pin);
    pinMode(pin, INPUT); // first port-access should by done by this FN, does more than DIRECT_MODE_....
    DIRECT_WRITE_LOW(pin_baseReg, pin_bitMask);

    static_assert(VALUE_IPL, "Your architecture has not been calibrated yet, please run examples/debug/calibrate_by_bus_timing and report instructions per loop (IPL) to https://github.com/orgua/OneWireHub");
    static_assert(ONEWIRE_TIME_VALUE_MIN>2,"YOUR ARCHITECTURE IS TOO SLOW, THIS MAY RESULT IN TIMING-PROBLEMS"); // it could work though, never tested
}


// attach a sensor to the hub
uint8_t OneWireHub::attach(OneWireItem &sensor)
{
    slave_list = &sensor;
    return 0;
}

bool OneWireHub::poll(void)
{
    _error = Error::NO_ERROR;

    while (true)
    {
        //Once reset is done, go to next step
        if (checkReset())           return false;

        // Reset is complete, tell the master we are present
        if (showPresence())         return false;

        //Now that the master should know we are here, we will get a command from the master
        if (recvAndProcessCmd())    return false;

        // on total success we want to start again, because the next reset could only be ~125 us away
    }
}

bool OneWireHub::checkReset(void) // there is a specific high-time needed before a reset may occur -->  >120us
{
    static_assert(ONEWIRE_TIME_RESET_MIN > (ONEWIRE_TIME_SLOT_MAX + ONEWIRE_TIME_READ_MAX), "Timings are wrong"); // last number should read: max(ONEWIRE_TIME_WRITE_ZERO,ONEWIRE_TIME_READ_MAX)
    static_assert(ONEWIRE_TIME_READ_MAX > ONEWIRE_TIME_WRITE_ZERO, "switch ONEWIRE_TIME_WRITE_ZERO with ONEWIRE_TIME_READ_MAX in checkReset(), because it is bigger (worst case)");
    static_assert(ONEWIRE_TIME_RESET_MAX > ONEWIRE_TIME_RESET_MIN, "Timings are wrong");

    DIRECT_MODE_INPUT(pin_baseReg, pin_bitMask);

    // is entered if there are two resets within a given time (timeslot-detection can issue this skip)
    if (_error == Error::RESET_IN_PROGRESS)
    {
        _error = Error::NO_ERROR;
        if (waitLoopsWhilePinIs(ONEWIRE_TIME_RESET_MIN - ONEWIRE_TIME_SLOT_MAX - ONEWIRE_TIME_READ_MAX, false) == 0) // last number should read: max(ONEWIRE_TIME_WRITE_ZERO,ONEWIRE_TIME_READ_MAX)
        {
            waitLoopsWhilePinIs(ONEWIRE_TIME_RESET_MAX, false); // showPresence() wants to start at high, so wait for it
            return false;
        }
    }

    if (!DIRECT_READ(pin_baseReg, pin_bitMask)) return true; // just leave if pin is Low, don't bother to wait, TODO: really needed?

    // wait for the bus to become low (master-controlled), since we are polling we don't know for how long it was zero
    if (waitLoopsWhilePinIs(ONEWIRE_TIME_RESET_TIMEOUT, true) == 0)
    {
        //_error = Error::WAIT_RESET_TIMEOUT;
        return true;
    }

    const timeOW_t loops_remaining = waitLoopsWhilePinIs(ONEWIRE_TIME_RESET_MAX, false);

    // wait for bus-release by master
    if (loops_remaining == 0)
    {
        _error = Error::VERY_LONG_RESET;
        return true;
    }

    // If the master pulled low for to short this will trigger an error
    //if (loops_remaining > (ONEWIRE_TIME_RESET_MAX[0] - ONEWIRE_TIME_RESET_MIN[od_mode])) _error = Error::VERY_SHORT_RESET; // could be activated again, like the error above, errorhandling is mature enough now

    return (loops_remaining > (ONEWIRE_TIME_RESET_MAX - ONEWIRE_TIME_RESET_MIN));
}

bool OneWireHub::showPresence(void)
{
    static_assert(ONEWIRE_TIME_PRESENCE_MAX > ONEWIRE_TIME_PRESENCE_MIN, "Timings are wrong");

    // Master will delay it's "Presence" check (bus-read)  after the reset
    waitLoopsWhilePinIs(ONEWIRE_TIME_PRESENCE_TIMEOUT, true); // no pinCheck demanded, but this additional check can cut waitTime

    // pull the bus low and hold it some time
    DIRECT_WRITE_LOW(pin_baseReg, pin_bitMask);
    DIRECT_MODE_OUTPUT(pin_baseReg, pin_bitMask);    // drive output low

    wait(ONEWIRE_TIME_PRESENCE_MIN); // stays till the end, because it drives the bus low itself

    DIRECT_MODE_INPUT(pin_baseReg, pin_bitMask);     // allow it to float

    // When the master or other slaves release the bus within a given time everything is fine
    if (waitLoopsWhilePinIs((ONEWIRE_TIME_PRESENCE_MAX - ONEWIRE_TIME_PRESENCE_MIN), false) == 0)
    {
        _error = Error::PRESENCE_LOW_ON_LINE;
        return true;
    }

    return false;
}

// note: this FN calls sendBit() & recvBit() but doesn't handle interrupts -> calling FN must do this
void OneWireHub::searchIDTree(void)
{
    uint8_t position_IDBit  = 0;

    while (position_IDBit < 64)
    {
        const uint8_t pos_byte = (position_IDBit >> 3);
        const uint8_t mask_bit = (static_cast<uint8_t>(1) << (position_IDBit & (7)));
        bool bit_send;

        if ((slave_list->ID[pos_byte] & mask_bit) != 0)
        {
                bit_send = true;
                if (sendBit(true))  return;
                if (sendBit(false)) return;
        }
        else
        {
                bit_send = false;
                if (sendBit(false)) return;
                if (sendBit(true))  return;
        }

        const bool bit_recv = recvBit();
        if (_error != Error::NO_ERROR)  return;

        if (bit_send != bit_recv)  return;
        
        position_IDBit++;
    }
}

bool OneWireHub::recvAndProcessCmd(void)
{
    uint8_t address[8], cmd;
    bool    flag = false;

    recv(&cmd);

    if (_error == Error::RESET_IN_PROGRESS) return false; // stay in poll()-loop and trigger another datastream-detection
    if (_error != Error::NO_ERROR)          return true;

    switch (cmd)
    {
        case 0xF0: // Search rom

            noInterrupts();
            searchIDTree();
            interrupts();
            return false; // always trigger a re-init after searchIDTree

        case 0x55: // MATCH ROM - Choose/Select ROM

            if (recv(address, 8))
            {
                break;
            }

            flag = true;
            for (uint8_t j = 0; j < 8; ++j)
            {
               if (slave_list->ID[j] != address[j])
               {
                  flag = false;
                  break;
               }
            }

            if (!flag)
            {
                return true;
            }

            slave_list->duty(this);
            break;

        case 0xCC: // SKIP ROM

            slave_list->duty(this);
            break;

        case 0x0F: // OLD READ ROM

            // only usable when there is ONE slave on the bus --> continue to current readRom

        case 0x33: // READ ROM

            slave_list->sendID(this);
            return false;

        case 0xEC: // ALARM SEARCH

            // TODO: Alarm searchIDTree command, respond if flag is set
            // is like searchIDTree-rom, but only slaves with triggered alarm will appear
            break;

        case 0xA5: // RESUME COMMAND

            slave_list->duty(this);
            break;

        default: // Unknown command

            _error = Error::INCORRECT_ONEWIRE_CMD;
    }

    if (_error == Error::RESET_IN_PROGRESS) return false;

    return (_error != Error::NO_ERROR);
}


// info: check for errors after calling and break/return if possible, returns true if error is detected
// NOTE: if called separately you need to handle interrupts, should be disabled during this FN
bool OneWireHub::sendBit(const bool value)
{
    const bool writeZero = !value;

    // Wait for bus to rise HIGH, signaling end of last timeslot
    timeOW_t retries = ONEWIRE_TIME_SLOT_MAX;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) == 0) && (--retries != 0));
    if (retries == 0)
    {
        _error = Error::RESET_IN_PROGRESS;
        return true;
    }

    // Wait for bus to fall LOW, start of new timeslot
    retries = ONEWIRE_TIME_MSG_HIGH_TIMEOUT;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) != 0) && (--retries != 0));
    if (retries == 0)
    {
        _error = Error::AWAIT_TIMESLOT_TIMEOUT_HIGH;
        return true;
    }

    // first difference to inner-loop of read()
    if (writeZero)
    {
        DIRECT_MODE_OUTPUT(pin_baseReg, pin_bitMask);
        retries = ONEWIRE_TIME_WRITE_ZERO;
    }
    else
    {
        retries = ONEWIRE_TIME_READ_MAX;
    }

    while ((DIRECT_READ(pin_baseReg, pin_bitMask) == 0) && (--retries != 0)); // TODO: we should check for (!retries) because there could be a reset in progress...
    DIRECT_MODE_INPUT(pin_baseReg, pin_bitMask);

    return false;
}


// should be the prefered function for writes, returns true if error occured
bool OneWireHub::send(const uint8_t address[], const uint8_t data_length)
{
    noInterrupts(); // will be enabled at the end of function
    DIRECT_WRITE_LOW(pin_baseReg, pin_bitMask);
    DIRECT_MODE_INPUT(pin_baseReg, pin_bitMask);
    uint8_t bytes_sent = 0;

    for ( ; bytes_sent < data_length; ++bytes_sent)             // loop for sending bytes
    {
        const uint8_t dataByte = address[bytes_sent];

        for (uint8_t bitMask = 0x01; bitMask != 0; bitMask <<= 1)    // loop for sending bits
        {
            if (sendBit(static_cast<bool>(bitMask & dataByte)))
            {
                if ((bitMask == 0x01) && (_error == Error::AWAIT_TIMESLOT_TIMEOUT_HIGH)) _error = Error::FIRST_BIT_OF_BYTE_TIMEOUT;
                interrupts();
                return true;
            }
        }
    }
    interrupts();
    return (bytes_sent != data_length);
}

bool OneWireHub::send(const uint8_t dataByte)
{
    return send(&dataByte,1);
}

// NOTE: if called separately you need to handle interrupts, should be disabled during this FN
bool OneWireHub::recvBit(void)
{
    // Wait for bus to rise HIGH, signaling end of last timeslot
    timeOW_t retries = ONEWIRE_TIME_SLOT_MAX;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) == 0) && (--retries != 0));
    if (retries == 0)
    {
        _error = Error::RESET_IN_PROGRESS;
        return true;
    }

    // Wait for bus to fall LOW, start of new timeslot
    retries = ONEWIRE_TIME_MSG_HIGH_TIMEOUT;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) != 0) && (--retries != 0));
    if (retries == 0)
    {
        _error = Error::AWAIT_TIMESLOT_TIMEOUT_HIGH;
        return true;
    }

    // wait a specific time to do a read (data is valid by then), // first difference to inner-loop of write()
    retries = ONEWIRE_TIME_READ_MIN;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) == 0) && (--retries != 0));

    return (retries > 0);
}


bool OneWireHub::recv(uint8_t address[], const uint8_t data_length)
{
    noInterrupts(); // will be enabled at the end of function
    DIRECT_WRITE_LOW(pin_baseReg, pin_bitMask);
    DIRECT_MODE_INPUT(pin_baseReg, pin_bitMask);

    uint8_t bytes_received = 0;
    for ( ; bytes_received < data_length; ++bytes_received)
    {
        uint8_t value = 0;

        for (uint8_t bitMask = 0x01; bitMask != 0; bitMask <<= 1)
        {
            if (recvBit())                 value |= bitMask;
            if (_error != Error::NO_ERROR)
            {
                if ((bitMask == 0x01) && (_error ==Error::AWAIT_TIMESLOT_TIMEOUT_HIGH)) _error = Error::FIRST_BIT_OF_BYTE_TIMEOUT;
                interrupts();
                return true;
            }
        }

        address[bytes_received] = value;
    }

    interrupts();
    return (bytes_received != data_length);
}

void OneWireHub::wait(const uint16_t timeout_us) const
{
    timeOW_t loops = timeUsToLoops(timeout_us);
    bool state = false;
    while (loops != 0)
    {
        loops = waitLoopsWhilePinIs(loops,state);
        state = !state;
    }
}

void OneWireHub::wait(const timeOW_t loops_wait) const
{
    timeOW_t loops = loops_wait;
    bool state = false;
    while (loops != 0)
    {
        loops = waitLoopsWhilePinIs(loops,state);
        state = !state;
    }
}

// returns false if pins stays in the wanted state all the time
timeOW_t OneWireHub::waitLoopsWhilePinIs(volatile timeOW_t retries, const bool pin_value) const
{
    if (retries == 0) return 0;
    while ((DIRECT_READ(pin_baseReg, pin_bitMask) == pin_value) && (--retries != 0));
    return retries;
}
