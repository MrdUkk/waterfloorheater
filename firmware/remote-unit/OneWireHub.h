#ifndef ONEWIRE_HUB_H
#define ONEWIRE_HUB_H

#include "platform.h" // code for compatibility

using     timeOW_t            = uint32_t;
constexpr timeOW_t timeOW_max = 4294967295; // will arduino-gcc ever offer some stl? std::numeric_limits::max would be cleaner

constexpr timeOW_t operator "" _us(const unsigned long long int time_us) // user defined literal used in config
{
    return timeOW_t(time_us * microsecondsToClockCycles(1) / VALUE_IPL); // note: microsecondsToClockCycles == speed in MHz....
    // TODO: overflow detection would be nice, but literals are allowed with return-only, not solvable ATM
}


// same FN, but not as literal
constexpr timeOW_t timeUsToLoops(const uint16_t time_us)
{
    return (time_us * microsecondsToClockCycles(1) / VALUE_IPL); // note: microsecondsToClockCycles == speed in MHz....
}

#include "OneWireHub_config.h" // outsource configfile

using mask_t = uint8_t;

constexpr timeOW_t VALUE1k      { 1000 }; // commonly used constant
constexpr timeOW_t TIMEOW_MAX   { 4294967295 };   // arduino does not support std-lib...

enum class Error : uint8_t {
    NO_ERROR                   = 0,
    READ_TIMESLOT_TIMEOUT      = 1,
    WRITE_TIMESLOT_TIMEOUT     = 2,
    WAIT_RESET_TIMEOUT         = 3,
    VERY_LONG_RESET            = 4,
    VERY_SHORT_RESET           = 5,
    PRESENCE_LOW_ON_LINE       = 6,
    READ_TIMESLOT_TIMEOUT_LOW  = 7,
    AWAIT_TIMESLOT_TIMEOUT_HIGH = 8,
    PRESENCE_HIGH_ON_LINE      = 9,
    INCORRECT_ONEWIRE_CMD      = 10,
    INCORRECT_SLAVE_USAGE      = 11,
    TRIED_INCORRECT_WRITE      = 12,
    FIRST_TIMESLOT_TIMEOUT     = 13,
    FIRST_BIT_OF_BYTE_TIMEOUT  = 14,
    RESET_IN_PROGRESS          = 15
};


class OneWireItem;

class OneWireHub
{
private:
    io_reg_t          pin_bitMask;
    volatile io_reg_t *pin_baseReg;

    OneWireItem *slave_list;  // private slave-list (use attach/detach)

    void searchIDTree(void);

    bool checkReset(void);      // returns true if error occured
    bool showPresence(void);    // returns true if error occured
    bool recvAndProcessCmd();   // returns true if error occured

    void wait(timeOW_t loops_wait) const;
    void wait(uint16_t timeout_us) const;

    inline __attribute__((always_inline))
    timeOW_t waitLoopsWhilePinIs(volatile timeOW_t retries, bool pin_value = false) const;

public:

    explicit OneWireHub(uint8_t pin);

    ~OneWireHub() = default; // nothing special to do here

    OneWireHub(const OneWireHub& hub) = delete;             // disallow copy constructor
    OneWireHub(OneWireHub&& hub) = default;               // default move constructor
    OneWireHub& operator=(OneWireHub& hub) = delete;        // disallow copy assignment
    OneWireHub& operator=(const OneWireHub& hub) = delete;  // disallow copy assignment
    OneWireHub& operator=(OneWireHub&& hub) = delete;       // disallow move assignment

    uint8_t attach(OneWireItem &sensor);

    bool poll(void);

    bool recvBit(void);
    bool recv(uint8_t address[], uint8_t data_length = 1);                 // returns 1 if error occured
    bool sendBit(bool value);                                                 // returns 1 if error occured
    bool send(uint8_t dataByte);                                              // returns 1 if error occured
    bool send(const uint8_t address[], uint8_t data_length = 1);              // returns 1 if error occured
    // CRC takes ~7.4µs/byte (Atmega328P@16MHz) but is distributing the load between each bit-send to 0.9 µs/bit (see debug-crc-comparison.ino)
    // important: the final crc is expected to be inverted (crc=~crc) !!!


    Error   _error;
};

#endif
