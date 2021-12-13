#ifndef ONEWIREHUB_CONFIG_H_H
#define ONEWIREHUB_CONFIG_H_H

#include "platform.h"
/////////////////////////////////////////////////////
// CONFIG ///////////////////////////////////////////
/////////////////////////////////////////////////////

/// the following TIME-values are in microseconds and are taken mostly from the ds2408 datasheet
//  arrays contain the normal timing value and the overdrive-value, the literal "_us" converts the value right away to a usable unit
//  should be --> datasheet
//  was       --> shagrat-legacy

// Reset: every low-state of the master between MIN & MAX microseconds will be recognized as a Reset
constexpr timeOW_t ONEWIRE_TIME_RESET_TIMEOUT        = {  5000_us };        // for not hanging to long in reset-detection, lower value is better for more responsive applications, but can miss resets
constexpr timeOW_t ONEWIRE_TIME_RESET_MIN         = {   430_us }; // should be 480
constexpr timeOW_t ONEWIRE_TIME_RESET_MAX         = {   960_us }; // from ds2413

// Presence: slave waits TIMEOUT and emits a low state after the reset with ~MIN length, if the bus stays low after that and exceeds MAX the hub will issue an error
constexpr timeOW_t ONEWIRE_TIME_PRESENCE_TIMEOUT     = {    20_us };        // probe measures 25us, duration of high state between reset and presence
constexpr timeOW_t ONEWIRE_TIME_PRESENCE_MIN      = {   160_us }; // was 125
constexpr timeOW_t ONEWIRE_TIME_PRESENCE_MAX      = {   480_us }; // should be 280, was 480


constexpr timeOW_t ONEWIRE_TIME_MSG_HIGH_TIMEOUT     = { 15000_us };        // there can be these inactive / high timeperiods after reset / presence, this value defines the timeout for these
constexpr timeOW_t ONEWIRE_TIME_SLOT_MAX          = {   135_us }; // should be 120, measured from falling edge to next falling edge

// read and write from the viewpoint of the slave!!!!
constexpr timeOW_t ONEWIRE_TIME_READ_MIN          = {    20_us }; // should be 15, was 30, says when it is safe to read a valid bit
constexpr timeOW_t ONEWIRE_TIME_READ_MAX          = {    60_us }; // low states (zeros) of a master should not exceed this time in a slot
constexpr timeOW_t ONEWIRE_TIME_WRITE_ZERO        = {    30_us }; // the hub holds a zero for this long

// VALUES FOR STATIC ASSERTS
constexpr timeOW_t ONEWIRE_TIME_VALUE_MAX            = { ONEWIRE_TIME_MSG_HIGH_TIMEOUT };
constexpr timeOW_t ONEWIRE_TIME_VALUE_MIN            = { ONEWIRE_TIME_READ_MIN };

// TODO: several compilers have problems with constexpress-FN in unified initializers of constexpr, will be removed for now -> test with arduino due, esp32, ...

/////////////////////////////////////////////////////
// END OF CONFIG ////////////////////////////////////
/////////////////////////////////////////////////////

#endif //ONEWIREHUB_CONFIG_H_H
