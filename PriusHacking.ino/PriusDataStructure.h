/* Prius Data Structures
 * (c) Edward Cheeseman 2023
 * 
 */
 
#include <stdint.h> // Use data types that have same length on any system
typedef union PriusHTM_ut {
  uint8_t u8[100];
  uint8_t u32[25];
  struct {
    uint8_t engine_speed_0;   // in RPM/32. (presume from ECU) Can calculate this simply as (mg1_rpm + mg2_rpm)/113. More accurately as (mg1_rpm + mg2_rpm*143/145)*5/18/32.
    uint8_t status0_1;        // 14 initally (2 counts), then 30. 28 after engine start (592counts) while engine increases in speed. Maybe undervoltage warning? More testing required
    uint8_t pad0_2;           // always0
    uint8_t status1_3;        // 0 or 2. 0 initially (89counts), 0 while starting engine (202counts), 0 at turnoff or stopping engine (342counts then back to 2)
    uint8_t pad0_4;           // always0
    uint8_t mg1_nqtorque_L5;  // negative, 1/4 torque request for MG1 low byte
    uint8_t mg1_nqtorque_H6;  // negative, 1/4 torque request for MG1 high byte
    uint8_t pad55_7;          // 0 initially (2 counts), then always 55
    uint8_t pad0_8;           // always0
    uint8_t pad126_9;         // 0 initially (2 counts), then always 126
    uint8_t pad255_10;        // 0 initially (2 counts), then always 255
    uint8_t mg1_nqtorque_L11; // copy of mg1_nqtorque_L5
    uint8_t mg1_nqtorque_H12; // copy of mg1_nqtorque_H6
    int8_t coolant_temp_13;   // 0 initially (6 counts), 255(-1?) (79counts), 31->40(129counts), goes up to 50 after high torque then recovers
    uint8_t pad0_14;          // always0
    uint8_t status2_15;       // 0,2,97,98
    uint8_t status3_16;       // 0,4,32,128
    uint8_t pad0_17;          // always0
    uint16_t pad0_18;         // always0
    uint16_t pad0_20;         // always0
    int16_t i16_22;           // unknown
    int16_t i16_24;           // unknown
    int16_t i16_26;           // unknown
    int16_t i16_28;           // unknown
    int16_t i16_30;           // unknown
    int8_t temperature_32;    // 0 initially (2 counts), then 49-52
    uint8_t pad0_33;          // always0
    uint8_t pad0_34;          // always0
    uint8_t pad0_35;          // always0
    int16_t i16_36;           // unknown
    int8_t temperature_38;    // copy of coolant_temp_13
    uint8_t pad0_39;          // always0
    uint8_t status4_40;       // {0,1,2,3}
    uint8_t status5_41;       // 
    uint8_t pad0_42;          // 
    uint8_t status6_43;       // maybe start?
    uint32_t pad0_44;         // 
    uint32_t pad0_48;         // 
    uint32_t pad0_52;         // 
    uint32_t pad0_56;         // 
    uint32_t pad0_60;         // 
    uint16_t pad0_64;         // 
    uint8_t pad0_66;          // 
    uint8_t pad4_67;          // always4
    uint32_t pad0_68;         // 
    int8_t temperature_72;    // closley related to 38
    uint8_t status7_73;       // {0-4}
    uint8_t control1_74;      // 0-75 slow changing value that stays at 0 and 75 for significant time
    uint8_t status8_75;       // 
    int16_t minLimit_76;      // power min (chg) limit?
    int16_t maxlimit_78;      // power max (dischg) limit?
    uint16_t pad0_80;         // 
    int8_t temperature_82;    // 
    uint8_t pad0_83;          // 
    uint16_t pad0_84;         // 
    uint8_t status8_86;       // 
    uint8_t pad0_87;          // 
    uint8_t bms_volts_88;     // 
    uint8_t pad0_89;          // potentially this is upper 8 bits of bms_volts
    uint8_t control2_90;      // ?
    uint8_t mg1_torque_L91;   // MG1 torque request low byte
    uint8_t mg2_torque_H92;   // MG1 torque request high byte
    uint8_t pad0_93;          //
    uint8_t sequence_94;      // increments by one every second message
    uint8_t control3_95;      //
    uint16_t pad0_96;         // 
    uint16_t checksum_98;     // bytewise sum of the data structure.
  }
};
