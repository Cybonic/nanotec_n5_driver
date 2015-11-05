#ifndef NANOTECN5DRIVER_H
#define NANOTECN5DRIVER_H

#include <string>
#include <cstring>
#include <std_msgs/Float64.h>

#include <nanotec_n5_driver/NanotecN5Port.h>

// Canopen variables
#define CANOPEN_INVALID_ID      0x0000
#define CANOPEN_MAX_ID          0x007F
#define CANOPEN_SEND_ID         0x600
#define CANOPEN_RECEIVE_ID      0x580
#define CANOPEN_EMERGENCY_ID    0x0080

#define CANOPEN_WRITE_SEND_1_BYTE       0x2F
#define CANOPEN_WRITE_SEND_2_BYTE       0x2B
#define CANOPEN_WRITE_SEND_4_BYTE       0x23 
#define CANOPEN_WRITE_SEND_UNDEFINED    0x22


#define CANOPEN_READ_RECEIVE_1_BYTE     0x4F
#define CANOPEN_READ_RECEIVE_2_BYTE     0x4B
#define CANOPEN_READ_RECEIVE_4_BYTE     0x43
#define CANOPEN_READ_RECEIVE_UNDEFINED  0x42



#define CANOPEN_OPERATION_MODE_PROFILE_POSITION 1
#define CANOPEN_OPERATION_MODE_VELOCITY 	2
#define CANOPEN_OPERATION_MODE_PROFILE_VELOCITY 3



// Platform dimensions 
#define GEAR_RATIO 48




// Message Flags
#define MESSAGE_SIZE_1 0X01
#define MESSAGE_SIZE_2 0X02
#define MESSAGE_SIZE_4 0X04

#define CANOPEN_DOMAIN_DOWNLOAD_REPLY	0x60
#define CANOPEN_DOMAIN_UPLOAD		0x40

//  *************  Data Objects  *************  
#define CANOPEN_CONTROL_WORD			0x6040
#define CANOPEN_STATUS_WORD			0x6041

// Break Action
#define CANOPEN_BREAK_ACTION_QUICK_STOP		0x605A
#define CANOPEN_BREAK_ACTION_READY_TO_SWITCH_ON	0x605B
#define CANOPEN_BREAK_ACTION_SWITCHED_ON	0x605C
#define CANOPEN_BREAK_ACTION_HALT		0x605D
#define CANOPEN_BREAK_ACTION_FAULT		0x605E

// User-defined Unites
#define CANOPEN_COMPENSATE_POLEPAIR_COUNT	0x2060
#define CANOPEN_GEAR_RATIO			0x6091
#define CANOPEN_FEED_CONSTANT			0x6092
#define CANOPEN_VELOCITY_NUMERATOR		0x2061
#define CANOPEN_VELOCITY_DENOMINATOR		0x2062
#define CANOPEN_ACCELERATION_NUMERATOR		0x2063
#define CANOPEN_ACCELERATION_DENOMINATOR	0x2064
#define CANOPEN_JERK_NUMERATOR 			0x2065
#define CANOPEN_JERK_DENOMINATOR		0x2066
#define CANOPEN_POSITION_ENCODER_RESOLUTION	0x608F

// Operating mode
//Object
#define CANOPEN_SET_MODES_OF_OPERATIONS		0x6060
#define CANOPEN_GET_MODES_OF_OPERATIONS		0x6061

// Profile Mode
// read
#define CANOPEN_VELOCITY_DEMAND_VALUE		0x606B 
#define CANOPEN_VELOCITY_ACTUAL_VALUE		0x606C
// read/write
#define CANOPEN_VELOCITY_WINDOW			0x606D
#define CANOPEN_VELOCITY_WINDOW_TIME		0x606E
#define CANOPEN_POLARITY			0x607E
#define CANOPEN_PROFILE_ACCELERATION		0x6083
#define CANOPEN_PROFILE_DECELERATION		0x6084
#define CANOPEN_QUICK_STOP_DECELERATION		0x6085
#define CANOPEN_MOTION_PROFILE_TYPE		0x6086
#define CANOPEN_VELOCITY_QUICK_STOP		0x604A
#define CANOPEN_TARGET_VELOCITY			0x60FF
#define CANOPEN_PEAK_CURRENT			0x2031
#define CANOPEN_MAX_ACCELERATION		0x60C5
#define CANOPEN_MAX_DECELERATION		0x60C6
#define CANOPEN_JERKS				0x60A4
// Modes
#define CANOPEN_VELOCITY_MODE			0x02
#define CANOPEN_PROFILE_VELOCITY_MODE		0x03


// Break settings
#define CANOPEN_IMMEDIATE_STOP		0x00
#define CANOPEN_SLOW_DOWN_RAMP		0x01
#define	CANOPEN_QUICK_STOP_RAMP		0x02


// Power States cap.7.1.1 state machine 

// Transition Commands			     // Transitions 
#define CANOPEN_DISABLE_VOLTAGE		0x00 // 6,7,9,12
#define CANOPEN_SWITCH_ON		0x07 // 2
#define CANOPEN_SHUTDOWN		0x06 // 1,5,8
#define CANOPEN_QUICK_STOP		0x02 // 10
#define CANOPEN_DISABLE_OPERATION	0x07 // 4
#define CANOPEN_ENABLE_OPERATION	0x0F // 3
#define CANOPEN_FAULT_RESET		0x80 // 13  0x00-> 0x80


// States
#define CANOPEN_NOT_READY_TO_SWITCH_ON	0x00
#define CANOPEN_SWITCH_ON_DISABLED	0x40
#define CANOPEN_READY_TO_SWITCH_ON 	0x21
#define CANOPEN_SWITCHED_ON		0x23
#define CANOPEN_OPERATION_ENABLED	0x27
#define CANOPEN_QUICK_STOP_ACTIVE	0x07
#define CANOPEN_FAULT_REACTION_ACTIVE   0x0F
#define CANOPEN_FAULT 			0x08

// Masks

#define MASK_BOOTUP 			0x6F
#define MASK_FAULT_STATE		0x4F

// State Machine direction flow
#define BACKWARDS	 -1
#define HOLD		  0
#define FOWARDS		  1

#define CLEARFAULD 	  1


#define QUICK_STOP 	-1  // --> QUICK STOP SATE
#define NORMAL_STOP	 1  // --> SWITCH ON DISABLED OR READY TO SWITCH ON

// Protocol parameters 

# define CANOPEN_SDO_MAX_FRAME_SIZE	20 // 
// Time 
#define CANOPEN_MIN_DELAY	3 // [ms] - min delay between writing and the relpy from the driver


// Motor Settings
#define GEAR_RATIO  48
#define DISTANCE_BETWEEN_AXES 0.75
#define TIRE_PERIMETER 1.303
namespace nanotec
{
  
  
  class NanotecN5Driver
  {
  public:
    
    NanotecN5Driver();
    ~NanotecN5Driver();
    void initDriver(nanotec::NanotecN5Port * port,int id);
    
    void stop();
    
    void setMotorSpeed(int speed);
    bool WriteObject ( u_int16_t index, u_int8_t subIndex, int data_size, int data);
    bool ReadObject ( u_int16_t index, u_int8_t subIndex, char * data);
    
    //! Run the state machine of CANopen
    /*!
     * State machine of CANopen DS402 
     * To switch the motor controller to an operational state, a state machine must be run through. 
     * This is defined in CANopen standard DS402.
     * 
     * \param direction - defines the  direction flow of the state machine. 
     * 			 =   1 Foward
     *			 =  -1 Backward 
     * 			 =   0 hold
    
     * \param stop - concerns the power state of the driver 
     * 			 =  0 do nothing
     * 			 = -1 quick stop (just in Operation Enabled state)
     * 			 =  1 normal stop -> Ready to switch on state (Operation enabled) or Switch on disabled state (Switched on) 
     * \sa stateMachine()
     * \return - state machine status
     * 			 = -1 it An error occured
     * 			 =  0 state machine flow normal
     *			 =  1 in operation enabled state 
     * 
     */
    int stateMachine(int direction, int stop,int clearfault);
    
    //! Set the Break Settings Cap.7.1.2 - Behavior after the "Oeration enabled" state is left
    /*!
     *  Breaking reaction 
     *  Different brake reaction can be programmed when leaving the "Operation enabled" state.
     * 
     *  In this verion default settings are loaded. 
     * 
     *  \sa setBreakingSettings(void)
     *  \return - returns "1" if all settings are succefully loaded and "0" if not
     * 
     */
    int setBreakingSettings(void);
    
    int loadDefaultSettings(void);
    
    int setUserDefinedUnits(void);
    
    int setOperatingMode(u_int8_t mode);
    
    void configOperationMode(u_int8_t mode);
    
    int setTargetVelocity(int32_t value);
    
    int getActualVelocity(void);
    
    
  private:
    
    nanotec::NanotecN5Port * node_;
    unsigned int id_;
  };
  
    
}


#endif // NANOTECN5DRIVER_H
