#ifndef NANOTECN5PORT_H
#define NANOTECN5PORT_H



#include <iostream> 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <iomanip>
#include <cereal_port/CerealPort.h>

#define NANOTEC5_MSG_LENGTH  100
#define NANOTEC5_TIMEOUT     100

#define REPLY_SIZE 100
#define TIMEOUT 100


namespace nanotec
{
    /*! \class Nanotec5Port NanotecN5Port.h "nanotecN5Driver/NanotecN5Port.h"
     *  \brief Nanotec5Port serial port, communication class for ROS.
     *
     * This class allows to set, send and receive message throught serial communication to  NanotecN5 Driver.
     */
    class NanotecN5Port
    {
    public:
        //! Constructor
        NanotecN5Port();
        //! Destructor
        ~NanotecN5Port();

        //! Open the serial port to the can network
        /*!
        *
        *  \param port_name    		Serial port name.
        *  \param baudrate  		Baudrate.
        *
        *  \return True if successful false if anything went wrong!
        */
        bool openPort(std::string & port_name, int baudrate);

        //! Send a command to the motor
        /*!
        *  The default function to send and receive data.
        *
        *  \param message    		Array of chars to send.
        *  \param message_size		Size of the message buffer.
        *  \param reply             Array of chars replied by the motor.
        *  \param reply_size		Size of the reply buffer.
        *
        *  \return True if successful false if anything went wrong!
        */
        bool sendCommand(std::string & message, std::string & reply);
	
	bool VerifySend (unsigned int nodeId, u_int16_t index, u_int8_t subIndex);
	
	void SendMessage(std::string message);
	
	bool RecvMessage(char * reply, int type);
	
	cereal::CerealPort port_;

    private:

        //! The serial port object
        
    };
}

void delay (int n);

#endif
// EOF
