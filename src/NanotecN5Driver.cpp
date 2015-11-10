
#include <nanotec_n5_driver/NanotecN5Driver.h>

#include <cstdio>


nanotec::NanotecN5Driver::NanotecN5Driver(){}
 
void nanotec::NanotecN5Driver::initDriver(nanotec::NanotecN5Port * port,int id)
{
  node_ = port;
  id_	= id;
}




nanotec::NanotecN5Driver::~NanotecN5Driver()
{
 
}


// Stop the nanotec motor controller
void nanotec::NanotecN5Driver::stop()
{
    int index, subIndex, data;
    unsigned  int nodeID = id_;
    //turn off the motor
    index=0x6040;
    subIndex=0x00;
    data=0x6;
    WriteObject (index, subIndex, MESSAGE_SIZE_2, data);
//    motor_stopped=true;
}



void nanotec::NanotecN5Driver::setMotorSpeed(int speed){

    int index, subIndex, data;
    unsigned  int nodeID = id_;
    index=0x60FF;
    subIndex=0x00;
    data=speed;
    WriteObject ( index, subIndex, MESSAGE_SIZE_4, data);

}


// The frames are in little endian format
bool nanotec::NanotecN5Driver::WriteObject ( u_int16_t index, u_int8_t subIndex, int data_size, int data)
{
        // write message
        std::ostringstream message_to_send,message_to_verify;
	char reply[100];
	

        message_to_send << 't';
        //Insert COB-ID
        message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (CANOPEN_SEND_ID + id_);
	//Insert Command Specifier - Domain Download
        message_to_send << std::hex << std::setfill('0') << std::uppercase << 4 + data_size;
        switch (data_size)
        {
            case 1 : message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << CANOPEN_WRITE_SEND_1_BYTE; break;
            case 2 : message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << CANOPEN_WRITE_SEND_2_BYTE; break;
            case 4 : message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << CANOPEN_WRITE_SEND_4_BYTE; break;
            default: return false;
        }
        //IndeX
        message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (index & 0xFF);
        message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((index >> 8) & 0xFF);
	// Sub-Index
        message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (u_int16_t) subIndex;
        //Insert data 
        message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (data & 0xFF);
        if(data_size>1)
            message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((data >> 8) & 0xFF);
        if(data_size>2){
            message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((data >> 16) & 0xFF);
            message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((data >> 24) & 0xFF);
        }
        message_to_send << '\r';
      
	// Send frame to Driver
        node_->SendMessage(message_to_send.str());
	// Wait -- it is necessary to wait at least 3ms until the driver reply
	delay(CANOPEN_MIN_DELAY);
	// Get reply frame frome Driver
        if(node_->RecvMessage(reply,1))
	{
	    //printf("\n[DEBUG] WriteObject: \n");
	    //printf("%s",reply);
	    
	    // Construction of the "supposed reply frame" to compare whit the frame send by the driver
	    message_to_verify << 't';
	    message_to_verify << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (CANOPEN_RECEIVE_ID + id_);
	    message_to_verify << std::hex << std::setfill('0') << std::uppercase << 8;
	    message_to_verify << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << CANOPEN_DOMAIN_DOWNLOAD_REPLY;
	    message_to_verify << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (index & 0xFF);
	    message_to_verify << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((index >> 8) & 0xFF);
	    message_to_verify << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (u_int16_t) subIndex;
	    message_to_verify << std::hex << std::setfill('0') << std::setw(8) << std::uppercase << (int) 0;
	    message_to_verify << '\r';
	    
	    //Comparison between "supposed reply frame" and the real frame send by the driver
	    if(message_to_send.str().compare(message_to_verify.str()))
	    {
	      //printf("\n * Domain download succeeded\n");
	      return true;
	    }else
	    {
	      printf("\nDomain download failed\n ");
	      return false;
	    }
	}
	else
	{
	    printf("\nNo answer from the Driver\n");
	    return false;
	}
	//std::cout << "Reply message:  \n" << reply   <<'\n';
	//std::cout << "Verify message: \n" << message_to_verify.str() <<'\n';
	
}


// The frames are in little endian format
// frame is divided in words; 1 word == 4 bits or 1/2 byte; i.e 2 word == 1 byte
bool nanotec::NanotecN5Driver::ReadObject ( u_int16_t index, u_int8_t subIndex,char * data)
{
      /* Create the frame to ask the value of the object */
    std::ostringstream message_to_send;
    char reply[100];
    char data_big_endian[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::ostringstream aux_cpy;
    char cob_id_reply_str[3];
    char index_subindex_reply_str[5];
    std::string cob_id_str;
    
    message_to_send << 't';
    //Insert COB-ID
    message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (CANOPEN_SEND_ID + id_);
    message_to_send << std::hex << std::setfill('0') << std::uppercase << 4;
    //Insert Command Specifier
    message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << CANOPEN_DOMAIN_UPLOAD;
    //Insert Index
    message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (index & 0xFF);
    message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((index >> 8) & 0xFF);
    //Subindex
    message_to_send << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (u_int16_t) subIndex;
    message_to_send << '\r';
    //Send frame
    node_->SendMessage(message_to_send.str());
    //wait -- It is necessary to wait at least 3ms until the driver reply
    delay(CANOPEN_MIN_DELAY);
    // Get reply frame frome Driver
    node_->RecvMessage(reply,1);
    
    
    /*  *************************  Decode and compare reply frame  ************************* */ 
    // create supposed reply frame
    
    // Not all fields in the frame will match with the reply frame. There are some fields (COB-ID, Index and Subindex) has to match and with this information we can ensure that the reply frame is the right reply. 
    

     // Create and compare COB-ID ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----  
     
     aux_cpy.str("");
     aux_cpy.clear();
     aux_cpy << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (CANOPEN_RECEIVE_ID + id_);
     cob_id_str.clear();
     cob_id_str = aux_cpy.str();
     
     //Extract COB-ID from reply frame
     strncpy(cob_id_reply_str,&reply[3],5);
     cob_id_reply_str[3]='\0';
     
     //printf("[DEBUG] COB-ID: %s\n",cob_id_str.c_str());
     //printf("[DEBUG] COB-ID replay: %s\n",cob_id_reply_str);
    
     
     // Compare COB-ID
     if(strcmp(cob_id_reply_str,cob_id_str.c_str())!=0)
     {
	printf("\n[ERROR] - Reply COB-ID doesn't match\n");
	printf("\n[DEBUG] ReadObject: \n");
	printf("%s",reply);
	
     
	return false;
     }
     
   
     // Create and compare Index subindex ----- ----- ----- ----- ----- ----- -----  ----- ----- ----- ----- ----- ----- -----  
     
     aux_cpy.str("");
     aux_cpy.clear();
     aux_cpy << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (index & 0xFF);
     aux_cpy << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << ((index >> 8) & 0xFF);
     aux_cpy << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (u_int16_t) subIndex;
     std::string index_subindex_str = aux_cpy.str();
     
     //Extract INDEX and Subindex from reply frame
     strncpy((char *)index_subindex_reply_str,&reply[9],6);
     
     
     //printf("[DEBUG] Index Subindex: %s\n", index_subindex_str.c_str());
     //printf("[DEBUG] Index Subindex reply: %s\n",index_subindex_reply_str);
     
     //Compare Index and Subindex
     if(strcmp(index_subindex_reply_str,index_subindex_str.c_str())!=0)
     {
	printf("\n[ERROR] - Reply Index or/and Subindex don't match\n");
	printf("\n[DEBUG] ReadObject: \n");
	printf("%s",reply);
	return false;
     }
    
     //printf("\n * Reply frame match\n");
     // Extract data (originally - little-endian) and convert data to big-endain format ----- ----- ----- ----- ----- ----- -----  ----- -----
     
    char aux[2]={0,0};
    int i=0;
    int start_of_the_last_frame_byte = CANOPEN_SDO_MAX_FRAME_SIZE-1;
    int start_of_next_byte_to_read;
    
    //Put data in Big-Endian format
    //                |  Index  | Sub-Index  | Data  |
    //byte position   |	 5-4	|     3      |	2-0  |
    //word position   |  11-8	|    7-6     |	5-0  |
    
    // the data is in little-endian format, as the frame is divided in words it should be copyed 2 word (1 byte) each iteration 
    
    for(i=0;i<=4;i+=2)
    {
      start_of_next_byte_to_read = start_of_the_last_frame_byte - i; //   
      strncpy(aux,&reply[start_of_next_byte_to_read],2);
      aux[2]='\0';
      strcpy(&data_big_endian[i],aux);
    }
    strcpy(data,data_big_endian);
   //printf("\n[DEBUG] - decode_frame: %s size: %lu\n",data_big_endian,sizeof(reply));
   //printf("\n[DEBUG] ReadObject: \n");
   //printf("%s",reply);
   
   return true;
}





int nanotec::NanotecN5Driver::stateMachine(int direction, int stop,int clearfault)
{
  int actual_state = 0;
  int next_state = 0x0;
  char data[100];
  int data_hex;
  int state;
  bool write_to_control_word = false;
 
  //std::string print = data_str.str();
  
  if(ReadObject(CANOPEN_STATUS_WORD,0x00,data))
  {
      std::istringstream data_str(data);
      
      data_str >> std::setbase(16) >> data_hex;
      if((data_hex & 0xF)>0) // verify if the lower word is > 0
	state = (data_hex & MASK_BOOTUP);
      else
	state = (data_hex & MASK_FAULT_STATE);
    
      
      switch(state)
      {
	case CANOPEN_SWITCH_ON_DISABLED:	//--State--
	  actual_state = CANOPEN_SWITCH_ON_DISABLED;
	  if(direction == FOWARDS)
	  {
	    next_state   = CANOPEN_SHUTDOWN; 		// 1 --> Ready to switch on
	    write_to_control_word = true;
	  }else if(direction == HOLD)
	    write_to_control_word = false;
	  
	  printf("\n[DEBUG] - SWITCH_ON_DISABLED\n");
	  break;
	  
	case CANOPEN_READY_TO_SWITCH_ON:	//--State--
	  actual_state = CANOPEN_READY_TO_SWITCH_ON;
	  if(direction == FOWARDS)
	  {
	      next_state = CANOPEN_SWITCH_ON;   	// 2 --> Switched on 
	      write_to_control_word = true;
	  }
	  else if(direction == BACKWARDS)
	  {
	      next_state = CANOPEN_SHUTDOWN;    	// 6 --> Switch on disabled
	      write_to_control_word = true;
	  }
	  else // HOLD
	  {
	    write_to_control_word = false;
	  }
	  printf("\n[DEBUG] - READY_TO_SWITCH_ON\n");
	  break;
	  
	case CANOPEN_SWITCHED_ON:		//--State--
	  switch(direction)
	  {
	    case FOWARDS:
	      next_state = CANOPEN_ENABLE_OPERATION;  	// 3 --> Operation Enabled
	      write_to_control_word = true;
	      break;
	    
	    case BACKWARDS:
	      next_state = CANOPEN_SHUTDOWN; 	     	// 5 --> Ready to switch on
	      write_to_control_word = true;
	      break;
	      
	    default:
	      write_to_control_word = false;
	       break;
	  }
	  if(stop == NORMAL_STOP)
	  {
	    next_state = CANOPEN_DISABLE_VOLTAGE;  	// 7 --> Switch On Disabled
	    write_to_control_word = true;
	  }
	  printf("\n[DEBUG] - SWITCH_ON\n");
	  break;
	  
	case CANOPEN_OPERATION_ENABLED:		//--State--
	  actual_state = CANOPEN_OPERATION_ENABLED;
	  switch(direction)
	  {
	    case FOWARDS:
	      // Do nothing
	      write_to_control_word = false;
	      //printf("\n[DEBUG] - OPERATION_ENABLED - FORWARD\n");
	      break;
	      
	    case BACKWARDS:
	      write_to_control_word = true;
	      next_state = CANOPEN_SHUTDOWN;		// 4 --> Switched on
	      printf("\n[DEBUG] - OPERATION_ENABLED - BACKWARD\n");
	      break;
	      
	    default :
	      write_to_control_word = false;
	      break;
	  }
	  if(stop == QUICK_STOP)
	  {
	    write_to_control_word = true;
	    next_state = CANOPEN_QUICK_STOP;		//10 --> Quik stop active
	     printf("\n[DEBUG] - OPERATION_ENABLED - QUICK_STOP\n");
	  }else if(stop == NORMAL_STOP)  		
	  {
	    write_to_control_word = true;
	    next_state = CANOPEN_SHUTDOWN;		// 8 --> Ready to switch on
	    printf("\n[DEBUG] - OPERATION_ENABLED - NORMAL_STOP\n");
	  }
	  
	  break;
	  
	case CANOPEN_QUICK_STOP_ACTIVE:		//--State--
	  actual_state = CANOPEN_QUICK_STOP_ACTIVE;
	  next_state   = CANOPEN_DISABLE_VOLTAGE; 	//12 --> Switch on disabled
	  write_to_control_word = true;
	  printf("\n[DEBUG] - CANOPEN_QUICK_STOP_ACTIVE\n");
	  break;
	  
	case CANOPEN_FAULT_REACTION_ACTIVE:	//--State--
	  // do nothing
	  actual_state = CANOPEN_FAULT_REACTION_ACTIVE;
	  write_to_control_word = false;
	  printf("\n[DEBUG] - CANOPEN_FAULT_REACTION_ACTIVE\n");
	  break;
	  
	case CANOPEN_FAULT:			//--State--
	  actual_state = CANOPEN_FAULT;
	  next_state   = CANOPEN_FAULT_RESET; 		//13 --> Switch on disabled
	  if(clearfault == CLEARFAULD)
	  {
	    write_to_control_word = true;
	  }
	  else
	  {
	    write_to_control_word = false;
	  }

	  printf("\n[DEBUG] - FAULT\n");
	  break;
	  
	default:
	  // Verify more specific which state  the driver is
	  next_state = CANOPEN_NOT_READY_TO_SWITCH_ON; 	//  --> Switch on disabled
	  write_to_control_word = true;
	  actual_state = -1;
	  printf("\n[ERRO] - State not recognised\n");
	  break;
      }
  }else
  {
    printf("\n[ERROR] - It was not possible to read the satus word\n");
    return -1;
  }
  if(write_to_control_word == true )
  {
    if(!WriteObject(CANOPEN_CONTROL_WORD,0x00,MESSAGE_SIZE_2,next_state))
    {
      printf("\n[ERROR] - It was not possible to write to control word\n");
      return -1;
    }else
    {
      // Return the actual state
      ReadObject(CANOPEN_STATUS_WORD,0x00,data);
      std::istringstream data_s(data);
	
      data_s >> std::setbase(16) >> data_hex;
      if((data_hex & 0xF)>0) // verify if the lower word is > 0
	actual_state = (data_hex & MASK_BOOTUP);
      else
	actual_state = (data_hex & MASK_FAULT_STATE);
      
      return actual_state;
    }
  }
  return -1;
  
}


int nanotec::NanotecN5Driver::setBreakingSettings(void)
{
  // Transition to Quick Stop 		(0x605A)
  WriteObject(CANOPEN_BREAK_ACTION_QUICK_STOP,0x00,MESSAGE_SIZE_2,CANOPEN_QUICK_STOP_RAMP);
  // Transition to Ready To Switch ON 	(0x605B)
  WriteObject(CANOPEN_BREAK_ACTION_READY_TO_SWITCH_ON,0x00,MESSAGE_SIZE_2,CANOPEN_SLOW_DOWN_RAMP);
  // Transition to Switched On 		(0x605C)
  WriteObject(CANOPEN_BREAK_ACTION_SWITCHED_ON,0x00,MESSAGE_SIZE_2,CANOPEN_SLOW_DOWN_RAMP);
  // Transition to Halt 		(0x605D)
  WriteObject(CANOPEN_BREAK_ACTION_HALT,0x00,MESSAGE_SIZE_2,CANOPEN_QUICK_STOP_RAMP);
   // Transition to Fault 		(0x605E)
  WriteObject(CANOPEN_BREAK_ACTION_FAULT,0x00,MESSAGE_SIZE_2,CANOPEN_QUICK_STOP_RAMP);
  return 1;
  
}

int nanotec::NanotecN5Driver::setUserDefinedUnits(void)
{
  int data;
  data = 1;
  WriteObject(CANOPEN_COMPENSATE_POLEPAIR_COUNT,0x00,MESSAGE_SIZE_2,data);
  
  // Gear Ratio (48)
  data = GEAR_RATIO;
  WriteObject(CANOPEN_GEAR_RATIO,0x01,MESSAGE_SIZE_4,data);
  
  // Feed Constant (Wheel perimeter)
  data=(int32_t)1;//(TIRE_PERIMETER*1000); // Perimeter of the wheel
  WriteObject(CANOPEN_FEED_CONSTANT, 0x01, MESSAGE_SIZE_4, data);
  data =(int32_t)1;//1000;
  WriteObject(CANOPEN_FEED_CONSTANT, 0x02, MESSAGE_SIZE_4, data);
  
  //Position encoder resolution
  data=(int32_t)4000; 
  WriteObject(CANOPEN_POSITION_ENCODER_RESOLUTION, 0x01, MESSAGE_SIZE_4, data);
  data =(int32_t)1;
  WriteObject(CANOPEN_POSITION_ENCODER_RESOLUTION, 0x02, MESSAGE_SIZE_4, data);
  
  // Setting the motion units
  
  // Speed factor
  // n_v = NUMERATOR/DEMOMINATOR (r/s)
  data=(int32_t)SPEED_NUMERATOR_CONST; 
  WriteObject(CANOPEN_VELOCITY_NUMERATOR, 0x00, MESSAGE_SIZE_4, data);
  data =(int32_t)SPEED_DENOMINATOR_CONST;
  WriteObject(CANOPEN_VELOCITY_DENOMINATOR, 0x00, MESSAGE_SIZE_4, data);
  // Acceleration factor
  //n_a = NUMERATOR/DEMOMINATOR (r/s^2)
  data=(int32_t)1;
  WriteObject(CANOPEN_ACCELERATION_NUMERATOR, 0x00, MESSAGE_SIZE_4, data);
  data =(int32_t)1;
  WriteObject(CANOPEN_ACCELERATION_DENOMINATOR, 0x00, MESSAGE_SIZE_4, data);
  // jerk factor
  //n_j = NUMERATOR/DEMOMINATOR (r/s^3)
  data=(int32_t)1; 
  WriteObject(CANOPEN_JERK_NUMERATOR, 0x00, MESSAGE_SIZE_4, data);
  data =(int32_t)1;
  WriteObject(CANOPEN_JERK_DENOMINATOR, 0x00, MESSAGE_SIZE_4, data);
  
  return 1;
}





int nanotec::NanotecN5Driver::loadDefaultSettings(void)
{
  setBreakingSettings();
  setUserDefinedUnits();
  setOperatingMode(CANOPEN_PROFILE_VELOCITY_MODE);
  return 1;
}




int nanotec::NanotecN5Driver::setOperatingMode(u_int8_t mode)
{
  char data[100];
  //SET
  WriteObject(CANOPEN_SET_MODES_OF_OPERATIONS, 0x00, MESSAGE_SIZE_1, mode);
  //GET (confirme if the operation mode is correctly set)
  if(ReadObject(CANOPEN_GET_MODES_OF_OPERATIONS,0x00,data))
  {
    if(mode == atoi(data))
    {
      configOperationMode(mode);
      printf("\n[INFO] - Operation mode %d is correctly set\n",mode);
      return 1;
    }
  }
  printf("\n[ERROR] - %d Operation mode is not correctly set\n",mode);
  return 0;
}






void nanotec::NanotecN5Driver::configOperationMode(u_int8_t mode)
{
   u_int16_t data;
   switch(mode)
   {
     case CANOPEN_PROFILE_VELOCITY_MODE:
       // set velocity window
       //WriteObject(CANOPEN_VELOCITY_WINDOW, 0x00, MESSAGE_SIZE_2, data);
       //WriteObject(CANOPEN_VELOCITY_WINDOW_TIME, 0x00, MESSAGE_SIZE_2, data);
       
       // set acceleration/deceleration settings
       data = 50; // [rps]  --> 3000 [rpm]
       WriteObject(CANOPEN_MAX_ACCELERATION, 0x00, MESSAGE_SIZE_4, data);
       WriteObject(CANOPEN_MAX_DECELERATION, 0x00, MESSAGE_SIZE_4, data);
       data = 30; // [rps]  --> xxxx [rpm]
       WriteObject(CANOPEN_PROFILE_ACCELERATION, 0x00, MESSAGE_SIZE_4, data);
       WriteObject(CANOPEN_PROFILE_DECELERATION, 0x00, MESSAGE_SIZE_4, data);
       data = 40; // [rps]
       WriteObject(CANOPEN_QUICK_STOP_DECELERATION, 0x00, MESSAGE_SIZE_4, (int32_t)data);
       data = (u_int16_t) 0; // Trapezoid ramp
       WriteObject(CANOPEN_MOTION_PROFILE_TYPE, 0x00, MESSAGE_SIZE_2, data);
       
       //Peak Current
       data = 18000; // 18A
       WriteObject(CANOPEN_PEAK_CURRENT, 0x00, MESSAGE_SIZE_4,(int32_t) data);
       //
       break;
     default:
       break;
   }
}




int nanotec::NanotecN5Driver::setTargetVelocity(double value)
{
  char data[100];
  char value_str_hex[100];
  int32_t input = (int32_t)round(value * SPEED_DENOMINATOR_CONST);
  // concerting int -> hex
  sprintf(value_str_hex,"%x",input);
  if(WriteObject(CANOPEN_TARGET_VELOCITY, 0x00, MESSAGE_SIZE_4,input))
     if(ReadObject(CANOPEN_TARGET_VELOCITY,0x00,data))
       if(strcmp(value_str_hex,data))
       {
	   
	 //printf("\n[DEBUG] - Target velocity was writen\n");
	 return 1;
       }
   printf("\n [ERROR] -  Target velocity was not writen: %s %d %lf %s\n",data,input,(value*SPEED_DENOMINATOR_CONST),value_str_hex);
  return 0;
}



double nanotec::NanotecN5Driver::getActualVelocity(void)
{
  char data[100];
  int data_hex;
  ReadObject(CANOPEN_VELOCITY_ACTUAL_VALUE,0x00,data);
  std::istringstream data_str(data);
  
  data_str >>std::hex >> (data_hex);
  return((double)hex2int(data_hex)/SPEED_DENOMINATOR_CONST);
}


int hex2int(int value)
{
  if( value > 0x7FFFFF)
  {
    return (0x1000000 - value);
  }
  return(value);
}