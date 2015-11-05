
#include <nanotec_n5_driver/NanotecN5Port.h>
#include <cstdio>

#define CANOPEN_SEND_ID         0x0600
#define CANOPEN_RECEIVE_ID      0x0580
#define CANOPEN_WRITE_RECEIVE   0x60

nanotec::NanotecN5Port::NanotecN5Port() : port_()
{

}

nanotec::NanotecN5Port::~NanotecN5Port()
{
    port_.close();
}


bool nanotec::NanotecN5Port::openPort(std::string & port_name, int baudrate)
{
    try{ port_.open(port_name.c_str(), baudrate); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    
    return true;
    port_.flush();
}



void nanotec::NanotecN5Port::SendMessage(std::string message)
{
   
    //printf("\n[DEBUG] SendMessage: ");
    //printf("\n%s\n",message.c_str());
    port_.write(message.c_str());
  
}



bool nanotec::NanotecN5Port::RecvMessage(char * reply, int type)
{
  char replys[50];
  //std::string reply;
  try{ port_.readLine(reply, REPLY_SIZE, TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
	printf("\n[ERROR] - Timout receiving reply - %s\n", replys);
	
	return false;
    }
   
    //printf("\n[DEBUG] - RecMessage: ");
    //printf("\n%s\n",reply);
    //strcpy((char*)reply.c_str(),replys);
    
    return true;
}

// n [ms]
void delay (int n)
{
 clock_t act_time,last_time;
 
 last_time = clock();
 act_time = clock();
  while(((act_time-last_time)/1000) < n)
  {
    act_time =clock();
  }
}
