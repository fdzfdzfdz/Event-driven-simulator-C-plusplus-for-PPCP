#include "Node.h"
#include <ctime>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <deque>

extern ofstream logs;
//extern ofstream logs4;
extern long long Simulation_time;
extern int Value_TCPP_Max;
extern int No_of_data_field;
extern int length_TCPPs_max;
extern int Maximum_EOT;
extern int Silence_unit;
extern int Processing_delay;
extern int Propagation_delay;


extern int Time_slot_SYNC;  //35
extern int Contention_windwo_SYNC; //15

extern int Time_slot_RTS; //25
extern int Contention_window_RTS; //31

extern int Back_off_main;     
extern int Time_for_SYNC;     
extern int Listen_period;     
extern int Sleep_period;
extern int Initial_listen_period;  
extern int Initial_SYNC_backoff_period;    //57        112
extern int Initial_rebro_SYNC_backoff_period;  //14       28
extern int SYNC_maximum; //54;
extern int SYNC_period;

extern int Calculate_Pulse_count(int *Signal_array, int i);
//extern int calculate_Pulse_swtich_count(int *Signal_array, int i);
extern bool Error_signal_identify(int *Signal_array, int i);
extern Node *getNode_from_id(int i);
extern void Choose_receiver(int* Table, int Size);
extern void Interleaving_cluster(int* Signal, int Signal_size, int* Values_array_id_transmitter, int* Number_sum_id_transmitter, 
	int* Values_array, int Values_array_size);
extern int Decoding_cluster(deque<TCPP_pair> &Queuing_system_temporary, int* TCPP_Data, int TCPP_Size, int Transmitter_id, 
	int Receiver_id, long long Current_time);
extern void Digits_check_big_data(int* Number_sum_array, int* Values_array, deque<TCPP_pair> &Queuing_system);
extern void Digits_check_id(int* Number_sum, int* Values_array, int Decimal_value);
extern void Singal_construction(int* Signal, int Signal_size, int type_decimal, int* Values_array_id_transmitter, int* Number_sum_id_transmitter, 
	int* Values_array_id_receiver, int* Number_sum_id_receiver, int* Values_array, int* Number_sum_array, int Num_of_values_in_packet);
extern void Signal_detection(int* Signal, int Signal_size, int type_decimal, int* Id_addition, int Id_addition_size);
extern void Receiving_time_calculation(long long* Parameter, int* Signal, int Signal_size, bool radio_on, long long Radio_off_time, 
	long long Last_moment_receiving, long long Current_time);

extern int Minimum_Schedule(long long *Schedule_table_sleep_time, int N);
extern int Relative_time_radio_off(int M, long long *Schedule_table_sleep_time, int N);
extern void Head_print(int Event_type, int Node_id, long long Current_time, int SYNC_buffer_0, int SYNC_buffer_1, int SYNC_buffer_2, 
	int Transmitter_id, int Receiver_id, int RTS_buffer_0, int RTS_buffer_1, int Interleaving_num, bool response_SYNC_control);
extern void Process_print(int Forward_type, int Forward_Node_id, long long Forward_time, int Event_pointer_size, int Section, long long Current_time);
extern void Print_schedule_table(int *Schedule_table_node_id, int M, long long *Schedule_table_sleep_time, int N);

using namespace std;

Node::Node(void): Node_id(-1), Xaxis(-1), Yaxis(-1), Timer_last_bit(-1), Back_off_coeff(1), Backoff_time(0), Interleaving_num(0), 
	Transmitter_id(-1), Receiver_id(-1),
	Timer_event(NULL), Time_of_bo(0), Temp_of_TCPP_time(-1), Temp_of_TCPP(-1), response_SYNC_control(true), Time_radio_on(-1), 
	Timer_event_radio_off(NULL), radio_on(false), 
	Timer_normal_periodic_SYNC(SYNC_period), Total_num_tran_TCPP(0), Succe_num_tran_TCPP(0), Total_num_rece_TCPP(0), Total_time_radio_on(0), 
	Total_time_interval(0), Data_tran_interval(0), Loop_waiting_delay(0), Loop_access_delay(0), 
	Waiting_delay(0), Access_delay(0), Total_RTS(0),  Total_CTS(0), Total_EOT(0), Pulse_count(0), 
	Pulse_count_non_SYNC(0), SYNC_time_radio_on(0), Trans_finish_time(0), Total_times_tran(0), 
	Succe_timers_tran(0), Total_time_receiving(0), SYNC_time_receiving(0), Radio_off_time(0), Last_moment_receiving(0), Total_SYNC(0)
{
	for(int i=0; i<10; i++) {this->RTS_CTS_buffer[i]=-1; this->SYNC_buffer[i]=-1;} 
	for (int i=0; i<5; i++) {this->Schedule_table_node_id[i]=-1; this->Schedule_table_sleep_time[i]=0; }
}
Node::~Node(void)
{}

void Node::Back_off_delay_SYNC(Event *handle, Event *New_coming_event)    //Event_type==0
{
	Head_print(0, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	if(this->radio_on!=false || this->Interleaving_num!=0)  //just for debug
		{cout<<"Error code State-01 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;}

	this->Time_radio_on=New_coming_event->getEvent_time(); this->radio_on=true;
	if(New_coming_event->getEvent_time()<=Initial_listen_period)   //synchronization for the first time       section-1
	{   //go to Event_type_1 to send SYNC and then go to sleep 
		int Initial_back_off_time=(rand()%(Initial_SYNC_backoff_period/SYNC_maximum))*SYNC_maximum;  
		Event* future_event=new Event;                 
		future_event->setEvent_time(New_coming_event->getEvent_time()+Initial_back_off_time); 
		future_event->setEvent_type(1); future_event->setEvent_node_id(this->Node_id); 
		future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1000); 
		handle->add(future_event);
		Process_print(1, this->Node_id, New_coming_event->getEvent_time()+Initial_back_off_time, 0,1, New_coming_event->getEvent_time());

		Event* future_event1=new Event;                  //radio turns off
		future_event1->setEvent_time(New_coming_event->getEvent_time()+Initial_listen_period); //New_coming_event->getEvent_time() is useless
		future_event1->setEvent_type(20); future_event1->setEvent_node_id(this->Node_id); 
		future_event1->setEvent_pointer(NULL); future_event1->setEvent_pointer_size(0); future_event1->setTCPP(-1000); 
		handle->add(future_event1);
		this->Timer_event_radio_off=future_event1;
		Process_print(20, this->Node_id, New_coming_event->getEvent_time()+Initial_listen_period, 0,1,New_coming_event->getEvent_time());
	}
	else if(New_coming_event->getEvent_time()>Initial_listen_period && New_coming_event->getEvent_time()>=this->Timer_normal_periodic_SYNC)  
	{//section-2
		//if(this->SYNC_buffer[0]==0)

		if(this->SYNC_buffer[0]<0 || this->SYNC_buffer[0]==1)
		{
			this->SYNC_buffer[0]=-1; this->response_SYNC_control=true; 
			int Initial_back_off_time=(rand()%                       //normal periodic SYNC   
				((Time_for_SYNC-Processing_delay-Propagation_delay-Time_slot_SYNC)/Time_slot_SYNC))*Time_slot_SYNC;   
			Event* future_event=new Event;
			future_event->setEvent_time(New_coming_event->getEvent_time()+Initial_back_off_time); 
			future_event->setEvent_type(1); future_event->setEvent_node_id(this->Node_id); 
			future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1000); 
			handle->add(future_event);
			Process_print(1, this->Node_id, New_coming_event->getEvent_time()+Initial_back_off_time, 0,2,New_coming_event->getEvent_time());
		}
		if(this->SYNC_buffer[0]==2 && this->response_SYNC_control==true)      //no chance to rebro. SYNC last time, now do it in this time
		{
			Event* future_event=new Event;
			future_event->setEvent_time(New_coming_event->getEvent_time() ); 
			future_event->setEvent_type(4); future_event->setEvent_node_id(this->Node_id); 
			future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1000); 
			handle->add(future_event);
			Process_print(4, this->Node_id, New_coming_event->getEvent_time(), 0,2,New_coming_event->getEvent_time());
		}
		/*if(this->Queuing_system.size()!=0)
		{*/
			if(this->Timer_event!=NULL)                         //for RTS/CTS    //check if a turn-off radio event is necessary or not
				{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
			Event* future_event1=new Event; 
			future_event1->setEvent_time(New_coming_event->getEvent_time()+Time_for_SYNC); 
			future_event1->setEvent_type(8); future_event1->setEvent_node_id(this->Node_id); 
			future_event1->setEvent_pointer(NULL); future_event1->setEvent_pointer_size(0); future_event1->setTCPP(-1000); 
			handle->add(future_event1);
			Process_print(8, this->Node_id, New_coming_event->getEvent_time()+Time_for_SYNC, 0,2,New_coming_event->getEvent_time());
		//}

		int m=Minimum_Schedule(this->Schedule_table_sleep_time, 5);                     //for radio turns off
		if(this->Schedule_table_sleep_time[m]-Listen_period!=New_coming_event->getEvent_time())   //just for debug
			cout<<"Error code State-02 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;
		int k=Relative_time_radio_off(m, this->Schedule_table_sleep_time, 5);  //calculate the time of radio's turning off
		if(this->Schedule_table_sleep_time[k]-Listen_period<New_coming_event->getEvent_time())                //just for debug
			cout<<"Error code State-03 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;
		Event* future_event2=new Event;              //radio turns off
		future_event2->setEvent_time(this->Schedule_table_sleep_time[k]); 
		future_event2->setEvent_type(20); future_event2->setEvent_node_id(this->Node_id); 
		future_event2->setEvent_pointer(NULL); future_event2->setEvent_pointer_size(0); future_event2->setTCPP(-1000); 
		handle->add(future_event2);
		this->Timer_event_radio_off=future_event2;
		Process_print(20, this->Node_id, this->Schedule_table_sleep_time[k], 0,2,New_coming_event->getEvent_time());
	}
	else
	{//section-3
		/*if(this->Queuing_system.size()!=0)
		{*/
			if(this->Timer_event!=NULL)                        //for RTS/CTS   //check if a turn-off radio event is necessary or not
				{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
			Event* future_event1=new Event;                        
			future_event1->setEvent_time(New_coming_event->getEvent_time()+Time_for_SYNC); 
			future_event1->setEvent_type(8); future_event1->setEvent_node_id(this->Node_id); 
			future_event1->setEvent_pointer(NULL); future_event1->setEvent_pointer_size(0); future_event1->setTCPP(-1000); 
			handle->add(future_event1);
			Process_print(8, this->Node_id, New_coming_event->getEvent_time()+Time_for_SYNC, 0, 3, New_coming_event->getEvent_time());
		//}

		int m=Minimum_Schedule(this->Schedule_table_sleep_time, 5);                     //for radio turns off
		if(this->Schedule_table_sleep_time[m]-Listen_period!=New_coming_event->getEvent_time())   //just for debug
			cout<<"Error code State-04 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;
		int k=Relative_time_radio_off(m, this->Schedule_table_sleep_time, 5);  //calculate the time of radio's turning off
		if(this->Schedule_table_sleep_time[k]-Listen_period<New_coming_event->getEvent_time())                //just for debug
			cout<<"Error code State-05 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;
		Event* future_event2=new Event;              //radio turns off
		future_event2->setEvent_time(this->Schedule_table_sleep_time[k]); 
		future_event2->setEvent_type(20); future_event2->setEvent_node_id(this->Node_id); 
		future_event2->setEvent_pointer(NULL); future_event2->setEvent_pointer_size(0); future_event2->setTCPP(-1000); 
		handle->add(future_event2);
		this->Timer_event_radio_off=future_event2;
		Process_print(20, this->Node_id, this->Schedule_table_sleep_time[k], 0,3,New_coming_event->getEvent_time());
	}
}

void Node::Send_SYNC(Event *handle, Event *New_coming_event)                  //Event_type==1
{
	Head_print(1, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	if(this->SYNC_buffer[0]<=0 && this->response_SYNC_control==true && New_coming_event->getEvent_time()>=this->Timer_last_bit )
	{//section-1
		this->SYNC_buffer[0]=1; this->response_SYNC_control=false;
		int SYNC_sleep_time=0;
		if(New_coming_event->getEvent_time()<=Initial_listen_period)        //synchronization for the first time
		{
			SYNC_sleep_time=static_cast<int>(Initial_listen_period-New_coming_event->getEvent_time());
			this->Schedule_table_node_id[0]=this->Node_id;
			this->Schedule_table_sleep_time[0]=New_coming_event->getEvent_time()+SYNC_sleep_time;
		}
		else                //normal periodic SYNC
		{
			SYNC_sleep_time=static_cast<int>(this->Schedule_table_sleep_time[0]-New_coming_event->getEvent_time());
			this->Timer_normal_periodic_SYNC=this->Timer_normal_periodic_SYNC+SYNC_period;   //control normal synchronization
		}
		if(SYNC_sleep_time==0)               //just for debug
			cout<<"SYNC_sleep_time==0 State-1 at "<<New_coming_event->getEvent_time()<<endl;

		int* Number_sum_id=new int [2](); int* Values_array_id=new int [5]();
		Digits_check_id(Number_sum_id, Values_array_id, this->Node_id);
		int* Number_sum_array=new int [2](); int* Values_array=new int [10]();
		Digits_check_id(Number_sum_array, Values_array, SYNC_sleep_time);

		int SYNC_size=5+3+2*Number_sum_id[0]+1+Number_sum_id[1]+2*Number_sum_array[0]+1+Number_sum_array[1];

		for(int i=0; i<10; i++)
		{   
			if(this->Neighbor_table[i]<0) break; 
			int* SYNC=new int [SYNC_size]();
			Singal_construction(SYNC, SYNC_size, 0, Values_array_id, Number_sum_id, NULL, NULL, Values_array, Number_sum_array, 0);
			
			Event* future_event=new Event;
			future_event->setEvent_pointer(SYNC);
			future_event->setEvent_time(New_coming_event->getEvent_time()+Propagation_delay); 
			future_event->setEvent_type(2); future_event->setEvent_node_id(this->Neighbor_table[i]); 
			future_event->setEvent_pointer_size(SYNC_size);
			future_event->setTCPP(-29);
			handle->add(future_event);

			if(i==0) //measurement
			{
				this->Total_SYNC=this->Total_SYNC+1;
				int N=Calculate_Pulse_count(SYNC,SYNC_size);
				this->Total_time_interval=this->Total_time_interval+SYNC_size-N;
				this->Pulse_count=this->Pulse_count+N;
			}
			Process_print(2, this->Neighbor_table[i], New_coming_event->getEvent_time()+Propagation_delay,
				SYNC_size, -1, New_coming_event->getEvent_time());
		}
		this->Timer_last_bit=New_coming_event->getEvent_time()+SYNC_size;
		delete[] Number_sum_id; delete[] Values_array_id; delete[] Number_sum_array; delete[] Values_array;
	}
}

void Node::Receive_SYNC_start(Event *handle, Event *New_coming_event)             //Event_type==2
{
	Head_print(2, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	for(int i=9; i>=0; i--)
	{
		if(this->SYNC_buffer[i]>0) {this->SYNC_buffer[i+1]=4; break; }
		if(i==0) this->SYNC_buffer[i]=4; 
	}
	int* SYNC=New_coming_event->getEvent_pointer(); int SYNC_size=New_coming_event->getEvent_pointer_size();
	Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+SYNC_size);
	future_event->setEvent_type(3); future_event->setEvent_node_id(this->Node_id); future_event->setEvent_pointer(SYNC);
	future_event->setEvent_pointer_size(SYNC_size); future_event->setTCPP(-11);
	handle->add(future_event);
	Process_print(3, this->Node_id, New_coming_event->getEvent_time()+SYNC_size, SYNC_size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_SYNC_end_back_off(Event *handle, Event *New_coming_event)      //Event_type==3
{
	Head_print(3, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* SYNC=New_coming_event->getEvent_pointer(); int SYNC_size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;      //check if collision-1
	for(int k=0; k<10; k++)
	{
		if(this->SYNC_buffer[k]>=4) kk++;
		if(this->SYNC_buffer[k]<=0) break;
	}
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-SYNC_size>this->Timer_last_bit)  //find receiver_id in RTS under the premise of non-error
	{
		int* Id_addition=new int [2]();
		Signal_detection(SYNC, SYNC_size, 0, Id_addition, 2);
		int Transmitter=Id_addition[0]; 
		int Sleep_time=Id_addition[1];
		
		//cout<<"SYNC_sleep_time1="<<Sleep_time1<<" "<<"SYNC_sleep_time2="<<Sleep_time2<<endl;
		for(int jj=0; jj<10; jj++)
			{if(this->SYNC_buffer[jj]==4) this->SYNC_buffer[jj]=-1; }
		long long Real_sleep_time=New_coming_event->getEvent_time()-SYNC_size-Propagation_delay+Sleep_time;
		
		//measurement
		//does not split necessary idle listening (idle between pulses) with unnecessary idle listening (waiting for signal)
		long long* Parameter=new long long [2](); 
		Parameter[0]=this->Total_time_receiving; 
		long long Time_receiving_before_SYNC=this->Total_time_receiving;
		Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, SYNC, SYNC_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; 
		this->SYNC_time_receiving=this->SYNC_time_receiving+this->Total_time_receiving-Time_receiving_before_SYNC;
		this->Last_moment_receiving=Parameter[1];

		delete[] SYNC; delete[] Id_addition; delete[] Parameter;
		for(int i=0; i<5; i++)  //put the received SYNC (schedule) into list 
		{
			if(this->Schedule_table_node_id[0]<0)
				{this->Schedule_table_node_id[0]=this->Node_id; this->Schedule_table_sleep_time[0]=Real_sleep_time; break; }
			if(this->Schedule_table_node_id[i]==Transmitter) 
				{this->Schedule_table_sleep_time[i]=Real_sleep_time; break;}
			if(i==4 && this->Schedule_table_node_id[i]!=Transmitter)
			{
				if(this->Schedule_table_sleep_time[0]!=Real_sleep_time) //check if the received SYNC is equal to its own SYNC //just for debug
				{
					for(int j=0; j<5; j++)
					{
						if(this->Schedule_table_node_id[j]<0) 
						{this->Schedule_table_node_id[j]=Transmitter; this->Schedule_table_sleep_time[j]=Real_sleep_time; break; }
					}
				}
			}
		}
		if(this->SYNC_buffer[0]<0 && this->response_SYNC_control==true)
		{
			this->SYNC_buffer[0]=2;
			Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+Processing_delay);  //go to rebroadcast_back_off
			future_event->setEvent_type(4); future_event->setEvent_node_id(this->Node_id); 
			future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1500); 
			handle->add(future_event);
			Process_print(4, this->Node_id, New_coming_event->getEvent_time()+Processing_delay, 0,-1,New_coming_event->getEvent_time());
		}
	}
	else    //error    //receive more than one RTS   //already flush the buffer before
	{
		//measurement
		long long* Parameter=new long long [2](); 
		Parameter[0]=this->Total_time_receiving; 
		long long Time_receiving_before_SYNC=this->Total_time_receiving;
		Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, SYNC, SYNC_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; 
		this->SYNC_time_receiving=this->SYNC_time_receiving+this->Total_time_receiving-Time_receiving_before_SYNC;
		this->Last_moment_receiving=Parameter[1];

		delete[] SYNC; delete[] Parameter;
		for(int i=0; i<10; i++)
			{if(this->SYNC_buffer[i]>=4) {this->SYNC_buffer[i]=-1; break; } }
		if(this->Schedule_table_sleep_time[0]==0 && this->Schedule_table_node_id[0]!=this->Node_id)
		{
				//cout<<"Node-"<<this->Node_id<<" cannot be synchronized state-3."<<endl; 
		}
		Process_print(3, -1, -1, -1,-1,New_coming_event->getEvent_time());
		//check if go to state-4 
	}
}

void Node::Rebroadcast_SYNC_bakc_off(Event *handle, Event *New_coming_event)   //Event_type==4
{
	Head_print(4, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	if(New_coming_event->getEvent_time()<Initial_listen_period)   //rebroadcast for the first time 
	{//section-1
		int Rebro_back_off_time=(rand()%(Initial_rebro_SYNC_backoff_period/SYNC_maximum))*SYNC_maximum;
		Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+Rebro_back_off_time);  //rebroadcast for the first time
		future_event->setEvent_type(5); future_event->setEvent_node_id(this->Node_id); 
		future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1500); 
		handle->add(future_event);
		Process_print(5, this->Node_id, New_coming_event->getEvent_time()+Rebro_back_off_time, 0, 1,New_coming_event->getEvent_time());
	}
	else      //regular rebroadcast (not first time)
	{//section-2
	    int Rebro_back_off_time=(rand()%((Time_for_SYNC-Time_slot_SYNC-Processing_delay-Propagation_delay)/Time_slot_SYNC))*Time_slot_SYNC; 
		if(New_coming_event->getEvent_time()+Rebro_back_off_time+Time_slot_SYNC+Propagation_delay+Processing_delay
			<=this->Time_radio_on+Time_for_SYNC)         //time left in current frame to rebroadcast SYNC
		{
			Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+Rebro_back_off_time); 
			future_event->setEvent_type(5); future_event->setEvent_node_id(this->Node_id); 
			future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1500); 
			handle->add(future_event);
			Process_print(5, this->Node_id, New_coming_event->getEvent_time()+Rebro_back_off_time, 0,2,New_coming_event->getEvent_time());
		}
		else         //no time left in current frame to rebroadcast SYNC
		{
			//cout<<"Node-"<<this->Node_id<<"will rebro SYNC next time!"<<endl;
		}
	}
}

void Node::Rebroadcast_SYNC(Event *handle, Event *New_coming_event)      //Event_type==5
{
	Head_print(5, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int kk=0;      //during rebro back-off time, this node maybe receives another SYNC
	for(int k=0; k<10; k++)
	{
		if(this->SYNC_buffer[k]>=4) kk++;
		if(this->SYNC_buffer[k]<=0) break;
	}
	if(kk>=1)
	{
		//cout<<"Receiving sth before rebro SYNC State-5 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time()<<endl;
	}
	else
	{
		if(this->response_SYNC_control==true)
		{
			this->response_SYNC_control=false;
			if(this->SYNC_buffer[0]==2) this->SYNC_buffer[0]=-1;

			if(New_coming_event->getEvent_time()>Initial_listen_period)
				this->Timer_normal_periodic_SYNC=this->Timer_normal_periodic_SYNC+SYNC_period;   //control normal synchronization, SYNC_period=10sec
			int SYNC_sleep_time=static_cast<int>(this->Schedule_table_sleep_time[0]-New_coming_event->getEvent_time());

			if(SYNC_sleep_time==0)
				cout<<"SYNC_sleep_time==0 State-1 at "<<New_coming_event->getEvent_time()<<endl;
			
			int* Number_sum_id=new int [2](); int* Values_array_id=new int [5]();
			Digits_check_id(Number_sum_id, Values_array_id, this->Node_id);
			int* Number_sum_array=new int [2](); int* Values_array=new int [10]();
			Digits_check_id(Number_sum_array, Values_array, SYNC_sleep_time);

			int SYNC_size=5+3+2*Number_sum_id[0]+1+Number_sum_id[1]+2*Number_sum_array[0]+1+Number_sum_array[1];

			for(int i=0; i<10; i++)
			{
				if(this->Neighbor_table[i]<0) break;
				int* SYNC=new int [SYNC_size]();
				Singal_construction(SYNC, SYNC_size, 0, Values_array_id, Number_sum_id, NULL, NULL, Values_array, Number_sum_array, 0);
				
				Event* future_event=new Event;
				future_event->setEvent_pointer(SYNC);
				future_event->setEvent_time(New_coming_event->getEvent_time()+Propagation_delay); 
				future_event->setEvent_type(6); future_event->setEvent_node_id(this->Neighbor_table[i]); 
				future_event->setEvent_pointer_size(SYNC_size); future_event->setTCPP(-29); 
				handle->add(future_event);

				//measurement
				if(i==0)
				{
					this->Total_SYNC=this->Total_SYNC+1;
					int N=Calculate_Pulse_count(SYNC,SYNC_size);
					this->Total_time_interval=this->Total_time_interval+SYNC_size-N;
					this->Pulse_count=this->Pulse_count+N;
				}
				Process_print(6, this->Neighbor_table[i], New_coming_event->getEvent_time()+Propagation_delay, 
					SYNC_size,-1,New_coming_event->getEvent_time());
			}
			this->Timer_last_bit=New_coming_event->getEvent_time()+SYNC_size;
			delete[] Number_sum_id; delete[] Values_array_id; delete[] Number_sum_array; delete[] Values_array;
		}
	}
}

void Node::Receive_rebro_SYNC_Start(Event *handle, Event *New_coming_event)              //Event_type=6
{
	Head_print(6, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	for(int i=9; i>=0; i--)
	{
		if(this->SYNC_buffer[i]>0) {this->SYNC_buffer[i+1]=4; break; }
		if(i==0) this->SYNC_buffer[i]=4; 
	}
	int* SYNC=New_coming_event->getEvent_pointer(); int SYNC_size=New_coming_event->getEvent_pointer_size();
	Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+SYNC_size);
	future_event->setEvent_type(7); future_event->setEvent_node_id(this->Node_id); future_event->setEvent_pointer(SYNC);
	future_event->setEvent_pointer_size(SYNC_size); future_event->setTCPP(-11);
	handle->add(future_event);
	Process_print(7, this->Node_id, New_coming_event->getEvent_time()+SYNC_size, 
		SYNC_size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_rebro_SYNC_end(Event *handle, Event *New_coming_event)                 //Event_type=7
{
	Head_print(7, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* SYNC=New_coming_event->getEvent_pointer(); int SYNC_size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;      //check if collision-1
	for(int k=0; k<10; k++)
	{
		if(this->SYNC_buffer[k]>=4) kk++;
		if(this->SYNC_buffer[k]<=0) break;
	}
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-SYNC_size>this->Timer_last_bit)  //find receiver_id in RTS under the premise of non-error
	{
		int* Id_addition=new int [2]();
		Signal_detection(SYNC, SYNC_size, 0, Id_addition, 2);
		int Transmitter=Id_addition[0]; 
		int Sleep_time=Id_addition[1];

		//cout<<"SYNC_sleep_time1="<<Sleep_time1<<" "<<"SYNC_sleep_time2="<<Sleep_time2<<endl;
		for(int jj=0; jj<10; jj++)
			{if(this->SYNC_buffer[jj]==4) this->SYNC_buffer[jj]=-1; }
		long long Real_sleep_time=New_coming_event->getEvent_time()-SYNC_size-Propagation_delay+Sleep_time;

		//measurement
		long long* Parameter=new long long [2](); 
		Parameter[0]=this->Total_time_receiving;
		long long Time_receiving_before_SYNC=this->Total_time_receiving;
		Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, SYNC, SYNC_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; 
		this->SYNC_time_receiving=this->SYNC_time_receiving+this->Total_time_receiving-Time_receiving_before_SYNC;
		this->Last_moment_receiving=Parameter[1];

		delete[] SYNC; delete[] Id_addition; delete[] Parameter;
		for(int i=0; i<5; i++)  //put the received rebro SYNC (schedule) into list 
		{
			if(this->Schedule_table_node_id[0]<0) 
				{this->Schedule_table_node_id[0]=this->Node_id; this->Schedule_table_sleep_time[0]=Real_sleep_time; break; }
			if(this->Schedule_table_node_id[i]==Transmitter) {this->Schedule_table_sleep_time[i]=Real_sleep_time; break;}
			if(i==4 && this->Schedule_table_node_id[i]!=Transmitter) 
			{
				if(this->Schedule_table_sleep_time[0]!=Real_sleep_time) //check if the received SYNC is equal to its own SYNC
				{
					for(int j=0; j<5; j++)
					{
						if(this->Schedule_table_node_id[j]<0) 
						{this->Schedule_table_node_id[j]=Transmitter; this->Schedule_table_sleep_time[j]=Real_sleep_time; break; }
					}
				}
			}
		}
		//this part is probably useless.
		if(this->SYNC_buffer[0]==2)
		{
			//cout<<"This node is decided to rebro SYNC before, not this time."<<endl;
		}
		if(this->SYNC_buffer[0]<0 && this->response_SYNC_control==true)
		{
			Event* future_event=new Event; future_event->setEvent_time(New_coming_event->getEvent_time()+Processing_delay);  //go to rebroadcast_back_off
			future_event->setEvent_type(4); future_event->setEvent_node_id(this->Node_id); 
			future_event->setEvent_pointer(NULL); future_event->setEvent_pointer_size(0); future_event->setTCPP(-1500); 
			handle->add(future_event);
			
			Process_print(4, this->Node_id, New_coming_event->getEvent_time()+Processing_delay, 
				0,-1,New_coming_event->getEvent_time());
		}
		else     //maybe this SYNC_buffer[0]==1
		{
			//cout<<"This node does not need to rebro SYNC again."<<endl;
		}
	}
	else    //error    //receive more than one RTS   //already flush the buffer before
	{
		//measurement
		long long* Parameter=new long long [2](); 
		Parameter[0]=this->Total_time_receiving; 
		long long Time_receiving_before_SYNC=this->Total_time_receiving;
		Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, SYNC, SYNC_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; 
		this->SYNC_time_receiving=this->SYNC_time_receiving+this->Total_time_receiving-Time_receiving_before_SYNC;
		this->Last_moment_receiving=Parameter[1];

		delete[] SYNC; delete[] Parameter;
		for(int i=0; i<10; i++)
			{if(this->SYNC_buffer[i]>=4) {this->SYNC_buffer[i]=-1; break;	} }
		if(this->Schedule_table_sleep_time[0]==0 && this->Schedule_table_node_id[0]!=this->Node_id)
		{
			//cout<<"Node-"<<this->Node_id<<" cannot be synchronized state-7."<<endl; 
		}
		Process_print(7, -1, -1, -1,-1,New_coming_event->getEvent_time());
	}
}

void Node::Back_off_delay_RTS(Event *handle, Event *New_coming_event)    //Event_type=8
{
	Head_print(8, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int kk=0;    //in order to avoid the scenario: RTS_CTS_buffer: -1,5,-1,-1
	for(int i=0; i<10; i++)
		{if(this->RTS_CTS_buffer[i]>=4) kk++; }
	if(this->Queuing_system.size()!=0 && kk==0 && this->RTS_CTS_buffer[0]<=0 && New_coming_event->getEvent_time()>=this->Timer_last_bit )
	{
		if(this->Temp_of_TCPP!=this->Queuing_system.begin()->TCPP || this->Temp_of_TCPP_time!=this->Queuing_system.begin()->Time_in)
		{
			this->Time_of_bo=New_coming_event->getEvent_time();
			this->Temp_of_TCPP=this->Queuing_system.begin()->TCPP; this->Temp_of_TCPP_time=this->Queuing_system.begin()->Time_in;
		}

		this->Backoff_time=(rand()%(Back_off_main/Time_slot_RTS))*Time_slot_RTS;
		//cout<<"Please input the backoff time for RTS at node-"<<this->getNode_id()<<": ";
		//cin>>this->Backoff_time;

		Event* future_event33=new Event; future_event33->setEvent_time(New_coming_event->getEvent_time()+this->Backoff_time); 
		future_event33->setEvent_type(9); future_event33->setEvent_node_id(this->Node_id); 
		future_event33->setEvent_pointer(NULL); future_event33->setEvent_pointer_size(0); future_event33->setTCPP(-29); 
		handle->add(future_event33);
		Process_print(9, this->Node_id, New_coming_event->getEvent_time()+this->Backoff_time, 0,-1,New_coming_event->getEvent_time());
	}
}
void Node::Send_RTS(Event *handle, Event *New_coming_event)     //event_type=9
{
	Head_print(9, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int kk=0;    //in order to avoid the scenario: RTS_CTS_buffer: -1,5,-1,-1
	for(int i=0; i<10; i++)
		{if(this->RTS_CTS_buffer[i]>=4) kk++; }
	if(this->radio_on==true && this->Queuing_system.size()!=0 && kk==0 && this->RTS_CTS_buffer[0]<=0 
		&& New_coming_event->getEvent_time()>this->Timer_last_bit)
	{//put the this->Timer in the above sentence, just for the scenario that receiver can be both receiver and sender
		/*cout<<"this->neighbor_table[0]: "; cin>>this->Neighbor_table[0]; cout<<"this->neighbor_table[1]: "; cin>>this->Neighbor_table[1];  //for debugging code*/
		if(this->Timer_event!=NULL)  //just for debug
			{cout<<endl<<"Error code-1 State-1 at"<<New_coming_event->getEvent_time()<<endl; this->Timer_event->setFlag_event_point(1); }
		this->RTS_CTS_buffer[0]=1;  //maybe this node is passive receiver, or is transmitter, we will decide it in the future
		
		int* Number_sum_id_transmitter=new int [2](); int* Values_array_id_transmitter=new int [5]();
		Digits_check_id(Number_sum_id_transmitter, Values_array_id_transmitter, this->Node_id);
		int* Number_sum_id_receiver=new int [2](); int* Values_array_id_receiver=new int [5]();
		Digits_check_id(Number_sum_id_receiver, Values_array_id_receiver, this->Neighbor_table[0]);
		
		int RTS_size=5+4+2*Number_sum_id_transmitter[0]+1+Number_sum_id_transmitter[1]+2*Number_sum_id_receiver[0]+1+Number_sum_id_receiver[1];

		for(int i=0; i<10; i++)
		{
			if(this->Neighbor_table[i]<0) break;
			int* RTS=new int [RTS_size]();
			Singal_construction(RTS, RTS_size, 1, Values_array_id_transmitter, Number_sum_id_transmitter, 
				Values_array_id_receiver, Number_sum_id_receiver, NULL, NULL, 0);
			
			Event* future_event2=new Event;
		    future_event2->setEvent_time(New_coming_event->getEvent_time()+Propagation_delay);
		    future_event2->setEvent_type(10); future_event2->setEvent_node_id(this->Neighbor_table[i]); 
			future_event2->setEvent_pointer(RTS); future_event2->setEvent_pointer_size(RTS_size); 
			future_event2->setTCPP(-3); 
			handle->add(future_event2); 

			if(i==0) //measurement
			{
				this->Total_RTS=this->Total_RTS+1;
				int N=Calculate_Pulse_count(RTS,RTS_size);
				this->Total_time_interval=this->Total_time_interval+RTS_size-N;
				this->Data_tran_interval=this->Data_tran_interval+RTS_size-N;
				this->Pulse_count=this->Pulse_count+N;
				this->Pulse_count_non_SYNC=this->Pulse_count_non_SYNC+N;
			}
			Process_print(10, this->Neighbor_table[i], New_coming_event->getEvent_time()+Propagation_delay, 
				RTS_size, -1,New_coming_event->getEvent_time());
        }
		this->Timer_last_bit=New_coming_event->getEvent_time()+RTS_size;
		delete[] Number_sum_id_transmitter; delete[] Values_array_id_transmitter; 
		delete[] Number_sum_id_receiver; delete[] Values_array_id_receiver;
	}
}

void Node::Receive_RTS_start(Event *handle, Event *New_coming_event) //event_type=10
{
	Head_print(10, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* RTS=New_coming_event->getEvent_pointer(); int RTS_size=New_coming_event->getEvent_pointer_size();
	for(int i=9; i>=0; i--)
	{ 
		if(this->RTS_CTS_buffer[i]>0) {this->RTS_CTS_buffer[i+1]=4; break; }
	    if(i==0) this->RTS_CTS_buffer[i]=4;
	}
	Event* future_event16=new Event; future_event16->setEvent_time(New_coming_event->getEvent_time()+RTS_size);
	future_event16->setEvent_type(11); future_event16->setEvent_node_id(this->Node_id); future_event16->setEvent_pointer(RTS);
	future_event16->setEvent_pointer_size(RTS_size); future_event16->setTCPP(-11);
	handle->add(future_event16);
	Process_print(11, this->Node_id, New_coming_event->getEvent_time()+RTS_size, 
		RTS_size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_RTS_end_send_CTS(Event *handle, Event *New_coming_event)   //event_type=11
{
	Head_print(11, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* RTS=New_coming_event->getEvent_pointer(); int RTS_size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;
	for(int k=0; k<10; k++)
	{
		if(this->RTS_CTS_buffer[k]>=4) kk++;
		if(this->RTS_CTS_buffer[k]<=0) break;
	}
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-RTS_size>this->Timer_last_bit)        
	{
		for(int jj=0; jj<10; jj++)
			{if(this->RTS_CTS_buffer[jj]==4) this->RTS_CTS_buffer[jj]=-1; }

		int* Id_addition=new int [2]();
		Signal_detection(RTS, RTS_size, 1, Id_addition, 2);
		int Transmitter=Id_addition[0];
		int Receiver=Id_addition[1];

		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, RTS, RTS_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];

		delete[] RTS; delete[] Id_addition; delete[] Parameter;
		if(this->Node_id==Receiver)            //become a real receiver  
		{
			if(this->RTS_CTS_buffer[0]<=0 || (this->RTS_CTS_buffer[0]==3 && this->Transmitter_id<0) ||
				(this->RTS_CTS_buffer[0]==3 && this->Transmitter_id==Transmitter) )
			{
				if(this->Timer_event!=NULL)      //cancel old timer
					{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
				this->Transmitter_id=Transmitter; this->Receiver_id=Receiver;
				this->RTS_CTS_buffer[0]=2;
				
				int* Number_sum_id_transmitter=new int [2](); int* Values_array_id_transmitter=new int [5]();
				Digits_check_id(Number_sum_id_transmitter, Values_array_id_transmitter, this->Transmitter_id);

				int CTS_size=5+5+2*Number_sum_id_transmitter[0]+1+Number_sum_id_transmitter[1];

				for(int i=0; i<10; i++)           //broadcast CTS
				{
					if(this->Neighbor_table[i]<0) break;
					int* CTS=new int [CTS_size]();
					Singal_construction(CTS, CTS_size, 2, Values_array_id_transmitter, Number_sum_id_transmitter, NULL, NULL, NULL, NULL, 0);
					
					Event* future_event3=new Event;
					future_event3->setEvent_time(New_coming_event->getEvent_time()+Processing_delay+Propagation_delay);
					future_event3->setEvent_type(12); future_event3->setEvent_node_id(this->Neighbor_table[i]); 
					future_event3->setEvent_pointer(CTS); future_event3->setEvent_pointer_size(CTS_size);
					future_event3->setTCPP(-4); 
					handle->add(future_event3);

					Process_print(12, this->Neighbor_table[i], New_coming_event->getEvent_time()
						+Processing_delay+Propagation_delay, CTS_size, 1, New_coming_event->getEvent_time());
					
					if(i==0) //measurement
					{   
						this->Total_CTS=this->Total_CTS+1;
					    int N=Calculate_Pulse_count(CTS,CTS_size);
						this->Total_time_interval=this->Total_time_interval+CTS_size-N;
						this->Data_tran_interval=this->Data_tran_interval+CTS_size-N;
						this->Pulse_count=this->Pulse_count+N;
						this->Pulse_count_non_SYNC=this->Pulse_count_non_SYNC+N;
					}
				}

				if(this->Timer_event_radio_off!=NULL)   //Timer-1
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				Event* future_event8=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()
					+Processing_delay+Propagation_delay
					+CTS_size+Processing_delay+Propagation_delay
					+length_TCPPs_max+1;
				future_event8->setEvent_time(Time_for_timer);
				future_event8->setEvent_type(20); future_event8->setEvent_node_id(this->Node_id); 
				future_event8->setEvent_pointer(NULL); future_event8->setEvent_pointer_size(0);
				future_event8->setTCPP(-100); 
				handle->add(future_event8);

				this->Timer_event=future_event8;
				this->Timer_last_bit=New_coming_event->getEvent_time()+Processing_delay+CTS_size;
				Process_print(20, this->Node_id, Time_for_timer, 0, 1, New_coming_event->getEvent_time());
				
				delete[] Number_sum_id_transmitter; delete[] Values_array_id_transmitter;
			}
			else   //already receive the RTS and send a CTS before, set a long timer. Then receive RTS from this same node again, 
			{
				if(this->RTS_CTS_buffer[0]==2 && this->Transmitter_id==Transmitter && this->Receiver_id==Receiver
					&& New_coming_event->getEvent_time()-RTS_size>this->Timer_last_bit)
				{
					int* Number_sum_id_transmitter=new int [2](); int* Values_array_id_transmitter=new int [5]();
					Digits_check_id(Number_sum_id_transmitter, Values_array_id_transmitter, this->Transmitter_id);
					
					int CTS_size=5+5+2*Number_sum_id_transmitter[0]+1+Number_sum_id_transmitter[1];

					for(int i=0; i<10; i++)           //broadcast CTS
					{   
						if(this->Neighbor_table[i]<0) break;
						int* CTS=new int [CTS_size]();
						Singal_construction(CTS, CTS_size, 2, Values_array_id_transmitter, Number_sum_id_transmitter, NULL, NULL, NULL, NULL, 0);
					    
						Event* future_event3=new Event;
					    future_event3->setEvent_time(New_coming_event->getEvent_time()+Processing_delay+Propagation_delay);
					    future_event3->setEvent_type(12); future_event3->setEvent_node_id(this->Neighbor_table[i]); 
						future_event3->setEvent_pointer(CTS); future_event3->setEvent_pointer_size(CTS_size);
					    future_event3->setTCPP(-4);
					    handle->add(future_event3);

						Process_print(12, this->Neighbor_table[i], New_coming_event->getEvent_time()+Processing_delay+Propagation_delay,
							CTS_size, 11, New_coming_event->getEvent_time());
						
						if(i==0) //measurement
						{   
							this->Total_CTS=this->Total_CTS+1;
							int N=Calculate_Pulse_count(CTS,CTS_size);
							this->Total_time_interval=this->Total_time_interval+CTS_size-N;
							this->Data_tran_interval=this->Data_tran_interval+CTS_size-N;
							this->Pulse_count=this->Pulse_count+N;
							this->Pulse_count_non_SYNC=this->Pulse_count_non_SYNC+N;
						}
					}
					if(this->Timer_event_radio_off!=NULL)     //Timer-1
						{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
					Event* future_event8=new Event;
					long long Time_for_timer=New_coming_event->getEvent_time()
						+Processing_delay+Processing_delay
						+CTS_size+Processing_delay+Propagation_delay
						+length_TCPPs_max+1;
					future_event8->setEvent_time(Time_for_timer);
					future_event8->setEvent_type(20); future_event8->setEvent_node_id(this->Node_id); 
					future_event8->setEvent_pointer(NULL); future_event8->setEvent_pointer_size(0);
					future_event8->setTCPP(-100); 
					handle->add(future_event8);

					this->Timer_event=future_event8;
					this->Timer_last_bit=New_coming_event->getEvent_time()+Processing_delay+CTS_size;
					Process_print(20, this->Node_id, Time_for_timer, 0, 2, New_coming_event->getEvent_time());

					delete[] Number_sum_id_transmitter; delete[] Values_array_id_transmitter;
				}
			}        
		}
		else  //big change             
			  //real receiver, real transmitter, competing transmitter, all three of them can receive RTS, 
		{
			if(this->Transmitter_id>=0 && this->Receiver_id>=0)
			{}
			else
			{
				this->RTS_CTS_buffer[0]=3;   //2015.06.06
				if(this->Timer_event_radio_off!=NULL)     //Timer-1
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				Event* future_event9=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()+Processing_delay;
				future_event9->setEvent_time(Time_for_timer);
				future_event9->setEvent_type(20); future_event9->setEvent_node_id(this->Node_id); 
				future_event9->setEvent_pointer(NULL); future_event9->setEvent_pointer_size(0);
				future_event9->setTCPP(-100); 
				handle->add(future_event9);

				this->Timer_event=future_event9;
				Process_print(20, this->Node_id, Time_for_timer, 0, 3, New_coming_event->getEvent_time());
			}
		}//passive node code
	}
	else    //error    //receive more than one RTS   //already flush the buffer before
	{
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, RTS, RTS_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];

		delete[] RTS; delete[] Parameter;
		for(int i=0; i<10; i++)
			{if(this->RTS_CTS_buffer[i]>=4) {this->RTS_CTS_buffer[i]=-1; break;	} }
		Process_print(11, this->Node_id, -1, -1,-1,New_coming_event->getEvent_time());
	}
}

void Node::Receive_CTS_start(Event *handle, Event *New_coming_event)   //event_type=12
{
	Head_print(12, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* CTS=New_coming_event->getEvent_pointer();
	int CTS_size=New_coming_event->getEvent_pointer_size();
	for(int i=9; i>=0; i--)
	{
		if(this->RTS_CTS_buffer[i]>0) {this->RTS_CTS_buffer[i+1]=4; break; }
		if(i==0) this->RTS_CTS_buffer[i]=4;
	}
	Event* future_event18=new Event; future_event18->setEvent_time(New_coming_event->getEvent_time()+CTS_size);
	future_event18->setEvent_type(13); future_event18->setEvent_node_id(this->Node_id); future_event18->setEvent_pointer(CTS);
	future_event18->setEvent_pointer_size(CTS_size); future_event18->setTCPP(-13);
	handle->add(future_event18);
	Process_print(13, this->Node_id, New_coming_event->getEvent_time()+CTS_size, 
		CTS_size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_CTS_end_Send_TCPP(Event *handle, Event *New_coming_event)   //event_type=13
{
	Head_print(13, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* CTS=New_coming_event->getEvent_pointer(); int CTS_size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;
	for(int k=0; k<10; k++)
	{
		if(this->RTS_CTS_buffer[k]>=4) kk++; 
		if(this->RTS_CTS_buffer[k]<=0) break;
	}
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-CTS_size>this->Timer_last_bit)
	{   
		for(int jj=0; jj<10; jj++)
			{if(this->RTS_CTS_buffer[jj]==4) this->RTS_CTS_buffer[jj]=-1; }
		
		int* Id_addition=new int [2]();
		Signal_detection(CTS, CTS_size, 2, Id_addition, 2);
		int Transmitter=Id_addition[0];
		
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, CTS, CTS_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];
		
		delete[] CTS; delete[] Id_addition; delete[] Parameter;
		if(this->Node_id==Transmitter && (this->RTS_CTS_buffer[0]==1 || this->RTS_CTS_buffer[0]==3 || 
			(this->RTS_CTS_buffer[0]<0 && this->Transmitter_id<0 && this->Receiver_id<0) ) )
			//(this->RTS_CTS_buffer[0]<0 && this->Transmitter_id==0 && this->Receiver_id==0) this part is just for the scenario: after
			//go through error signal, this node has flushed its everything
		{	//become a real receiver, should do interleaving in this area
			if(this->RTS_CTS_buffer[0]==3 && this->Transmitter_id>=0)
				//already receive a CTS before, so in this time the node just igores this CTS
			{
				cout<<"CTS_will_ignore"<<endl;
			}
			else
			{
				this->RTS_CTS_buffer[0]=1;        //1: transmitter,   2: receiver,   3: passive receiver
				this->Transmitter_id=Transmitter; this->Receiver_id=this->Neighbor_table[0];

				int* Number_sum_id_transmitter=new int [2](); int* Values_array_id_transmitter=new int [5]();
				Digits_check_id(Number_sum_id_transmitter, Values_array_id_transmitter, this->Transmitter_id);
				int* Number_sum_array=new int [2](); int* Values_array=new int [30]();
				Digits_check_big_data(Number_sum_array, Values_array, this->Queuing_system);
				
				int m=5+6+2*Number_sum_id_transmitter[0]+1+Number_sum_id_transmitter[1]
						+2*Number_sum_array[0]+No_of_data_field+Number_sum_array[1];
				int* arrayTotal=new int [m]();

				for(int nn=0; nn<10; nn++)
				{
					if(this->Neighbor_table[nn]<0) break;
					if(nn==0)
					{
						Interleaving_cluster(arrayTotal, m, Values_array_id_transmitter, Number_sum_id_transmitter, 
							Values_array, Number_sum_array[0]);
						this->Interleaving_num=No_of_data_field;

						Event* future_event4=new Event;
						future_event4->setEvent_time(New_coming_event->getEvent_time()+Processing_delay+Propagation_delay);
						future_event4->setEvent_type(14);                       //receiver wants to receive data
						future_event4->setEvent_node_id(this->Neighbor_table[nn]); future_event4->setEvent_pointer(arrayTotal);
						future_event4->setEvent_pointer_size(m); 
						future_event4->setTCPP(-5);
						handle->add(future_event4);

						Process_print(14, this->Neighbor_table[nn], New_coming_event->getEvent_time()+Processing_delay+Propagation_delay, 
							m, 1, New_coming_event->getEvent_time());
								
						
						//measurement
						this->Total_times_tran=this->Total_times_tran+1;
						this->Total_num_tran_TCPP=this->Total_num_tran_TCPP+this->Interleaving_num;
						int N=Calculate_Pulse_count(arrayTotal,m);
						this->Total_time_interval=this->Total_time_interval+m-N;
						this->Data_tran_interval=this->Data_tran_interval+m-N;
						this->Pulse_count=this->Pulse_count+N;
						this->Pulse_count_non_SYNC=this->Pulse_count_non_SYNC+N;

					}  //if(nn==0)
					else     //send data to other non-real receiver, nn>=1
					{   int* other_arrayTotal= new int [m]();
						for(int mm=0; mm<m; mm++)
							{if(arrayTotal[mm]==1) other_arrayTotal[mm]=arrayTotal[mm]; }
						Event* future_event11=new Event;
						future_event11->setEvent_time(New_coming_event->getEvent_time()+Processing_delay+Propagation_delay);
						future_event11->setEvent_type(14);                       //receiver wants to receive data
						future_event11->setEvent_node_id(this->Neighbor_table[nn]); future_event11->setEvent_pointer(other_arrayTotal);
						future_event11->setEvent_pointer_size(m);
						future_event11->setTCPP(-6);
						handle->add(future_event11);

						Process_print(14, this->Neighbor_table[nn], New_coming_event->getEvent_time()+Processing_delay+Propagation_delay, 
									m,11,New_coming_event->getEvent_time());
					}
				}
				if(this->Timer_event_radio_off!=NULL)//Timer-2
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				Event* future_event10=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()
					+Processing_delay+Propagation_delay
					+m+Processing_delay+Propagation_delay
					+Maximum_EOT+1;
				future_event10->setEvent_time(Time_for_timer); 
		   		future_event10->setEvent_type(20); future_event10->setEvent_node_id(this->Node_id); 
				future_event10->setEvent_pointer(NULL); future_event10->setEvent_pointer_size(0);
				future_event10->setTCPP(-300);
				handle->add(future_event10);
				this->Timer_event=future_event10;
				this->Timer_last_bit=New_coming_event->getEvent_time()+Processing_delay+m;

				Process_print(20, this->Node_id, Time_for_timer, 0, 3, New_coming_event->getEvent_time());
				delete[] Number_sum_id_transmitter; delete[] Values_array_id_transmitter; 
				delete[] Number_sum_array; delete[] Values_array;
			}
		}
		else  //become a passive receiver, can receive CTS, long timer(length_TCPPs_max)
		{
			if( (this->RTS_CTS_buffer[0]==1 || this->RTS_CTS_buffer[0]==2) && this->Transmitter_id>=0 && this->Receiver_id>=0)
				//already become a real transmitter or a real receiver, so in this time the node just igores this CTS
			{
				Process_print(-1, -1, -1, -1,2,New_coming_event->getEvent_time());
			}
			else
			{
				this->RTS_CTS_buffer[0]=3;   //2015.06.06
				if(this->Timer_event_radio_off!=NULL)     //Timer-1
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				if(this->Timer_event!=NULL)     //Timer-1
					{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
				Event* future_event9=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()+Processing_delay;
				future_event9->setEvent_time(Time_for_timer);
				future_event9->setEvent_type(20); future_event9->setEvent_node_id(this->Node_id); 
				future_event9->setEvent_pointer(NULL); future_event9->setEvent_pointer_size(0);
				future_event9->setTCPP(-100); 
				handle->add(future_event9);

				this->Timer_event=future_event9;
				Process_print(20, this->Node_id, Time_for_timer, 0, 4, New_coming_event->getEvent_time());
			}
		}
	}
	else   //error happens   //receive CTS and sth else at the same time
	{
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, CTS, CTS_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];

		delete[] CTS; delete[] Parameter;
		for(int i=0; i<10; i++)
			{if(this->RTS_CTS_buffer[i]>=4) {this->RTS_CTS_buffer[i]=-1; break;	} }
		Process_print(-1, -1, -1, -1,-1,New_coming_event->getEvent_time());
	}
}

void Node::Receive_TCPP_start(Event *handle, Event *New_coming_event) //event_type=14
{
	Head_print(14, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* TCPP_Data=New_coming_event->getEvent_pointer();
	int TCPP_Size=New_coming_event->getEvent_pointer_size();
	for(int i=9; i>=0; i--)
	{
		if(this->RTS_CTS_buffer[i]>0) {this->RTS_CTS_buffer[i+1]=4; break; }
		if(i==0) this->RTS_CTS_buffer[i]=4;
	}
	Event* future_event20=new Event;
    future_event20->setEvent_time(New_coming_event->getEvent_time()+TCPP_Size);
	future_event20->setEvent_type(15);                    //current node wants to receive the end of RTS
	future_event20->setEvent_node_id(this->Node_id); future_event20->setEvent_pointer(TCPP_Data);
	future_event20->setEvent_pointer_size(TCPP_Size);
	future_event20->setTCPP(-15);
	handle->add(future_event20);
	Process_print(15, this->Node_id, New_coming_event->getEvent_time()+TCPP_Size, 
		TCPP_Size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_TCPP_end_send_EOT(Event *handle, Event *New_coming_event)   //event_type=15
{
	Head_print(15, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

    int* TCPP_Data=New_coming_event->getEvent_pointer(); int TCPP_Size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;
	for(int k=0; k<10; k++)
	{
		if(this->RTS_CTS_buffer[k]>=4) kk++; 
		if(this->RTS_CTS_buffer[k]<=0) break;
	}
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-TCPP_Size>this->Timer_last_bit)
	{
		for(int jj=0; jj<10; jj++) 
			{if(this->RTS_CTS_buffer[jj]==4) this->RTS_CTS_buffer[jj]=-1; }
		
		int* Id_addition=new int [2]();
		Signal_detection(TCPP_Data, TCPP_Size, 3, Id_addition, 2);
		int Transmitter=Id_addition[0];

		if(this->Node_id==this->Receiver_id && this->Transmitter_id==Transmitter) 
		{
			deque<TCPP_pair> Queuing_system_temporary;
			int NN=Decoding_cluster(Queuing_system_temporary, TCPP_Data, TCPP_Size, Transmitter, this->Node_id, New_coming_event->getEvent_time());
			this->Total_num_rece_TCPP=this->Total_num_rece_TCPP+NN;

			deque<TCPP_pair>::iterator TCPP_point_rece, TCPP_point_tran;     //calculate the Waiting_delay and Access_delay
			Node *Sender=getNode_from_id(this->Transmitter_id);
			
			for(TCPP_point_rece=Queuing_system_temporary.begin(), TCPP_point_tran=Sender->Queuing_system.begin(); 
				TCPP_point_rece!=Queuing_system_temporary.end(); TCPP_point_tran++, TCPP_point_rece++)
			{
				if(TCPP_point_rece==Queuing_system_temporary.begin() )
				{
					if(Sender->getLoop_access_delay()!=0 || Sender->getLoop_waiting_delay()!=0)  
						{Sender->setLoop_waiting_delay(0); Sender->setLoop_access_delay(0);}
				}
				if(TCPP_point_rece->TCPP==TCPP_point_tran->TCPP)
				{
					if(TCPP_point_tran->Time_in<=Sender->getTime_of_bo() )
					{
						Sender->setLoop_waiting_delay(Sender->getLoop_waiting_delay()+Sender->getTime_of_bo()-TCPP_point_tran->Time_in);
						Sender->setLoop_access_delay(Sender->getLoop_access_delay()+TCPP_point_rece->Time_in-Sender->getTime_of_bo() );
					}
					else
					{
						Sender->setLoop_waiting_delay(Sender->getLoop_waiting_delay()+0);
						Sender->setLoop_access_delay(Sender->getLoop_access_delay()+TCPP_point_rece->Time_in-TCPP_point_tran->Time_in );
					}
					//Sender->setLoop_delay(Sender->getLoop_delay()+TCPP_point_rece->Time_in-TCPP_point_tran->Time_in);
				}
				else
					{cout<<endl<<"Error code State-15.1 at"<<New_coming_event->getEvent_time()<<endl; }
			}
			Queuing_system_temporary.clear();
		}
		
		//measurement
		long long* Parameter=new long long [2](); 
		Parameter[0]=this->Total_time_receiving; 
		Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, TCPP_Data, TCPP_Size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; 
		this->Last_moment_receiving=Parameter[1];

		delete[] TCPP_Data; delete[] Id_addition; delete[] Parameter;
		if(this->Node_id==this->Receiver_id && this->Transmitter_id==Transmitter)
		{
			int* Number_sum_id_transmitter=new int [2](); int* Values_array_id_transmitter=new int [5]();
			Digits_check_id(Number_sum_id_transmitter, Values_array_id_transmitter, this->Transmitter_id);
			
			int EOT_size=5+7+2*Number_sum_id_transmitter[0]+1+Number_sum_id_transmitter[1];

			for(int n=0; n<10; n++)                     //broadcast EOT
			{   
				if(this->Neighbor_table[n]<0) break;
				int* EOT=new int [EOT_size]();
				Singal_construction(EOT, EOT_size, 4, Values_array_id_transmitter, Number_sum_id_transmitter, NULL, NULL, NULL, NULL, 0);
				
				Event* future_event5=new Event;
				future_event5->setEvent_time(New_coming_event->getEvent_time()+Processing_delay+Propagation_delay);
				future_event5->setEvent_type(16); future_event5->setEvent_node_id(this->Neighbor_table[n]);  
				future_event5->setEvent_pointer(EOT); future_event5->setEvent_pointer_size(EOT_size); 
				future_event5->setTCPP(-7);
				handle->add(future_event5);

				Process_print(16, this->Neighbor_table[n], New_coming_event->getEvent_time()
					+Processing_delay+Propagation_delay, EOT_size, -1, New_coming_event->getEvent_time() );
				
				if(n==0) //measurement
				{
					this->Total_EOT=this->Total_EOT+1;
					int N=Calculate_Pulse_count(EOT,EOT_size);
					this->Total_time_interval=this->Total_time_interval+EOT_size-N;
					this->Data_tran_interval=this->Data_tran_interval+EOT_size-N;
					this->Pulse_count=this->Pulse_count+N;
					this->Pulse_count_non_SYNC=this->Pulse_count_non_SYNC+N;
				}
			}
			this->Timer_last_bit=New_coming_event->getEvent_time()+Processing_delay+EOT_size;
			if(this->Timer_event!=NULL)    //cancel old timer
				{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL;}
			this->Receiver_id=-1; this->Transmitter_id=-1;
			for(int i=0; i<10; i++) 
				this->RTS_CTS_buffer[i]=-1;

			Event* future_event3=new Event;
			future_event3->setEvent_time(New_coming_event->getEvent_time()+Processing_delay);
			future_event3->setEvent_type(20); future_event3->setEvent_node_id(this->Node_id); 
			future_event3->setEvent_pointer(NULL); future_event3->setEvent_pointer_size(0); future_event3->setTCPP(-4);
			handle->add(future_event3);
			this->Timer_event_radio_off=future_event3;
			Process_print(20, this->Node_id, New_coming_event->getEvent_time()+Processing_delay, 0,-1,New_coming_event->getEvent_time());
			
			delete[] Number_sum_id_transmitter; delete[] Values_array_id_transmitter;
		}
		else       //passive receiver receives data. 
		{
			if( (this->RTS_CTS_buffer[0]==1 || this->RTS_CTS_buffer[0]==2) && this->Transmitter_id>=0 && this->Receiver_id>=0)
				//already become a real transmitter or a real receiver, so in this time the node just igores this CTS
			{
				Process_print(-1, -1, -1, -1, 2, New_coming_event->getEvent_time());
			}
			else
			{
				this->RTS_CTS_buffer[0]=3;  //2015.06.06
				if(this->Timer_event_radio_off!=NULL)     //Timer-1
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				if(this->Timer_event!=NULL)     //Timer-1
					{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
				Event* future_event9=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()+Processing_delay;
				future_event9->setEvent_time(Time_for_timer);
				future_event9->setEvent_type(20); future_event9->setEvent_node_id(this->Node_id); 
				future_event9->setEvent_pointer(NULL); future_event9->setEvent_pointer_size(0);
				future_event9->setTCPP(-100); 
				handle->add(future_event9);

				this->Timer_event=future_event9;
				Process_print(20, this->Node_id, Time_for_timer, 0, 4, New_coming_event->getEvent_time());
			}
		}
	}
	else          //data gets collision
	{
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, TCPP_Data, TCPP_Size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];

		delete[] TCPP_Data; delete[] Parameter;

		for(int i=0; i<10; i++)
			{if(this->RTS_CTS_buffer[i]>=4) {this->RTS_CTS_buffer[i]=-1; break;	} }
		Process_print(15, this->Node_id, -1, -1,-1,New_coming_event->getEvent_time());
	}
}

void Node::Receive_EOT_Start(Event *handle, Event *New_coming_event)  //event_type=16
{
	Head_print(16, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	int* EOT=New_coming_event->getEvent_pointer();
	int EOT_size=New_coming_event->getEvent_pointer_size();
	for(int i=9; i>=0; i--)
	{ 
		if(this->RTS_CTS_buffer[i]>0) {this->RTS_CTS_buffer[i+1]=4; break;}  
		if(i==0) this->RTS_CTS_buffer[i]=4;
	}
	Event* future_event21=new Event; future_event21->setEvent_time(New_coming_event->getEvent_time()+EOT_size);
	future_event21->setEvent_type(17); future_event21->setEvent_node_id(this->Node_id); future_event21->setEvent_pointer(EOT);
	future_event21->setEvent_pointer_size(EOT_size);
	future_event21->setTCPP(-16);
	handle->add(future_event21);
	Process_print(17, this->Node_id, New_coming_event->getEvent_time()+EOT_size, 
		EOT_size,-1,New_coming_event->getEvent_time());
}

void Node::Receive_EOT_end(Event *handle, Event *New_coming_event)      //event_type=17
{
	Head_print(17, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

    int* EOT=New_coming_event->getEvent_pointer(); int EOT_size=New_coming_event->getEvent_pointer_size();
	//check out error in the future
	int kk=0;
	for(int k=0; k<10; k++)
	{
		if(this->RTS_CTS_buffer[k]>=4) kk++;
		if(this->RTS_CTS_buffer[k]<=0) break;
	}
	//check out error in the future
	if(kk==1 && this->radio_on==true && New_coming_event->getEvent_time()-EOT_size>this->Timer_last_bit)
	{   
		for(int jj=0; jj<10; jj++)
			{if(this->RTS_CTS_buffer[jj]==4) this->RTS_CTS_buffer[jj]=-1; }
		
		int* Id_addition=new int [2]();
		Signal_detection(EOT, EOT_size, 4, Id_addition, 2);
		int Transmitter=Id_addition[0];
		
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, EOT, EOT_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];
		
		delete[] EOT; delete[] Id_addition; delete[] Parameter;
		if(this->Node_id==Transmitter)         
		{
			//successful transmission
			this->Succe_num_tran_TCPP=this->Succe_num_tran_TCPP+this->Interleaving_num;
			this->Succe_timers_tran=this->Succe_timers_tran+1;
			this->Waiting_delay=this->Waiting_delay+this->Loop_waiting_delay;
			this->Access_delay=this->Access_delay+this->Loop_access_delay;
			this->Loop_waiting_delay=0; this->Loop_access_delay=0;

			for(; this->Interleaving_num>0; this->Interleaving_num--) //cancel the corresponding no. of TCPPs from the queue
			{
				if(this->Queuing_system.size()==0)
					cout<<endl<<"Error code State-17.1 at "<<New_coming_event->getEvent_time()<<endl;  //just for test
				this->Queuing_system.pop_front(); 
			}
				
			Choose_receiver(this->Neighbor_table, 10);
			this->Transmitter_id=-1; this->Receiver_id=-1;
			for(int i=0; i<10; i++)
				this->RTS_CTS_buffer[i]=-1;
			if(this->Timer_event!=NULL)                                //cancel old timer
				{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL;}

			Event* future_event7=new Event; future_event7->setEvent_time(New_coming_event->getEvent_time()+Processing_delay);   
			future_event7->setEvent_type(20); future_event7->setEvent_node_id(this->Node_id); 
			future_event7->setEvent_pointer(NULL); future_event7->setEvent_pointer_size(0); future_event7->setTCPP(-8);
			handle->add(future_event7);
			this->Timer_event_radio_off=future_event7;
			Process_print(20, this->Node_id, New_coming_event->getEvent_time()+Processing_delay, 
				0, 1, New_coming_event->getEvent_time());
		}
		else     //passive receiver got EOT
		{
			if( (this->RTS_CTS_buffer[0]==1 || this->RTS_CTS_buffer[0]==2) && this->Transmitter_id>=0 && this->Receiver_id>=0)
				//already become a real transmitter or a real receiver, so in this time the node just igores this CTS
			{
				Process_print(-1, -1, -1, -1, 2, New_coming_event->getEvent_time());
			}
			else
			{
				this->RTS_CTS_buffer[0]=3;  //2015.06.06
				if(this->Timer_event_radio_off!=NULL)     //Timer-1
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				if(this->Timer_event!=NULL)     //Timer-1
					{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL; }
				Event* future_event9=new Event;
				long long Time_for_timer=New_coming_event->getEvent_time()+Processing_delay;
				future_event9->setEvent_time(Time_for_timer);
				future_event9->setEvent_type(20); future_event9->setEvent_node_id(this->Node_id); 
				future_event9->setEvent_pointer(NULL); future_event9->setEvent_pointer_size(0);
				future_event9->setTCPP(-100); 
				handle->add(future_event9);

				this->Timer_event=future_event9;
				Process_print(20, this->Node_id, Time_for_timer, 0, 4, New_coming_event->getEvent_time());
			}
		}//passive receiver got EOT
	}
	else        //error happened
	{
		//measurement
		long long* Parameter=new long long [2](); Parameter[0]=this->Total_time_receiving; Parameter[1]=this->Last_moment_receiving;
		Receiving_time_calculation(Parameter, EOT, EOT_size, this->radio_on, this->Radio_off_time, 
			this->Last_moment_receiving, New_coming_event->getEvent_time());
		this->Total_time_receiving=Parameter[0]; this->Last_moment_receiving=Parameter[1];

		delete[] EOT; delete[] Parameter;
		for(int i=0; i<10; i++)
			{if(this->RTS_CTS_buffer[i]>=4) {this->RTS_CTS_buffer[i]=-1; break;	} }
		Process_print(17, this->Node_id, -1, -1,-1,New_coming_event->getEvent_time());
	}
}

void Node::TCPP_insert_queue(Event *handle, Event *New_coming_event)       //Event_type=19
{
	int TCPP_new=New_coming_event->getTCPP();
	if(TCPP_new>=0 )
	{
		for(int i=0; i<No_of_data_field; i++)
		{
			TCPP_pair element1;
			element1.TCPP=rand()%(Value_TCPP_Max+1);
			element1.Time_in=New_coming_event->getEvent_time(); 
			this->Queuing_system.push_back(element1); 
		}
	}
	//Process_print(19, this->Node_id, New_coming_event->getEvent_time(), 0,-1,New_coming_event->getEvent_time());
}

void Node::Sleep_mode(Event *handle, Event *New_coming_event)                   //Event_type=20
{
	Head_print(20, this->Node_id, New_coming_event->getEvent_time(), this->SYNC_buffer[0], this->SYNC_buffer[1], this->SYNC_buffer[2], this->Transmitter_id,
		this->Receiver_id, this->RTS_CTS_buffer[0], this->RTS_CTS_buffer[1], this->Interleaving_num, this->response_SYNC_control);

	if(this->Timer_event_radio_off==New_coming_event && this->radio_on==false)  //just for debug
		cout<<"Error code State-20 Node-"<<this->Node_id<<" at "<<New_coming_event->getEvent_time();

	//if(New_coming_event->getEvent_time()>5000) //just for debug
	//	Print_schedule_table(this->Schedule_table_node_id, 5, this->Schedule_table_sleep_time, 5);

	if(this->Timer_event_radio_off!=NULL)
	{
		if(this->Timer_event_radio_off==New_coming_event)
		{//section-1
			if((this->RTS_CTS_buffer[0]==1 || this->RTS_CTS_buffer[0]==2) && this->Transmitter_id>=0 && this->Receiver_id>=0)
				{this->Timer_event_radio_off=NULL; }
			else
			{
				for(int j=0; j<5; j++)
				{
					if(New_coming_event->getEvent_time()<=Initial_listen_period
						&& this->Schedule_table_sleep_time[j]>0)
						{this->Schedule_table_sleep_time[j]=this->Schedule_table_sleep_time[j]+Sleep_period+Listen_period; }

					//if( (this->Schedule_table_sleep_time[j]<=this->Timer_event_radio_off->getEvent_time()+Basic_unit_duration2
					if( (this->Schedule_table_sleep_time[j]<=this->Timer_event_radio_off->getEvent_time()+5 
						|| this->Schedule_table_sleep_time[j]-Listen_period<New_coming_event->getEvent_time() )
						&& New_coming_event->getEvent_time()>Initial_listen_period
						&& this->Schedule_table_sleep_time[j]>0)
						{this->Schedule_table_sleep_time[j]=this->Schedule_table_sleep_time[j]+Sleep_period+Listen_period; }
				}
				
				if(this->Schedule_table_node_id[0]<0 && New_coming_event->getEvent_time()<=Initial_listen_period)
				{
					this->Schedule_table_node_id[0]=this->Node_id;
					this->Schedule_table_sleep_time[0]=Initial_listen_period+Sleep_period+Listen_period; 
				}
				
				int k=Minimum_Schedule(this->Schedule_table_sleep_time, 5);  
				Event* future_event25=new Event; future_event25->setEvent_time(this->Schedule_table_sleep_time[k]-Listen_period);
				future_event25->setEvent_type(0); future_event25->setEvent_node_id(this->Node_id); 
				future_event25->setEvent_pointer(NULL); future_event25->setEvent_pointer_size(0);
				future_event25->setTCPP(-20);
				handle->add(future_event25);
				Process_print(0, this->Node_id, this->Schedule_table_sleep_time[k]-Listen_period, 
					0,1,New_coming_event->getEvent_time());
				
				//measure
				this->Total_time_radio_on=this->Total_time_radio_on+New_coming_event->getEvent_time()-this->Time_radio_on; //measurement
				this->SYNC_time_radio_on=this->SYNC_time_radio_on+Time_for_SYNC;

				this->radio_on=false; this->Radio_off_time=New_coming_event->getEvent_time();
				this->Transmitter_id=-1; this->Receiver_id=-1;
				for(int i=0; i<10; i++) 
					this->RTS_CTS_buffer[i]=-1;
				if(this->Timer_event!=NULL)                               
					{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL;}
				if(this->Timer_event_radio_off!=NULL)
					{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
				if(this->Interleaving_num!=0)
					this->Interleaving_num=0;  //control unsuccessful transmission in the future
			}
		}
		else//section-2     //this->Timer_event_radio_off!=New_coming_event
		{       //RTSs get collision at this node    
			for(int j=0; j<5; j++)
			{
				if(New_coming_event->getEvent_time()<=Initial_listen_period
					&& this->Schedule_table_sleep_time[j]>0)
					{this->Schedule_table_sleep_time[j]=this->Schedule_table_sleep_time[j]+Sleep_period+Listen_period; }

				if(this->Schedule_table_sleep_time[j]<=this->Timer_event_radio_off->getEvent_time()+5
					&& New_coming_event->getEvent_time()>Initial_listen_period
					&& this->Schedule_table_sleep_time[j]>0)
					{this->Schedule_table_sleep_time[j]=this->Schedule_table_sleep_time[j]+Sleep_period+Listen_period; }
			}
			
			int k=Minimum_Schedule(this->Schedule_table_sleep_time, 5);
			Event* future_event25=new Event; future_event25->setEvent_time(this->Schedule_table_sleep_time[k]-Listen_period);
			future_event25->setEvent_type(0); future_event25->setEvent_node_id(this->Node_id); 
			future_event25->setEvent_pointer(NULL); future_event25->setEvent_pointer_size(0);
			future_event25->setTCPP(-20);
			handle->add(future_event25);
			Process_print(0, this->Node_id, this->Schedule_table_sleep_time[k]-Listen_period, 
				0,2,New_coming_event->getEvent_time());
			
			//measure
			this->Total_time_radio_on=this->Total_time_radio_on+New_coming_event->getEvent_time()-this->Time_radio_on; //measurement
			this->SYNC_time_radio_on=this->SYNC_time_radio_on+Time_for_SYNC;

			this->radio_on=false; this->Radio_off_time=New_coming_event->getEvent_time();
			this->Transmitter_id=-1; this->Receiver_id=-1;
			for(int i=0; i<10; i++) 
				this->RTS_CTS_buffer[i]=-1;
			this->Interleaving_num=0;  //control unsuccessful transmission in the future
			if(this->Timer_event!=NULL)                               
				{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL;}
			if(this->Timer_event_radio_off!=NULL)
				{this->Timer_event_radio_off->setFlag_event_point(1); this->Timer_event_radio_off=NULL; }
		}
	}
	else //this->Timer_event_radio_off==NULL        This part is for Timer
	{//section-3
	 //1. already sent out EOT or received EOT, Transmission is finished  2. Sent out data/CTS, but doesn't successfully receive EOT/Data
	 //3. passive nodes receives RTS or CTS
		
		//measure
		this->Total_time_radio_on=this->Total_time_radio_on+New_coming_event->getEvent_time()-this->Time_radio_on;
		this->SYNC_time_radio_on=this->SYNC_time_radio_on+Time_for_SYNC;

		this->radio_on=false; this->Radio_off_time=New_coming_event->getEvent_time();
		this->Transmitter_id=-1; this->Receiver_id=-1;
		for(int i=0; i<10; i++) 
			this->RTS_CTS_buffer[i]=-1;
		if(this->Timer_event!=NULL)                               
			{this->Timer_event->setFlag_event_point(1); this->Timer_event=NULL;}
		this->Interleaving_num=0;  //control unsuccessful transmission in the future
		for(int i=0; i<5; i++) //increase one cycle for qualified schedule 
		{	//1. for initial schedule, all items get one cycple   2. only the schedule, which is smaller than the current one, can get one cycle
			if(this->Schedule_table_sleep_time[i]>0)
			{
				if(this->Schedule_table_sleep_time[i]<=New_coming_event->getEvent_time())
					{this->Schedule_table_sleep_time[i]=this->Schedule_table_sleep_time[i]+Sleep_period+Listen_period; }
				
				if(this->Schedule_table_sleep_time[i]>New_coming_event->getEvent_time())
				{
					if(this->Schedule_table_sleep_time[i]-Listen_period<New_coming_event->getEvent_time())
						{this->Schedule_table_sleep_time[i]=this->Schedule_table_sleep_time[i]+Sleep_period+Listen_period; }
				}
			}
		}
		
		int k=Minimum_Schedule(this->Schedule_table_sleep_time, 5);  
		Event* future_event25=new Event; future_event25->setEvent_time(this->Schedule_table_sleep_time[k]-Listen_period);
		future_event25->setEvent_type(0); future_event25->setEvent_node_id(this->Node_id); 
		future_event25->setEvent_pointer(NULL); future_event25->setEvent_pointer_size(0);
		future_event25->setTCPP(-20);
		handle->add(future_event25);
		Process_print(0, this->Node_id, this->Schedule_table_sleep_time[k]-Listen_period, 0,3,New_coming_event->getEvent_time());
	}
}