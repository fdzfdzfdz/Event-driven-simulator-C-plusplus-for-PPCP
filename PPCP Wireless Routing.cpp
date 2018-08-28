// TCPPNetwork.cpp : Defines the entry point for the console application.
//
#include <ctime>
#include <cmath>
#include <list>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "Event.h"
#include "Node.h"
#include <queue>
#include <deque>

extern void Print_result(int node_id);
extern void Initialize_network_routingtable();

ofstream logs;
ofstream logs2;
ofstream logs3;
ofstream logs4;
list<Node> Node_network; 

long long Simulation_time=131068000/2; 
double Channel_rate=2000;
int SYNC_period=432000000/2;
int Silence_unit=1;

int Value_TCPP_Max=0;
int No_of_data_field=1;
int Value_max_array[11]={31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767};

// {9.1875, 10.7813, 12.4219, 14.2109, 15.8945, 17.7402, 19.3154, 21.2056, 23.0349, 24.7509, 26.4136} average length
int Value_max_length[11]={14, 16, 19, 22, 25, 27, 29, 33, 36, 39, 41};
          // type(20): 12,    id(64): 16                     maximum length
int length_TCPPs_max=0;    //5+12+16+No_of_data_field*
int Maximum_EOT=5+7+16;   //EOT_type:4

int Processing_delay=1;     //note: 100(ns)=1(tick)
int Propagation_delay=1;
double Lambda=0.1;              //unit: TCPP/sec
int Numofnodes=64;
int Node_array[64];

int Id_Denominator=6;
int Denominator_X=6; 

int Time_slot_SYNC=5+3+11+5;      
int SYNC_maximum=54;
int Contention_windwo_SYNC=3;

int Time_slot_RTS=5+4+15+15+Propagation_delay*2+Propagation_delay+1;    //Id:11~16             
int Contention_window_RTS=5;

int Time_for_SYNC=Time_slot_SYNC*Contention_windwo_SYNC+Time_slot_SYNC   //+Processing_delay+Propagation_delay
	+Processing_delay;       //
int Back_off_main=Time_slot_RTS*Contention_window_RTS;  //
int Listen_period=Time_for_SYNC+Back_off_main+Propagation_delay+Processing_delay+Time_slot_RTS    
	+Processing_delay;      // 358

int Sleep_period=0;
int Cycle_period_samples[3]={716000, 71600, 7160};
                            //0.0005, 0.005, 0.05,
                            
int Initial_listen_period=30*35;                  //1080
int Initial_SYNC_backoff_period=Initial_listen_period;
int Initial_rebro_SYNC_backoff_period=Initial_listen_period/17;  //2 Time_slot_SYNC

//long long scale=0;

using namespace std;

Node *getNode_from_id(int i)
{
	list<Node>::iterator Node_iterator;
	for(Node_iterator= Node_network.begin(); Node_iterator !=Node_network.end(); Node_iterator++)
	{
		if( Node_iterator->getNode_id()== i) 
			break;
	}
	return &(*Node_iterator);
}

void Initialize_event(Event *Event_initial, int source_and_node_id, long long t, int TCPP)
{
	if(TCPP>=0)
		Event_initial->setEvent_type(19);
	else
		Event_initial->setEvent_type(0);
	Event_initial->setEvent_node_id(source_and_node_id);
	Event_initial->setEvent_time(t);
	Event_initial->setEvent_pointer(NULL);
	Event_initial->setEvent_pointer_size(0);
	Event_initial->setTCPP(TCPP);
	Node* Source_node=getNode_from_id(source_and_node_id);
}

int main(int argc, char* argv[])
{
	//for(double NN=0.1; NN<=1; NN=NN+0.1)   //for Lambda
	//{
	
		//for(int QQ=0; QQ<3; QQ++)       //for cycle time
		//{
			Sleep_period=Cycle_period_samples[0]-Listen_period;
			Lambda=0.1;
		
			for(int MM=0; MM<11; MM++)   //for maximum value--Delta
			{
				Value_TCPP_Max=Value_max_array[MM];
				length_TCPPs_max=5+12+16+No_of_data_field*Value_max_length[MM]; //12: type(20)

				int Time=static_cast<int>(time(0)); 
				//int Time=1475382421;
				cout<<"Time="<<Time<<endl;
				srand(Time);
				logs.open("State Machine.txt", ios::trunc);      //ios::trunc cancel and rewrite     logFileName.c_str() is equivalent to "logs.txt"
				logs2.open("Every Event state(after).txt", ios::trunc);
				logs3.open("Every Event state(before).txt", ios::trunc);
				logs4.open("Running process.txt", ios::app);
		
				Initialize_network_routingtable();
				Simulator *simulator= new Simulator;

				for(int i=0; i<Numofnodes; i++)         //for radio on
				{
					Event *temp_event=new Event;
					Initialize_event(temp_event,i,0,-1);
					simulator->add(temp_event);

					Node_array[i]=1;
				}

				for(int i=0; i<Numofnodes; i++)         //for data
				{
					Event *temp_event=new Event;
					//int TCPP=rand()%(Value_TCPP_Max+1);
					Initialize_event(temp_event,i,0,1);
					simulator->add(temp_event);
				}

				/*Event *event1=new Event;
				int TCPP=rand()%(Value_TCPP_Max+1);
				Initialize_event(event1,0,0,1);
				simulator->add(event1);*/
		
				logs<<"Event_type	Node_id	Current_time	SYNC_buffer_0	SYNC_buffer_1	SYNC_buffer_2	Transmitter_id	Receiver_id	RTS_buffer_0"
					<<"	RTS_buffer_1	Interleaving_num	response_SYNC_control"<<endl;
				logs<<"	Forward_type	Forward_Node_id	Forward_time	Event_pointer_size	Section"<<endl;
				simulator->action();
		
				cout<<"Duty Cycle	"<<static_cast<double>(Listen_period)/(Listen_period+Sleep_period)<<"	"
					<<"Lambda	"<<Lambda<<"	"<<"Value_Max	"<<Value_TCPP_Max<<endl;
				for(int i=0; i<Numofnodes; i++)
					Print_result(i);
		        
				cout<<"	"<<"Listen Period:	"<<Listen_period<<"	"<<"Sleep_period:	"<<Sleep_period<<endl<<endl;
				
				logs.close();
				logs2.close();
				logs3.close();
				logs4.close();
				//scale=0;
				Node_network.clear();

				while(simulator->get_event_queue().size()!=0)
				{
					Event *current_event=simulator->get_event_queue().top();
					simulator->get_event_queue().pop();  
					delete current_event;
				}
				delete simulator;
			}
		//}
	//}
	return 0;
}

