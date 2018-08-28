#include <deque>
#include <iostream>
#include <fstream>
#include "Node.h"

extern ofstream logs;

//Event_type, Node_id, Current_time, SYNC_buffer[0], SYNC_buffer[1], SYNC_buffer[2], Transmitter_id, Receiver_id, RTS_buffer[0], RTS_buffer[1]
void Head_print(int Event_type, int Node_id, long long Current_time, int SYNC_buffer_0, int SYNC_buffer_1, int SYNC_buffer_2, 
	int Transmitter_id, int Receiver_id, int RTS_buffer_0, int RTS_buffer_1, int Interleaving_num, bool response_SYNC_control)
{
	//if(Current_time>=309200592 && Current_time<=359200592 || Current_time>=3173801028 )
	//{
		/*logs<<"--------------------------------------------------------------------------"<<endl;
		logs<<Event_type<<"	"<<Node_id<<"	"<<Current_time<<"	"<<SYNC_buffer_0<<"	"<<SYNC_buffer_1<<"	"<<SYNC_buffer_2<<"	"
			<<Transmitter_id<<"	"<<Receiver_id<<"	"<<RTS_buffer_0<<"	"<<RTS_buffer_1<<"	"<<Interleaving_num<<"	"
			<<response_SYNC_control<<endl;*/
	//}
}

void Process_print(int Forward_type, int Forward_Node_id, long long Forward_time, int Event_pointer_size, int Section, long long Current_time)
{
	//if(Current_time>=309200592 && Current_time<=359200592 || Current_time>=3173801028 )
	//{
		//logs<<"	"<<Forward_type<<"	"<<Forward_Node_id<<"	"<<Forward_time<<"	"<<Event_pointer_size<<"	"<<Section<<endl;
	//}
}

//void End_print(int SYNC_buffer_0, int SYNC_buffer_1, int SYNC_buffer_2, int Transmitter_id, int Receiver_id)
//{
	//logs<<SYNC_buffer_0<<"	"<<SYNC_buffer_1<<"	"<<SYNC_buffer_2<<"	"<<Transmitter_id<<"	"<<Receiver_id<<endl;
//}

void Print_schedule_table(int *Schedule_table_node_id, int M, long long *Schedule_table_sleep_time, int N)
{
	for(int i=0; i<M; i++)
	{
		cout<<Schedule_table_node_id[i]<<"	";
	}
	cout<<endl;
	for(int i=0; i<N; i++)
	{
		cout<<Schedule_table_sleep_time[i]<<"	";
	}
	cout<<endl;
}