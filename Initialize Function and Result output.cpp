#include <iostream>
#include <ctime>
#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include "Node.h"
#include <queue>
#include <deque>


extern ofstream logs4;

extern int Numofnodes;
extern list<Node> Node_network;
extern Node *getNode_from_id(int i);
extern int Basic_unit_duration;
extern int Listen_period;

using namespace std;

double Distance(int X1, int Y1, int X2, int Y2)
{
	return sqrt(static_cast<double>((X2-X1)*(X2-X1)+(Y2-Y1)*(Y2-Y1)));
}

void Initialize_network_routingtable()
{
	for(int i=0; i<Numofnodes; i++)
		{Node node; node.setNode_id(i); Node_network.push_back(node); }

	int MM=0;					         //grid network
	for(int n=0; n<sqrt(static_cast<double>(Numofnodes)); n++)   
	{
		for(int m=0; m<sqrt(static_cast<double>(Numofnodes)); m++)
		{
			Node* temp_node=getNode_from_id(MM);
			temp_node->setXaxis_Yaxis(m,n);
			MM++;
			//cout<<temp_node->getXaxis()<<" "<<temp_node->getYaxis()<<endl;   //just for debug
		}
	}
	for(MM=0; MM<Numofnodes; MM++)
	{
		int* NT=new int [10]();
		for(int j=0; j<10; j++)	{NT[j]=-1; }
		Node* temp_node=getNode_from_id(MM);
		for(int i=0; i<Numofnodes; i++)
		{
			if(i!=MM)
			{
				Node* temp_node2=getNode_from_id(i);
				if(Distance(temp_node->getXaxis(), temp_node->getYaxis(), temp_node2->getXaxis(), temp_node2->getYaxis())<=1)
				{
					for(int j=0; j<10; j++) 
					{
						if(NT[j]<0) 
						{NT[j]=i; break;}
					} 
				}
			}
		}
		//for(int j=0; j<10; j++)    //just for debug
		//	cout<<NT[j]<<" ";
		//cout<<endl;
		temp_node->setNeighbor_table(NT, 10);
		delete[] NT;
		//temp_node->printNeighbor_table(10);
	}

	//for(int i=0; i<Numofnodes; i++)      //linear network
	//{
	//	int* NT=new int [10]();
	//	for(int j=2; j<10; j++)	{NT[j]=-1; }
	//	if(i==0)
	//		{NT[0]=1; NT[1]=-1; Node* temp_node=getNode_from_id(i); temp_node->setNeighbor_table(NT, 10); }
	//	if(i==Numofnodes-1)
	//		{NT[0]=Numofnodes-2; NT[1]=-1; Node* temp_node=getNode_from_id(i); temp_node->setNeighbor_table(NT, 10); }
	//	if(i!=0 && i!=Numofnodes-1)
	//		{NT[0]=i-1; NT[1]=i+1; Node* temp_node=getNode_from_id(i); temp_node->setNeighbor_table(NT, 10); }
	//	//temp_node->printNeighbor_table(10);
	//	delete[] NT;
	//}

	//int* NT=new int [10]();              //node-1
	//for(int j=0; j<10; j++)	{NT[j]=-1; }
	//NT[0]=2; NT[1]=3;       
	//Node* node1=getNode_from_id(1);
	//node1->setNeighbor_table(NT,10);
	////node1->printNeighbor_table(10);

	//for(int j=0; j<10; j++)	{NT[j]=-1; }      //node-2
	//NT[0]=1; NT[1]=4;       
	//Node* node2=getNode_from_id(2);
	//node2->setNeighbor_table(NT,10);
	////node2->printNeighbor_table(10);
}

void Print_result(int i)
{
	Node *current_node=getNode_from_id(i);
	logs4<<i<<"	"<<current_node->getTotal_num_tran_TCPP()<<"	"
		<<current_node->getSucce_num_tran_TCPP()<<"	"<<current_node->getTotal_num_rece_TCPP()<<"	"
		<<current_node->getTotal_time_radio_on()<<"	"<<current_node->getTotal_time_interval()<<"	"
		<<current_node->getData_tran_interval()<<"	"<<current_node->getWaiting_delay()<<"	"
		<<current_node->getTotal_RTS()<<"	"<<current_node->getTotal_CTS()<<"	"
		<<current_node->getTotal_EOT()<<"	"<<current_node->getPulse_count()<<"	"
		<<current_node->getPulse_count_non_SYNC()<<"	"<<current_node->getSYNC_time_radio_on()<<"	"
		<<current_node->getTrans_finish_time()<<"	"<<current_node->getTotal_times_tran()<<"	"
		<<current_node->getSucce_timers_tran()<<"	"<<current_node->getTotal_SYNC()<<"	"
		<<current_node->getTotal_time_receiving()<<"	"<<current_node->getAccess_delay()<<"	"
		<<current_node->getSYNC_time_receiving()<<endl;
	
	cout<<i<<"	"<<current_node->getTotal_num_tran_TCPP()<<"	"
		<<current_node->getSucce_num_tran_TCPP()<<"	"<<current_node->getTotal_num_rece_TCPP()<<"	"
		<<current_node->getTotal_time_radio_on()<<"	"<<current_node->getTotal_time_interval()<<"	"
		<<current_node->getData_tran_interval()<<"	"<<current_node->getWaiting_delay()<<"	"
		<<current_node->getTotal_RTS()<<"	"<<current_node->getTotal_CTS()<<"	"
		<<current_node->getTotal_EOT()<<"	"<<current_node->getPulse_count()<<"	"
		<<current_node->getPulse_count_non_SYNC()<<"	"<<current_node->getSYNC_time_radio_on()<<"	"
		<<current_node->getTrans_finish_time()<<"	"<<current_node->getTotal_times_tran()<<"	"
		<<current_node->getSucce_timers_tran()<<"	"<<current_node->getTotal_SYNC()<<"	"
		<<current_node->getTotal_time_receiving()<<"	"<<current_node->getAccess_delay()<<"	"
		<<current_node->getSYNC_time_receiving()<<endl;
		
}

int Calculate_Pulse_count(int *Signal_array, int i)
{
	int n=0;
	for(int k=0; k<i; k++)
	{if(Signal_array[k]==1) n++; }
	return n;
}

//int calculate_Pulse_swtich_count(int *Signal_array, int i)
//{
//	int n=0;
//	for(int k=0; k<i-1; k++)
//	{if(Signal_array[k]!=Signal_array[k+1]) n++;}
//	return n;
//}

bool Error_signal_identify(int *Signal_array, int i)
{
	for(int m=0; m<6; m++)
	{
		if(Signal_array[m]!=1 || Signal_array[i-1-m]!=1)
			return false;
	}
	return true;
}

void Choose_receiver(int* Table, int Size)
{
	int k=0;
	for(int i=0; i<Size; i++)
	{
		if(Table[i]>=0)
		{ k++; } 
	}
	int i=rand()%k;
	int temp=Table[i]; Table[i]=Table[0]; Table[0]=temp;
}

int Minimum_Schedule(long long *Schedule_table_sleep_time, int N)
{
	long long Minimum=Schedule_table_sleep_time[0]; int m=0;
	for(int i=1; i<N; i++)
	{
		if(Schedule_table_sleep_time[i]<Minimum && Schedule_table_sleep_time[i]>0)
		{Minimum=Schedule_table_sleep_time[i]; m=i;}
	}
	return m;
}

int Relative_time_radio_off(int M, long long *Schedule_table_sleep_time, int N)    //standard=Schedule_table_sleep_time[M] is minimum number currently
{
	int m=M; long long standard=Schedule_table_sleep_time[M];
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<N; j++)
		{
			if(Schedule_table_sleep_time[j]-Listen_period<=standard && standard<Schedule_table_sleep_time[j])
			{standard=Schedule_table_sleep_time[j]; m=j; i=-1; break; }
		}
	}
	return m;
}

int Calculate_Pulse_count_part(int *Signal_array, int Signal_size, int Start_point, int End_point)
{
	int n=0;
	for(int k=Start_point; k<End_point; k++)
		{if(Signal_array[k]==1) n++; }
	return n;
}

void Receiving_time_calculation(long long* Parameter, int* Signal, int Signal_size, bool radio_on, long long Radio_off_time, 
	long long Last_moment_receiving, long long Current_time)
{ //Parameter[0]=Total_time_receiving;  Parameter[1]=New_coming_event->getti
	if(radio_on==false)
	{
		if(Last_moment_receiving<Radio_off_time && 
			Current_time-Signal_size<Last_moment_receiving && 
			Current_time-Signal_size<Radio_off_time)
		{
			int Start_point=static_cast<int>(Last_moment_receiving-(Current_time-Signal_size));
			int End_point=static_cast<int>(Radio_off_time-(Current_time-Signal_size));
			Parameter[0]=Parameter[0]+Calculate_Pulse_count_part(Signal, Signal_size, Start_point, End_point);
			Parameter[1]=Current_time;
		}
	}
	else
	{
		Parameter[0]=Parameter[0]+Calculate_Pulse_count(Signal, Signal_size);
		Parameter[1]=Current_time;
	}
}



//int Relative_time_radio_on(int *Schedule_table_sleep_time, int N)
//{
//	int Relative_time=Schedule_table_sleep_time[0];
//	int m=0;
//	for(int i=1; i<N; i++)
//	{
//		if(Schedule_table_sleep_time[i]<Relative_time)
//		{Relative_time=Schedule_table_sleep_time[i]; m=i;}
//	}
//	return m;
//}

