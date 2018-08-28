#include <deque>
#include <iostream>
#include <fstream>
#include "Node.h"
#include "Event.h"

using namespace std;

extern ofstream logs;
extern int No_of_data_field;
extern int length_TCPPs_max;
extern int Silence_unit;
extern int Id_Denominator; //6
extern int Denominator_X; //6

void Interleaving_cluster(int* Signal, int Signal_size, int* Values_array_id_transmitter, int* Number_sum_id_transmitter, int* Values_array, int Values_array_size)
{
	Signal[0]=1; Signal[1]=1; Signal[2]=1; Signal[3]=1;  Signal[8]=1;  Signal[9]=1;  //1111000011
	int Next_start_point=10;                               
	for(int i=0; i<Number_sum_id_transmitter[0]; i++)
	{
		Signal[Next_start_point+Values_array_id_transmitter[i]+1]=1;
		Next_start_point=Next_start_point+Values_array_id_transmitter[i]+2;
	}
	Signal[Next_start_point]=1;
	Next_start_point=Next_start_point+1;
	int i=0;
	while(i<Values_array_size+No_of_data_field)
	{
		Signal[Next_start_point+Values_array[i]+1]=1;
		Next_start_point=Next_start_point+Values_array[i]+2;
		i++;
	}
	Signal[Next_start_point]=1; 
	if( (Next_start_point+1)!= Signal_size)            //just for debug
		cout<<"Interleaving_cluster"<<endl; 
	
	//for(int j=0; j<Signal_size; j++)  //just for debug
	//	cout<<Signal[j]<<" ";
	//cout<<endl;
}

int Decoding_cluster(deque<TCPP_pair> &Queuing_system_temporary, int* TCPP_Data, int TCPP_Size, int Transmitter_id, 
	int Receiver_id, long long Current_time) 
{                                                    //1111000011
	int Num_of_Data=0; int m=0; int a=0;
	int Next_start_point=0;
	for(int i=10; i<TCPP_Size; i++)
	{
		if(TCPP_Data[i]==1 && TCPP_Data[i+1]==1 && TCPP_Data[i+2]!=1)
			{Next_start_point=i+2; m=i+2; break; }
	}
	for(int i=Next_start_point; i<TCPP_Size-1; i++)
	{
		if(TCPP_Data[i]==1)
		{
			a=a*Denominator_X+i-m-1; m=i+1;
			if(1==TCPP_Data[i+1])
			{
				TCPP_pair element;
				element.TCPP=a; element.Time_in=Current_time;
				Queuing_system_temporary.push_back(element);
				Num_of_Data++;
				a=0; m=i+2; i=i+1;
			}
		}
	}
	return Num_of_Data;
}

void Singal_construction(int* Signal, int Signal_size, int type_decimal, int* Values_array_id_transmitter, int* Number_sum_id_transmitter, 
	int* Values_array_id_receiver, int* Number_sum_id_receiver, int* Values_array, int* Number_sum_array, int Num_of_values_in_packet)
{
	if(type_decimal==0)  //SYNC=0
	{
		int Next_start_point=7;  
		Signal[0]=1; Signal[1]=1; Signal[2]=1; Signal[3]=1;  Signal[5]=1; Signal[6]=1; 
		                               //start from 0
		for(int i=0; i<Number_sum_id_transmitter[0]; i++)
		{
			Signal[Next_start_point+Values_array_id_transmitter[i]+1]=1;
			Next_start_point=Next_start_point+Values_array_id_transmitter[i]+2;
		}
		Signal[Next_start_point]=1;
		Next_start_point=Next_start_point+1;
		for(int i=0; i<Number_sum_array[0]; i++)
		{
			Signal[Next_start_point+Values_array[i]+1]=1;
			Next_start_point=Next_start_point+Values_array[i]+2;
		}
		Signal[Next_start_point]=1; Signal[Next_start_point+1]=1;

		if( (Next_start_point+2)!= Signal_size)            //just for debug
			cout<<"Singal_construction_SYNC"<<endl;

		//for(int j=0; j<Signal_size; j++)  //just for debug
		//	cout<<Signal[j]<<" ";
		//cout<<endl;
	}
	if(type_decimal==1)  //RTS=1
	{
		int Next_start_point=8;
		Signal[0]=1; Signal[1]=1; Signal[2]=1; Signal[3]=1;  Signal[6]=1; Signal[7]=1;
		
		for(int i=0; i<Number_sum_id_transmitter[0]; i++)
		{
			Signal[Next_start_point+Values_array_id_transmitter[i]+1]=1;
			Next_start_point=Next_start_point+Values_array_id_transmitter[i]+2;
		}
		Signal[Next_start_point]=1;
		Next_start_point=Next_start_point+1;
		for(int i=0; i<Number_sum_id_receiver[0]; i++)
		{
			Signal[Next_start_point+Values_array_id_receiver[i]+1]=1;
			Next_start_point=Next_start_point+Values_array_id_receiver[i]+2;
		}
		Signal[Next_start_point]=1; Signal[Next_start_point+1]=1;

		if( (Next_start_point+2)!= Signal_size)            //just for debug
			cout<<"Singal_construction_RTS"<<endl;
		//for(int j=0; j<Signal_size; j++)  //just for debug
		//	cout<<Signal[j]<<" ";
		//cout<<endl;
	}
	if(type_decimal==2)  //CTS=2
	{
		int Next_start_point=9;
		Signal[0]=1; Signal[1]=1; Signal[2]=1; Signal[3]=1;  Signal[7]=1; Signal[8]=1;

		for(int i=0; i<Number_sum_id_transmitter[0]; i++)
		{
			Signal[Next_start_point+Values_array_id_transmitter[i]+1]=1;
			Next_start_point=Next_start_point+Values_array_id_transmitter[i]+2;
		}
		Signal[Next_start_point]=1; Signal[Next_start_point+1]=1;

		if( (Next_start_point+2)!= Signal_size)            //just for debug
			cout<<"Singal_construction_CTS"<<endl;
		//for(int j=0; j<Signal_size; j++)  //just for debug
		//	cout<<Signal[j]<<" ";
		//cout<<endl;
	}
	if(type_decimal==4)  //EOT=4
	{
		int Next_start_point=11;
		Signal[0]=1; Signal[1]=1; Signal[2]=1; Signal[3]=1;  Signal[9]=1; Signal[10]=1;
		for(int i=0; i<=Number_sum_id_transmitter[0]-1; i++)
		{
			Signal[Next_start_point+Values_array_id_transmitter[i]+1]=1;
			Next_start_point=Next_start_point+Values_array_id_transmitter[i]+2;
		}
		Signal[Next_start_point]=1; Signal[Next_start_point+1]=1;
		
		if( (Next_start_point+2)!= Signal_size)            //just for debug
			cout<<"Singal_construction_EOT"<<endl;
		//for(int j=0; j<Signal_size; j++)  //just for debug
		//	cout<<Signal[j]<<" ";
		//cout<<endl;
	}
}

void Signal_detection(int* Signal, int Signal_size, int type_decimal, int* Id_addition, int Id_addition_size)
{
	//for(int j=0; j<Signal_size; j++)  //just for debug
	//	cout<<Signal[j]<<" ";
	//cout<<endl;

	if(type_decimal==0)  //SYNC
	{
		int m=7;   //m: 1111011
		for(int i=7; i<Signal_size; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {m=i+1; break; }
				else
				{
					Id_addition[0]=Id_addition[0]*Id_Denominator+i-m-1; 
					m=i+1;
				}
			}
		}
		for(int i=m+1; i<Signal_size; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {break; }
				else
				{
					Id_addition[1]=Id_addition[1]*Denominator_X+i-m-1; 
					m=i+1;
				}
			}
		}
		if( (m!=Signal_size-2) || (Signal[m+1]!=1) )          //just for debug
			cout<<"Signal_detection_SYNC_error"<<endl;
	}
	if(type_decimal==1)  //RTS=1
	{
		int m=8;   //m: 11110011
		for(int i=8; i<Signal_size; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {m=i+1; break; }
				else
				{
					Id_addition[0]=Id_addition[0]*Id_Denominator+i-m-1; 
					m=i+1;
				}
			}
		}
		for(int i=m+1; i<Signal_size; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {break; }
				else
				{
					Id_addition[1]=Id_addition[1]*Denominator_X+i-m-1; 
					m=i+1;
				}
			}
		}
		if( (m!=Signal_size-2) || (Signal[m+1]!=1) )        //just for debug
			cout<<"Signal_detection_RTS_error"<<endl;
	}
	if(type_decimal==2)  //CTS=2
	{
		int m=9;   //m: 111100011
		for(int i=9; i<Signal_size; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {break; }
				else
				{
					Id_addition[0]=Id_addition[0]*Id_Denominator+i-m-1; 
					m=i+1;
				}
			}
		}
		if( (m!=Signal_size-2) || (Signal[m+1]!=1) )        //just for debug
			cout<<"Signal_detection_CTS_error"<<endl;

	}
	if(type_decimal==3)  //cluster=3: only detect TX_id 
	{
		int m=10;  //m: 1111000011
		for(int i=10; i<Signal_size-1; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {m=i+1; break; }
				else
				{
					Id_addition[0]=Id_addition[0]*Id_Denominator+i-m-1; 
					m=i+1;
				}
			}
		}
	}
	if(type_decimal==4)  //EOT=4
	{
		int m=11;  //m: 11110000011
		for(int i=11; i<Signal_size-1; i++)
		{
			if(Signal[i]==1)
			{
				if(i-m-1<0) {break; }
				else
				{
					Id_addition[0]=Id_addition[0]*Id_Denominator+i-m-1; 
					m=i+1;
				}
			}
		}
	}
}

void Digits_check_big_data(int* Number_sum_array, int* Values_array, deque<TCPP_pair> &Queuing_system)
{
	deque<TCPP_pair>::iterator TCPP_point;
	int NN=0; int k=0;
	int Array_sequence_number=0;
	int m=0;
	for(TCPP_point=Queuing_system.begin(), NN=0; NN<No_of_data_field  && TCPP_point!=Queuing_system.end(); NN++, TCPP_point++)
	{
		int a=TCPP_point->TCPP;
		if(a==0)
			{Number_sum_array[0]++; /*Number_sum_array[1]=0;*/ Values_array[k]=0; k++; }
		else
		{
			while(a)
			{
				Values_array[k]=a%Denominator_X;   //note: value=123, in Values_array, the order is 3 2 1
				Number_sum_array[1]=Number_sum_array[1]+Values_array[k];
				a=a/Denominator_X;
				k++;
			}
			m=0;
			for(int j=Array_sequence_number; j<=(k-Array_sequence_number)/2-1+Array_sequence_number; j++, m++)
				{a=Values_array[j]; Values_array[j]=Values_array[k-1-m]; Values_array[k-1-m]=a; }//note: value=123, in Values_array, the order is 1 2 3
		}
		Values_array[k]=-1; k++; Array_sequence_number=k; 
		if(k>30)
			cout<<"Digits_check_big_data"<<endl;
	}
	Number_sum_array[0]=k-No_of_data_field;

	//cout<<"Number_sum_array: "<<Number_sum_array[0]<<" "<<Number_sum_array[1]<<endl;  //just for debug
	//cout<<"Values_array: ";
	//for(int j=0; j<k; j++)
	//	{cout<<Values_array[j]<<" ";}
	//cout<<endl;
}

void Digits_check_id(int* Number_sum, int* Values_array, int Decimal_value)
{//Number_sum[0]: record how many remainders;     //Number_sum[1]: sum of remainders;   //Values_array: each value of remainder
	int a=Decimal_value;
	int k=0;
	if(a==0)
		{Number_sum[0]=1; Number_sum[1]=0; Values_array[0]=0; k=1; }
	else
	{
		while(a)
		{
			Values_array[k]=a%Id_Denominator;   //note: remainder value=123, in Values_array, the order is 3 2 1
			Number_sum[1]=Number_sum[1]+Values_array[k];
			a=a/Id_Denominator;
			k++;
		}
		for(int j=0; j<=k/2-1; j++)
			{a=Values_array[j]; Values_array[j]=Values_array[k-1-j]; Values_array[k-1-j]=a; } //note: value=123, in Values_array, the order is 1 2 3
		Number_sum[0]=k;
	}
	if(k>5)
		cout<<"Digits_check_id"<<endl;
	
	//cout<<"Number_sum: "<<Number_sum[0]<<" "<<Number_sum[1]<<endl;  //just for debug
	//cout<<"Values_array: ";
	//for(int j=0; j<k; j++)
	//	{cout<<Values_array[j]<<" ";}
	//cout<<endl;
}

//int Interleaving_cluster(deque<TCPP_pair> &Queuing_system, int *arrayTotal, int Node_id, Event *New_coming_event)
//{
//	int Interleaving_num=0;
//	int Beta=length_TCPPs_max/7+20;      //maybe change later according to Vmax and length_TCPPs_max ???
//	arrayTotal[0]=1; arrayTotal[2]=1;    //arrayTotal[4+this->Node_id]=1;
//	int* BetaEnd=new int [Beta]();  int* BetaSize=new int [Beta]();
//	int assignStart=0, assignK=1;        //assignStart=4+this->Node_id
//	deque<TCPP_pair>::iterator TCPP_point;
//	for(TCPP_point=Queuing_system.begin(); TCPP_point!=Queuing_system.end(); TCPP_point++)
//	{
//		int j, Last_BetaEnd;
//		for(j=assignStart; j<4+Node_id+length_TCPPs_max; j++)
//		{   bool collision=false;
//			if(j>BetaEnd[0]+BetaSize[0]+1)                //judge if the closed pulse gets ended. 
//			{   for(int K=0; K<Beta-1; K++)
//				{BetaEnd[K]=BetaEnd[K+1]; BetaSize[K]=BetaSize[K+1];}
//				assignK=1;
//				for(int K=0; K<Beta; K++)
//				{if(assignK==BetaSize[K]) {K=-1; assignK++;}  }
//			}
//			for(int n=j; n<=j+assignK+1; n++)         //pick up the position in order to put the current TCPP in the array
//			{   int m;
//				for(m=0; m<Beta; m++)                     //m is just the last non-zero sequence number
//				{if(BetaEnd[m]==0) break;}
//				Last_BetaEnd=m-1;                       //record the position of 
//				for(int X=0; X<m; X++)       //X<m, the first tcpp cannot be implemented here
//				{   if((n>=BetaEnd[X] && n<=BetaEnd[X]+BetaSize[X]+1) || 
//						(n+assignK+2+(TCPP_point->TCPP+1)*Silence_unit>=BetaEnd[X] && 
//						n+assignK+2+(TCPP_point->TCPP+1)*Silence_unit<=BetaEnd[X]+BetaSize[X]+1))
//					{collision=true; break;}
//				}
//				if(collision==true) break;
//			}
//			if(collision==false)  break;
//		}
//		if(TCPP_point==Queuing_system.begin() )
//		{
//			arrayTotal[4+Node_id]=1; arrayTotal[4+Node_id+1+(TCPP_point->TCPP+1)*Silence_unit]=1; 
//			arrayTotal[4+Node_id+3+(TCPP_point->TCPP+1)*Silence_unit]=1; 
//			assignStart=4+Node_id+1; Interleaving_num++; 
//			BetaEnd[0]=4+Node_id+1+(TCPP_point->TCPP+1)*Silence_unit; BetaSize[0]=assignK;
//			if(New_coming_event->getEvent_time()>=-3 && New_coming_event->getEvent_time()<=-2)
//			{
//				logs<<"                                                         Data_on_send: "<<TCPP_point->TCPP<<" ";
//			}
//			//break;   //for non-interleaving
//		}
//		else
//		{
//			//if(j>BetaEnd[Last_BetaEnd] || j+2*assignK+3+(TCPP_point->TCPP+1)*Silence_unit+1>4+Node_id+length_TCPPs_max) break;
//			if(j+2*assignK+3+(TCPP_point->TCPP+1)*Silence_unit+1>4+Node_id+length_TCPPs_max) break;
//			else
//			{
//				arrayTotal[j]=1; arrayTotal[j+assignK+1]=1; 
//				arrayTotal[j+assignK+2+(TCPP_point->TCPP+1)*Silence_unit]=1; 
//				arrayTotal[j+2*assignK+3+(TCPP_point->TCPP+1)*Silence_unit]=1;
//				assignStart=j+assignK+2; Interleaving_num++;
//				int n;
//				for(n=0; n<Beta; n++)                    //find the position to put the end of current TCPP in BetaEnd and BetaSize
//				{	if(BetaEnd[n]==0)break;
//					if(j+assignK+2+(TCPP_point->TCPP+1)*Silence_unit<BetaEnd[n]) break;
//				}
//				if(BetaEnd[n]==0) {BetaEnd[n]=j+assignK+2+(TCPP_point->TCPP+1)*Silence_unit; BetaSize[n]=assignK; }  //insert the end of current TCPP into the BetaEnd
//				else
//				{	int m;
//					for(m=0; m<Beta; m++)
//					{if(BetaEnd[m]==0) {m=m-1; break;} }
//					for(int X=m; X>=n; X--)
//					{BetaEnd[X+1]=BetaEnd[X]; BetaSize[X+1]=BetaSize[X]; }
//					BetaEnd[n]=j+assignK+2+(TCPP_point->TCPP+1)*Silence_unit; BetaSize[n]=assignK;
//				}
//			}
//			if(New_coming_event->getEvent_time()>=-3 && New_coming_event->getEvent_time()<=-2)
//			{
//				logs<<TCPP_point->TCPP<<" ";   //for exhibiting transmitted data
//			}
//		}
//		assignK=1;
//		for(int K=0; K<Beta; K++)                               //could be optimized in the future
//		{if(assignK==BetaSize[K]) {K=-1; assignK++;} }
//	}
//	if(New_coming_event->getEvent_time()>=-3 && New_coming_event->getEvent_time()<=-2)
//	{
//		logs<<endl;    //for exhibiting transmitted data
//	}
//	delete[] BetaEnd; delete[] BetaSize;
//	return Interleaving_num;
//}


////Event_type==1
//int SYNC_sleep_time1=0; int SYNC_sleep_time2=0;
//if(New_coming_event->getEvent_time()<=Initial_listen_period)        //synchronization for the first time
//{
//	SYNC_sleep_time1=static_cast<int>((Initial_listen_period-New_coming_event->getEvent_time())/Basic_unit_duration1);
//	SYNC_sleep_time2=static_cast<int>(floor(static_cast<double>((Initial_listen_period-New_coming_event->getEvent_time())%Basic_unit_duration1)/Basic_unit_duration2));
//	this->Schedule_table_node_id[0]=this->Node_id; 
//	this->Schedule_table_sleep_time[0]=New_coming_event->getEvent_time()+SYNC_sleep_time1*Basic_unit_duration1
//		+SYNC_sleep_time2*Basic_unit_duration2;
//}
//else     //normal periodic SYNC
//{
//	SYNC_sleep_time1=static_cast<int>((this->Schedule_table_sleep_time[0]-New_coming_event->getEvent_time())/Basic_unit_duration1);
//	SYNC_sleep_time2=static_cast<int>(floor(static_cast<double>((this->Schedule_table_sleep_time[0]-New_coming_event->getEvent_time())%Basic_unit_duration1)/Basic_unit_duration2));
//	this->Timer_normal_periodic_SYNC=this->Timer_normal_periodic_SYNC+SYNC_period;   //control normal synchronization, SYNC_period=10sec
//}
////cout<<"SYNC_sleep_time1="<<SYNC_sleep_time1<<" "<<"SYNC_sleep_time2="<<SYNC_sleep_time2<<endl;  //just for debug
//			
//Event* future_event=new Event;
//int SYNC_size=0;
//if(SYNC_sleep_time2==0)
//{
//	SYNC_size=6+this->Node_id+SYNC_sleep_time1; int* SYNC=new int [SYNC_size]();
//	SYNC[0]=1; SYNC[1]=1; SYNC[3+this->Node_id]=1;
//	SYNC[4+this->Node_id+SYNC_sleep_time1]=1; SYNC[5+this->Node_id+SYNC_sleep_time1]=1;
//	future_event->setEvent_pointer(SYNC);
//	if(i==0)
//	{
//		this->Total_SYNC=this->Total_SYNC+1;
//		int N=Calculate_Pulse_count(SYNC,SYNC_size);
//		this->Total_time_interval=this->Total_time_interval+SYNC_size-N;
//		this->Pulse_count=this->Pulse_count+N;
//	}
//}
//else
//{
//	SYNC_size=7+this->Node_id+SYNC_sleep_time1+SYNC_sleep_time2;
//	int* SYNC=new int [SYNC_size](); //SYNC_send_time starts from 1, not 0.   //id is from 0, not 1.
//	SYNC[0]=1; SYNC[1]=1; SYNC[3+this->Node_id]=1; SYNC[4+this->Node_id+SYNC_sleep_time1]=1; 
//	SYNC[5+this->Node_id+SYNC_sleep_time1+SYNC_sleep_time2]=1; SYNC[6+this->Node_id+SYNC_sleep_time1+SYNC_sleep_time2]=1;
//	future_event->setEvent_pointer(SYNC);
//	if(i==0)
//	{
//		this->Total_SYNC=this->Total_SYNC+1;
//		int N=Calculate_Pulse_count(SYNC,SYNC_size);
//		this->Total_time_interval=this->Total_time_interval+SYNC_size-N;
//		this->Pulse_count=this->Pulse_count+N;
//	}
//}


////Event_type==7
//int Transmitter, Sleep_time1=0, Sleep_time2=0, m=0; 
//if(i==5)
//{
//	i=0;    // control the search position 
//	for(int k=0; k<SYNC_size; k++)
//	{
//		if(SYNC[k]==1)
//		{   i++;
//			if(i==2) Transmitter=k;
//			if(i==3) {Transmitter=k-Transmitter-2; m=k;}
//		}
//		if(i==4) {Sleep_time1=k-m-1; break; }
//	}
//}
//if(i==6)
//{
//	i=0;    // control the search position 
//	for(int k=0; k<SYNC_size; k++)
//	{
//		if(SYNC[k]==1)
//		{   i++;
//			if(i==2) m=k;
//			if(i==3) {Transmitter=k-m-2; m=k;}
//			if(i==4) {Sleep_time1=k-m-1; m=k; }
//			if(i==5) {Sleep_time2=k-m-1; break; }
//		}
//	}
//}



//Event_type==15
//int* BetaEnd=new int [250](); int* BetaSize=new int [250]();
//bool flag=false, TCPP_first_flag=true;
//BetaEnd[0]=i; BetaSize[0]=1;
//int m=1;                            //m is the order number of BetaEnd and BetaSize
//i++;
//deque<TCPP_pair> Queuing_system_temporary; 
//while(i<TCPP_Size)                  //for detect how many pair of '1s' in this array
//{
//	if(TCPP_Data[i]==1 && flag==false)
//	{   flag=true; BetaEnd[m]=i; i++; }
//	if(TCPP_Data[i]==1 && flag==true)
//	{   flag=false;
//		BetaSize[m]=i-BetaEnd[m]-1;
//		for(int n=1; n<=m; n++)                            //for detecting the matched TCPP 
//		{
//			if(BetaSize[0]==BetaSize[n])
//			{
//				if(TCPP_first_flag==true)
//				{   TCPP_first_flag=false;                         
//					if(this->Node_id==this->Receiver_id && this->Transmitter_id==Transmitter) 
//					{   
//						this->Total_num_rece_TCPP++; TCPP_pair element2;
//						element2.TCPP=(BetaEnd[n]-BetaEnd[0]-1)/Silence_unit-1; 
//						element2.Time_in=New_coming_event->getEvent_time();  //not sure about this, question?
//						Queuing_system_temporary.push_back(element2);//the node is both sender and receiver
//						if(New_coming_event->getEvent_time()>=-3 && New_coming_event->getEvent_time()<=-2)
//						{//for exhibiting received data
//							logs<<"                                                         Data_on_rece: "
//								<<Queuing_system_temporary.back().TCPP<<" ";
//						}
//					}
//				}
//				else
//				{   if(this->Node_id==this->Receiver_id && this->Transmitter_id==Transmitter) 
//					{   this->Total_num_rece_TCPP++; TCPP_pair element3;
//						element3.TCPP=(BetaEnd[n]-BetaEnd[0]-BetaSize[0]-2)/Silence_unit-1; 
//						element3.Time_in=New_coming_event->getEvent_time();  //not sure about this, question?
//						Queuing_system_temporary.push_back(element3);
//						if(New_coming_event->getEvent_time()>=-3 && New_coming_event->getEvent_time()<=-2)
//						{
//							logs<<Queuing_system_temporary.back().TCPP<<" ";    //for exhibiting received data
//						}
//					}     
//				}
//				for(int p=0; p<n; p++)               //cancel the matched TCPP from BetaEnd and BetaSize
//				{BetaEnd[p]=BetaEnd[p+1]; BetaSize[p]=BetaSize[p+1]; } 
//				for(int p=n; p<=m+1; p++)            //need to check out m+1 
//				{BetaEnd[p-1]=BetaEnd[p+1]; BetaSize[p-1]=BetaSize[p+1]; }
//				m=m-2; n=0;
//			}
//		}
//		m++;
//	}
//	i++;
//}
//delete[] BetaEnd; delete[] BetaSize;