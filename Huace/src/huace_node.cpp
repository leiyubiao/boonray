#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
//#include "ICANCmd.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
//#include "ECanVci.h"//广成的
#include "controlcan.h"//自带的

#include "gnss_pos/pos_xy.h"
#include <iomanip>

#define msleep(ms)  usleep((ms)*1000)
#define min(a,b)  (((a) < (b)) ? (a) : (b))


#define MAX_CHANNELS  2
#define CHECK_POINT  200
#define RX_WAIT_TIME  0
#define RX_BUFF_SIZE  1


#define height 0

using namespace std;

unsigned int gDevType = 0;
unsigned int gDevIdx = 0;
unsigned int gChMask = 1;
unsigned int gBaud = 0;
unsigned char gTxType = 0;
unsigned int gTxSleep = 0;
unsigned int gTxFrames = 0;
unsigned int CAN_ID_Huace=1;
unsigned int CAN_ID_CarControl=0;
ros::Publisher pub_huace_msg ;
ros::Subscriber sub_command;

geometry_msgs::PoseStamped global_command;

void printCan(VCI_CAN_OBJ can)
{
    if(can.ID==804||can.ID==805||can.ID==807||can.ID==810)//我们需要的数据,can消息的ID是unsigned int
    {
        printf("ID = %d \n",can.ID);
        printf("Data:  ");
        for(int i=0;i<8;++i)
        {
            printf("%x ",can.Data[i]);
        }
        printf("\n");
    }
}



/******车辆控制代码********/
//转角±30度，左负右正。对应-1024~1024 ,D_R=1表示前进，-1表示后退,发的刹车值为0-60,这边解析*10倍，速度m/s
VCI_CAN_OBJ generateCommand(int D_R,float speed,float steering_angle,float braking_value)
{
	if(speed>5)	
	{
		cout<<"speed= "<<speed<<" is too much"<<endl;
		speed=5;
	}

	if(steering_angle>25)	
	{
		cout<<"steering_angle= "<<steering_angle<<" is too much"<<endl;
		speed=25;
	}

	if(steering_angle<-25)	
	{
		cout<<"steering_angle= "<<steering_angle<<" is too little"<<endl;
		steering_angle=-25;
	}
	cout<<"steering_angle= "<<steering_angle<<endl;

	speed=speed*3.6;
	unsigned int brake_val=braking_value*10;
	brake_val=min(brake_val,600);//最大值为600
	if(brake_val<10)//刹车值太小，不刹车
	{	
		brake_val=0;
	}
	else//需要刹车，速度变为0
	{
		speed=0;
		cout<<"breaking value= "<<brake_val<<endl;
	}
	

	if(abs(speed)<0.1)
	{
		speed=0;
		//brake_val=600;
		//ROS_ERROR("BRAKEING...");
	
	} 
	
    VCI_CAN_OBJ can_message;
	can_message.ID=0x183;
	can_message.SendType=0;
	can_message.RemoteFlag=0;
	can_message.ExternFlag=0;
	can_message.DataLen=8;
	
	unsigned int speed_num=abs(speed*10);
	unsigned int steering_angle_num=abs(steering_angle)*1024/30;
	can_message.Data[0]=speed_num&0x00FF;
	can_message.Data[1]=(speed_num>>8)&0x00FF;


	//can_message.Data[2]=0x00;
	//can_message.Data[3]=0x00;	
    
    
     can_message.Data[2]=brake_val&0x00FF;//低8位
	 can_message.Data[3]=(brake_val>>8)&0x00FF; //高8位
    

	unsigned int steer=(steering_angle>=0)?steering_angle_num:(((~steering_angle_num)+1));
	//printf("data  %x \n",steering_angle_num);
	//printf("~  %x \n",(~steering_angle_num)&0xFFFF);
	//printf("~+1  %x \n",((~steering_angle_num)+1)&0xFFFF);
	
	can_message.Data[4]=steer&0x00FF;
	can_message.Data[5]=(steer>>8)&0x00FF;
	

	
	can_message.Data[6]=(D_R>0)?0x01:0x03;
	can_message.Data[7]=0x80;
	
	return can_message;
	
}

 
//转角±30度，左负右正。对应-1024~1024 ,D_R=1表示前进，-1表示后退
int sendCommand(int D_R,float speed,float steering_angle,float braking_value)

{
      VCI_CAN_OBJ can;

      can=generateCommand(D_R,speed,steering_angle,braking_value);
       printCan(can);

      int length=VCI_Transmit(gDevType, gDevIdx,CAN_ID_CarControl, &can, 1);//0是canID，可能为0 1;
      int flag=1;
      if (1 != length)
      {
            printf("CAN%d TX failed: ID=%08x\n", 0, can.ID);
            flag = 0;
      }
      //msleep(20);  
    return flag;
}

//pose的x存储的速度值，y存储的转角值
void car_command_msg_callback(const geometry_msgs::PoseStampedConstPtr& command_ptr)
{
    printf("get new  car control command speed=%f, steer=%f",command_ptr->pose.position.x,command_ptr->pose.position.y);
	global_command.pose.position.x=command_ptr->pose.position.x;
	global_command.pose.position.y=command_ptr->pose.position.y;
	global_command.pose.position.z=command_ptr->pose.position.z;

    //sendCommand(1,command_ptr->pose.position.x,command_ptr->pose.position.y);

}


/**************************************/


void Receive_data()
{
    VCI_CAN_OBJ can;
    int len=VCI_Receive(gDevType, gDevIdx, CAN_ID_Huace, &can, 1, 1000/*ms*/);
    printf("receive len= %d\n",len);
    printCan(can);
}
gnss_pos::pos_xy Huace_pos;
void Receive_datas()
{
    
    VCI_CAN_OBJ can[15];
    VCI_ClearBuffer(4,0, CAN_ID_Huace);
	int Rec_len=VCI_GetReceiveNum(gDevType, gDevIdx, CAN_ID_Huace);
    int len=VCI_Receive(gDevType, gDevIdx, CAN_ID_Huace, can, 15, 1000/*ms*/);//每次收10帧数据，一起处理.不过有可能没有这么多数据帧
    //printf("receive  len= %d  ",len);
    if(len<1)
    {
        ROS_ERROR("NO GPS MESSAGE");
    }
    bool flag804=false;
    bool flag805=false;
    bool flag807=false;
    bool flag810=false;
    
    
    for(int i=0;i<len;++i)
    {
        //printCan(can[i]);
        if(can[i].ID==804)//lat lon 
        {
            flag804=true;
            //cout<<hex<<can[i].Data[0]<<24<<endl;
            //printCan(can[i]);
            unsigned int PosLat= (can[i].Data[0]<<24)|(can[i].Data[1]<<16)|(can[i].Data[2]<<8)|(can[i].Data[3]);
            //unsigned int PosLat= int(can[i].Data[0])<<24+ int(can[i].Data[1])<<16+ int(can[i].Data[2])<<8+ int(can[i].Data[3]);//这种写法是错误的
            unsigned int PosLon=(can[i].Data[4]<<24)|(can[i].Data[5]<<16)|(can[i].Data[6]<<8)|(can[i].Data[7]);
            
            if(PosLat >0x80000000) 
            {
                Huace_pos.latitude = 0.0000001*(PosLat-4294967296);
                cout.setf(ios::right); // 设置对齐方式
                //cout.width(9);
                //cout<<"  latitude1 = "<<setprecision(12)<<Huace_pos.latitude;
            }
				
			else
            {
                Huace_pos.latitude = 0.0000001*PosLat;
                cout.setf(ios::right); // 设置对齐方式
                //cout.width(9);
               // cout<<"  latitude2 = "<<setprecision(12)<<Huace_pos.latitude<<endl;

            }
				

			if (PosLon >0x80000000)
            {
                cout.setf(ios::right); // 设置对齐方式
                //cout.width(9);
                Huace_pos.longitude = 0.0000001*(PosLon-4294967296);
                //cout<<"  longitude1 = "<<setprecision(12)<<Huace_pos.longitude;
            }
				
		    else
            {
                cout.setf(ios::right); // 设置对齐方式
                 //cout.width(9);
                Huace_pos.longitude = 0.0000001*PosLon;
                //cout<<"  longitude2 = "<<setprecision(12)<<Huace_pos.longitude<<endl;
            }
				
        }
        if(can[i].ID==805)
        {
            flag805=true;
            //printCan(can[i]);
            unsigned int PosAlt = (can[i].Data[0]<<24)|(can[i].Data[1]<<16)|(can[i].Data[2]<<8)|(can[i].Data[3]);

			if (PosAlt>0x80000000)
            {
                cout.setf(ios::right); // 设置对齐方式
                 cout.width(9);
                Huace_pos.altitude = 0.001*(PosAlt-4294967296);
                //cout<<"altitude1 = "<<Huace_pos.altitude<<endl;
            }
				
			else
            {
                cout.setf(ios::right); // 设置对齐方式
                cout.width(9);
                Huace_pos.altitude = 0.001*PosAlt; 
                //cout<<"altitude2 = "<<Huace_pos.altitude<<endl;

            }
				        
        }

        if(can[i].ID==807)
        {
            flag807=true;
            //printCan(can[i]);
            unsigned int pspeed = (can[i].Data[6]<<8)|(can[i].Data[7]);

            if( pspeed> 0x8000)
            {
                Huace_pos.speed2D = (pspeed-65536) * 0.01;
                //cout<<"speed2D1 = "<<Huace_pos.speed2D<<endl;

            }
				
			else
            {
                Huace_pos.speed2D = pspeed * 0.01 ;
                //cout<<"speed2D2 = "<<Huace_pos.speed2D<<endl;

            }
				
        }

       if(can[i].ID==810)
       {
           flag810=true;
            //printCan(can[i]);
			unsigned int pheading = (can[i].Data[0]<<8)|(can[i].Data[1]);
			Huace_pos.heading = 0.01*pheading;
            //cout<<"heading = "<<Huace_pos.heading<<endl;
       }
					

    }
    if(flag804&&flag807&&flag810)//收到其中一个
	{
		  pub_huace_msg.publish(Huace_pos);
        //cout<<"  latitude = "<<setprecision(12)<<Huace_pos.latitude<<"  longitude = "<<setprecision(12)<<Huace_pos.longitude<<"  speed2D = "<<Huace_pos.speed2D<<"  heading = "<<Huace_pos.heading<<endl;

		
		//cout<<endl;
	}
      
	
    //sendCommand(1,global_command.pose.position.x,global_command.pose.position.y);
    //printf("get car control command speed=%f, steer=%f",global_command.pose.position.x,global_command.pose.position.y);
}



unsigned int s2n(const char *s)
{
    unsigned l = strlen(s);
    unsigned v = 0;
    unsigned h = (l > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
    unsigned char c;
    unsigned char t;
    if (!h) return atoi(s);
    if (l > 10) return 0;
    for (s += 2; c = *s; s++)
    {
        if (c >= 'A' && c <= 'F') c += 32;
        if (c >= '0' && c <= '9') t = c - '0';
        else if (c >= 'a' && c <= 'f') t = c - 'a' + 10;
        else return 0;
        v = (v << 4) | t;
    }
    return v;
}

bool init()
{
    gDevType = s2n("4");        //argv[1]   //设备类型号 3-USBCAN I, 4-USBCAN II 
    gDevIdx = s2n("0");         //argv[2]   //设备索引号 0-只有一个设备，0 or 1-有两个设备
    gChMask = s2n("1");         //argv[3]   //CAN通道号 0-CAN0, 1-CAN1
    gBaud = s2n("0x1c00");      //argv[4]   //通讯波特率：0x1C00=500kbps
    gTxType = s2n("0");         //argv[5]   //发送类型 normal
    gTxSleep = s2n("1");        //argv[6]   //延时时间ms
    gTxFrames = s2n("1000");    //argv[7]
    printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d)\n",
        gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames);

    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) 
    {     //开启设备
        printf("VCI_OpenDevice failed\n");
        return false;
    }
    printf("VCI_OpenDevice succeeded\n");


    // ----- init & start -------------------------------------------------

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = 0x00;
    config.Timing1 = 0x1c;
   
    //if ((gChMask & (1 << i)) == 0) continue;
    for(int CAN_ID=0;CAN_ID<2;++CAN_ID)
	{
		if (!VCI_InitCAN(gDevType, gDevIdx, CAN_ID, &config))
		{
		    printf("VCI_InitCAN(%d) failed\n", CAN_ID);
		    return false;
		}
		printf("VCI_InitCAN(%d) succeeded\n", CAN_ID);
		//VCI_ClearBuffer(gDevType, gDevIdx,CAN_ID);
		if (!VCI_StartCAN(gDevType, gDevIdx, CAN_ID))
		{
		    printf("VCI_StartCAN(%d) failed\n", CAN_ID);
		    return false;
		}
		printf("VCI_StartCAN(%d) succeeded\n", CAN_ID);
    }
    return true;

}    



int main(int argc, char **argv)
{
	ros::init(argc, argv, "huace_node");
	ros::NodeHandle nh;

	global_command.pose.position.x=0;
	global_command.pose.position.y=0;


	pub_huace_msg = nh.advertise<gnss_pos::pos_xy>("/huace_msg", 1);
	sub_command=nh.subscribe<geometry_msgs::PoseStamped>("/car_command", 1, car_command_msg_callback);
	if(!init())
    {
        ROS_ERROR("device open failed");
        return 0;
    }
		
	ros::Rate r(100);
    while(ros::ok()){
		ros::spinOnce();
        Receive_datas();
        r.sleep();
		//int value=sendCommand(1,0,-10);
        sendCommand(1,global_command.pose.position.x,global_command.pose.position.y,global_command.pose.position.z);
		
    }
    VCI_CloseDevice(gDevType, gDevIdx);         //关闭设备
    printf("VCI_CloseDevice\n");
    return 0;
}
