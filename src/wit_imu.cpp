#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<time.h>
#include<sys/types.h>
#include<errno.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/shared_ptr.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/MagneticField.h>

static int ret;
static int fd;

// #define BAUD 115200 //for JY61 ,9600 for others
#define BAUD 115200


int uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("open %s success!\n",pathname);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 
    return fd; 
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
     struct termios newtio,oldtio; 
     if  ( tcgetattr( fd,&oldtio)  !=  0) {  
      perror("SetupSerial 1");
	  printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
      return -1; 
     } 
     bzero( &newtio, sizeof( newtio ) ); 
     newtio.c_cflag  |=  CLOCAL | CREAD;  
     newtio.c_cflag &= ~CSIZE;  
     switch( nBits ) 
     { 
     case 7: 
      newtio.c_cflag |= CS7; 
      break; 
     case 8: 
      newtio.c_cflag |= CS8; 
      break; 
     } 
     switch( nEvent ) 
     { 
     case 'o':
     case 'O': 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag |= PARODD; 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      break; 
     case 'e':
     case 'E': 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag &= ~PARODD; 
      break;
     case 'n':
     case 'N': 
      newtio.c_cflag &= ~PARENB; 
      break;
     default:
      break;
     } 

     /*设置波特率*/ 

switch( nSpeed ) 
     { 
     case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 
     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 
     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 
     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 
     case 460800: 
      cfsetispeed(&newtio, B460800); 
      cfsetospeed(&newtio, B460800); 
      break; 
     default: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 
     } 
     if( nStop == 1 ) 
      newtio.c_cflag &=  ~CSTOPB; 
     else if ( nStop == 2 ) 
      newtio.c_cflag |=  CSTOPB; 
     newtio.c_cc[VTIME]  = 0; 
     newtio.c_cc[VMIN] = 0; 
     tcflush(fd,TCIFLUSH); 

if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
     { 
      perror("com set error"); 
      return -1; 
     } 
     printf("set done!\n"); 
     return 0; 
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    return 0;
}
int send_data(int  fd, char *send_buffer,int length)
{
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;
}
int recv_data(int fd, char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	return length;
}

static geometry_msgs::TransformStamped toTFTransform(tf2::Quaternion q,
                                            const std::string &frame_id,
                                            const std::string &child_frame_id) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    return transformStamped;
}

float a[3],w[3],Angle[3],mag[3], quat[4];
int all_data_received = 0;
int seq = 0;
void ParseData(char chr, ros::Publisher imu_pub, tf2_ros::TransformBroadcaster tf_pub, ros::Publisher mag_pub)
{
		static char chrBuf[100];
		static unsigned char chrCnt=0;
		signed short sData[4];
		unsigned char i;
		char cTemp=0;
		time_t now;
		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return;
		for (i=0;i<10;i++) cTemp+=chrBuf[i];
		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {
            printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);
            chrCnt--;
            return;
        }
		memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
                {
                    for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
					time(&now);
                    all_data_received |= 1;
					break;
                }
				case 0x52:
                {
                    // GYRO OUTPUT 
					for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;				
                    all_data_received |= 1 << 1;
					break;
                }
				case 0x53:
                {
                    for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
					break;
                }
					
				case 0x54: // Magnetic
                {
					for (i=0;i<3;i++) mag[i] = (float)sData[i];
                    all_data_received |= 1 << 2;
					break;
                }
                case 0x59:
                {
                    for (i=0; i<4; i++) quat[i] = (float)sData[i] / 32768.0;
                    all_data_received |= 1 << 3;
                    break;
                }
		}		
        if( all_data_received & 0xF == 0xF ){
            printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
            printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);				
            printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
            printf("mag:%4.0f %4.0f %4.0f ",mag[0],mag[1],mag[2]);
            sensor_msgs::Imu data;
            data.linear_acceleration.x = (double) a[0] * 9.8066;
            data.linear_acceleration.y = (double) a[1] * 9.8066;
            data.linear_acceleration.z = (double) a[2] * 9.8066;
            data.angular_velocity.x = w[0] * (3.1415/180.0);
            data.angular_velocity.y = w[1] * (3.1415/180.0);
            data.angular_velocity.z = w[2] * (3.1415/180.0);
            // float roll = (Angle[0])*(3.1415/180.0);
            // float pitch = Angle[1]*(3.1415/180.0);
            // float yaw = Angle[2]*(3.1415/180.0);
            //Convert from euler angles to quaternion
            tf2::Quaternion myQuaternion(quat[0], quat[1], quat[2], quat[3]);
            // myQuaternion.setRPY( roll, pitch, yaw );

            data.orientation = tf2::toMsg(myQuaternion); // comment back in for fused witmotion estimate

            // geometry_msgs::Vector3 accel_vec3;
            // geometry_msgs::Vector3 gyro_vec3;
            data.header.stamp = ros::Time::now();
            data.header.frame_id = "imu_world"; //ros::this_node::getNamespace() + "wit_imu";
            data.header.seq = seq;
            
            // data.header.frame_id.erase(data.header.frame_id.begin());
            imu_pub.publish( data );

            sensor_msgs::MagneticField mag_data;
            mag_data.header.stamp = ros::Time::now();
            mag_data.header.frame_id = "imu_world";
            mag_data.header.seq = seq;
            mag_data.magnetic_field.x = mag[0];
            mag_data.magnetic_field.y = mag[1];
            mag_data.magnetic_field.z = mag[2];
            mag_pub.publish(mag_data);

            all_data_received = 0;
            seq += 1;
        }
		chrCnt=0;		
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wit_imu");
    ros::NodeHandle n;
    static tf2_ros::TransformBroadcaster tf_pub;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw",100);
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag",100);
    std::string device;
    device = ros::param::param< std::string >("~device", "/dev/ttyUSB0");
    std::cout << device << std::endl;

    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,const_cast<char*>(device.c_str()));/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */ 
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,BAUD,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

	//iFILE *fp;
	//fp = fopen("Record.txt","w");
    ros::Rate r(10);
    while( ros::ok() )
    {
        ret = recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) {
            //fprintf(fp,"%2X ",r_buf[i]);
            ParseData(r_buf[i],imu_pub,tf_pub,mag_pub);}
        r.sleep();
        // usleep(500);
    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
