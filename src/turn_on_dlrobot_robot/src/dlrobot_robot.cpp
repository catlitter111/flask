#include "turn_on_dlrobot_robot/dlrobot_robot.h"
#include "rclcpp/rclcpp.hpp"
#include "turn_on_dlrobot_robot/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"     // CHANGE
#include "dlrobot_robot_msg/msg/data.hpp"     // CHANGE

//sensor_msgs::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 
sensor_msgs::msg::Imu Mpu6050;
using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //auto node= std::make_shared<turn_on_robot>();

    turn_on_robot Robot_Control;
    Robot_Control.Control();
    return 0;
}
/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/

short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;   
  transition_16 |=  Data_Low;
  return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机

***************************************/
void turn_on_robot::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl) 
{
  short  transition;  //intermediate variable //中间变量
  //if(akm_cmd_vel=="ackermann_cmd") {RCLCPP_INFO(this->get_logger(),"is akm");} //Prompt message //提示信息
  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
  Send_Data.tx[1] = 0; //set aside //预留位
  Send_Data.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = akm_ctl->drive.speed*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  //transition=0;
  //transition = twist_aux->linear.y*1000;
  //Send_Data.tx[6] = transition;
  //Send_Data.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = akm_ctl->drive.steering_angle*1000/2;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BBC check bits, see the Check_Sum function //BBC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D

  // 打印要发送的串口数据
  RCLCPP_INFO(this->get_logger(), "🚗📤 [阿克曼模式] 发送串口数据 (速度=%.3f, 转向角=%.3f):", 
              akm_ctl->drive.speed, akm_ctl->drive.steering_angle);
  printf("串口数据: ");
  for(int i = 0; i < sizeof(Send_Data.tx); i++) {
    printf("0x%02X ", Send_Data.tx[i]);
  }
  printf("\n");

  try
  { 
 Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    RCLCPP_INFO(this->get_logger(), "✅ 串口数据发送成功");
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

//void turn_on_robot::Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl) 
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short  transition;  //intermediate variable //中间变量
  //if(akm_cmd_vel=="none") {RCLCPP_INFO(this->get_logger(),"not akm");} //Prompt message //提示信息
  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
  Send_Data.tx[1] = 0; //set aside //预留位
  Send_Data.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux->linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux->angular.z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BBC check bits, see the Check_Sum function //BBC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D

  // 打印要发送的串口数据
  RCLCPP_INFO(this->get_logger(), "🚗📤 [普通模式] 发送串口数据 (线速度=%.3f, 角速度=%.3f):", 
              twist_aux->linear.x, twist_aux->angular.z);
  printf("串口数据: ");
  for(int i = 0; i < sizeof(Send_Data.tx); i++) {
    printf("0x%02X ", Send_Data.tx[i]);
  }
  printf("\n");

  try
  {
    if(akm_cmd_vel=="none")  
    {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
      RCLCPP_INFO(this->get_logger(), "✅ 串口数据发送成功");
    }
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/

void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  

  imu_publisher->publish(Imu_Data_Pub);
 
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/

void turn_on_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
     //把Z轴转角转换为四元数进行表达

    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);

    dlrobot_robot_msg::msg::Data robotpose;
    dlrobot_robot_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据

    odom.header.stamp = rclcpp::Node::now(); 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标

    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数


    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

 /*   geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.header.stamp = rclcpp::Node::now();

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;

    tf_bro->sendTransform(odom_tf);

*/
    //There are two types of this matrix, which are used when the robot is at rest 
    //and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    //tf_pub_->publish(odom_tf);
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
    robotpose_publisher->publish(robotpose); //Pub odometer topic //发布里程计话题
    robotvel_publisher->publish(robotvel); //Pub odometer topic //发布里程计话题

}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/

void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
        voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
      }
}/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BBC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BBC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}


/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/

bool turn_on_robot::Get_Sensor_Data()
{ 
  short transition_16=0, j=0, Header_Pos=0, Tail_Pos=0; //Intermediate variable //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据
 //Record the position of the head and tail of the frame //记录帧头帧尾位置
  for(j=0;j<24;j++)
  {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
  }

  if(Tail_Pos==(Header_Pos+23))
  {
    //If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
    //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
    // ROS_INFO("1----");
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
  }
  else if(Header_Pos==(1+Tail_Pos))
  {
    //如果帧头在帧尾后面，纠正数据位置后复制数据包到Receive_Data.rx
    // If the header is behind the end of the frame, copy the packet to receive_data.rx after correcting the data location
    // ROS_INFO("2----");
    for(j=0;j<24;j++)
    Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%24];
  }
  else 
  {
    //其它情况则认为数据包有错误
    // In other cases, the packet is considered to be faulty
    // ROS_INFO("3----");
    return false;
  }    

  Receive_Data.Frame_Header= Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail= Receive_Data.rx[23];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER ) //Judge the frame header //判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //Judge the end of the frame //判断帧尾
    { 
      //BBC check passes or two packets are interlaced //BBC校验通过或者两组数据包交错
      if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)||(Header_Pos==(1+Tail_Pos))) 
      {
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); 
        //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); 
                                                                         //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
        
        //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
        //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        //Get the battery voltage
        //获取电池电压
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;
        transition_16 |=  Receive_Data.rx[21];  
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)

        return true;
     }
    }
  } 
  return false;
}
/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/

void turn_on_robot::Control()
{

  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
    current_time = rclcpp::Node::now();
    //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 
    //获取时间间隔，用于积分速度获得位移(里程)
    Sampling_Time = (current_time - last_time).seconds(); 

    //The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
    //通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    if (true == Get_Sensor_Data()) 
                                   
    {
      //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m 
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time;

      //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
      //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      Publish_ImuSensor(); //Pub the IMU topic //发布IMU话题    
      Publish_Voltage();   //Pub the topic of power supply voltage //发布电源电压话题
      Publish_Odom();

      rclcpp::spin_some(this->get_node_base_interface());

    }

    last_time = current_time; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔

    }
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot()
: rclcpp::Node ("dlrobot_robot")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200;

  this->declare_parameter("serial_baud_rate", serial_baud_rate); // 115200 是一个常见的默认波特率
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom_combined", 10);
  //odom_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("mobile_base/sensors/imu_data", 10);    // CHANGE
  //imu_timer = create_wall_timer(1s/100, [=]() { Publish_ImuSensor(); });

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  //voltage_timer = create_wall_timer(1s/100, [=]() { Publish_Voltage(); });    
  //tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  robotpose_publisher = create_publisher<dlrobot_robot_msg::msg::Data>("robotpose", 10);
  //robotpose_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

  robotvel_publisher = create_publisher<dlrobot_robot_msg::msg::Data>("robotvel", 10);
  //robotvel_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });
  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 100, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 100, std::bind(&turn_on_robot::Akm_Cmd_Vel_Callback, this, _1));
  
  try
  { 
   // Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"dlrobot_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"dlrobot_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/

turn_on_robot::~turn_on_robot()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 

  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口  
  RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
  
}




