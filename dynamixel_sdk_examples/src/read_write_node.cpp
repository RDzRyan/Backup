// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               13               // DXL1 ID
#define DXL2_ID               21               // DXL2 ID
#define BAUDRATE              115200          // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;
int dmx_id[18]={11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63};
int i=0;
int step=0;
int k=0;
int l=0;

bool getPresentPositionCallback(
  dynamixel_sdk_examples::GetPosition::Request & req,
  dynamixel_sdk_examples::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)msg->position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }	

	for (i=0;i<=18;i++){
		dxl_comm_result = packetHandler->write4ByteTxRx(
    	portHandler, dmx_id[i], ADDR_GOAL_POSITION, 500, &dxl_error);
	}
	//sleep(3);
//	while(1){
	//	usleep(200*1000);
	//	switch(step){
	//		case 0:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 650, &dxl_error);
		//					dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 550, &dxl_error);
				//			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 650, &dxl_error);
					//		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 650, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 550, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 350, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 350, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 450, &dxl_error);
						//	
						//	
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 450, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 550, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 500, &dxl_error);
						//	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 550, &dxl_error);
						//	step=step+1;
				//break;
			//case 1:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				//

			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				step=step+1;
			//	break;
			//case 2:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				
			//				
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				step=step+1;
			//	break;
			//case 3:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				step=step+1;
			//	break;
			//case 4:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 650, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 350, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				
			//				step=step+1;
			//	break;
			//case 5:	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 21, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 22, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 23, ADDR_GOAL_POSITION, 450, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 41, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 42, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 43, ADDR_GOAL_POSITION, 550, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 61, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 62, ADDR_GOAL_POSITION, 500, &dxl_error);
			//				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 63, ADDR_GOAL_POSITION, 550, &dxl_error);//

	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 11, ADDR_GOAL_POSITION, 650, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 12, ADDR_GOAL_POSITION, 650, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 13, ADDR_GOAL_POSITION, 550, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 31, ADDR_GOAL_POSITION, 650, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 32, ADDR_GOAL_POSITION, 650, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 33, ADDR_GOAL_POSITION, 550, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 51, ADDR_GOAL_POSITION, 350, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 52, ADDR_GOAL_POSITION, 350, &dxl_error);
	//						dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 53, ADDR_GOAL_POSITION, 450, &dxl_error);
	//						
	//						step=step+1;
	//			break;
	//		default:step=0;
	//	}
	//}
  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
