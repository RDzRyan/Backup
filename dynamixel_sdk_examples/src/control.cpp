/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples bulk_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /bulk_set_item dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'position', item2: 'LED', value1: 1000, value2: 1}"
 * $ rostopic pub -1 /bulk_set_item dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'LED', item2: 'position', value1: 1, value2: 1000}"
 * $ rosservice call /bulk_get_item "{id1: 1, id2: 2, item1: 'position', item2: 'LED'}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <unistd.h>

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE 24
#define ADDR_PRESENT_LED 25
#define ADDR_PRESENT_POSITION 36
#define ADDR_GOAL_POSITION 30

// Protocol version
#define PROTOCOL_VERSION 1.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID 11                 // DXL1 ID
#define DXL2_ID 12                 // DXL2 ID
#define BAUDRATE 115200            // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

int dmx_id[18] = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53, 61, 62, 63};
uint8_t step[18][10] = {
    // 11   12   13   21   22   23   31   32   33   41   42   43   51   52   53   61   62   63
    {650, 650, 500, 500, 500, 500, 650, 650, 500, 500, 500, 500, 350, 350, 500, 500, 500, 500}, // 0 no repeat
    {650, 650, 550, 500, 500, 500, 650, 650, 550, 500, 500, 500, 350, 350, 450, 500, 500, 500}, // 1 no repeat
    {500, 500, 550, 500, 500, 500, 500, 500, 550, 500, 500, 500, 500, 500, 450, 500, 500, 500}, // 2 no repeat
    {500, 500, 550, 650, 650, 500, 500, 500, 550, 350, 350, 500, 500, 500, 450, 350, 350, 500}, // 3 no repeat
    {500, 500, 450, 650, 650, 550, 500, 500, 450, 350, 350, 450, 500, 500, 550, 350, 350, 450}, // 4
    {500, 500, 450, 500, 500, 550, 500, 500, 450, 500, 500, 450, 500, 500, 550, 500, 500, 450}, // 5
    {650, 650, 450, 500, 500, 550, 650, 650, 450, 500, 500, 450, 350, 350, 550, 500, 500, 450}, // 6
    {650, 650, 550, 500, 500, 450, 650, 650, 550, 500, 500, 550, 350, 350, 450, 500, 500, 550}, // 7
    {500, 500, 550, 500, 500, 450, 500, 500, 550, 500, 500, 550, 500, 500, 450, 500, 500, 550}, // 8
    {500, 500, 550, 650, 650, 450, 500, 500, 550, 350, 350, 550, 500, 500, 450, 350, 350, 550}  // 9
};
int i = 0;

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

void bulkSetItemCallback(const dynamixel_sdk_examples::BulkSetItem::ConstPtr &msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position1[4];
  uint8_t param_goal_position2[4];
  uint8_t param_goal_led1[1];
  uint8_t param_goal_led2[1];
  uint8_t addr_goal_item[2];
  uint8_t len_goal_item[2];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  if (msg->item1 == "position")
  {
    uint32_t position1 = (unsigned int)msg->value1; // Convert int32 -> uint32
    param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
    param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
    param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(position1));
    param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(position1));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;
  }
  else if (msg->item1 == "LED")
  {
    uint32_t led1 = (unsigned int)msg->value1; // Convert int32 -> uint32
    param_goal_led1[0] = led1;
    addr_goal_item[0] = ADDR_PRESENT_LED;
    len_goal_item[0] = 1;
  }

  if (msg->item2 == "position")
  {
    uint32_t position2 = (unsigned int)msg->value2; // Convert int32 -> uint32
    param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
    param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
    param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(position2));
    param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(position2));
    addr_goal_item[1] = ADDR_GOAL_POSITION;
    len_goal_item[1] = 4;
  }
  else if (msg->item2 == "LED")
  {
    uint32_t led2 = (unsigned int)msg->value2; // Convert int32 -> uint32
    param_goal_led2[0] = led2;
    addr_goal_item[1] = ADDR_PRESENT_LED;
    len_goal_item[1] = 1;
  }

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  if (msg->item1 == "position")
  {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id1, addr_goal_item[0], len_goal_item[0], param_goal_position1);
  }
  else if (msg->item1 == "LED")
  {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id1, addr_goal_item[0], len_goal_item[0], param_goal_led1);
  }
  if (dxl_addparam_result != true)
  {
    ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", msg->id1);
  }

  if (msg->item2 == "position")
  {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id2, addr_goal_item[1], len_goal_item[1], param_goal_position2);
  }
  else if (msg->item2 == "LED")
  {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id2, addr_goal_item[1], len_goal_item[1], param_goal_led2);
  }
  if (dxl_addparam_result != true)
  {
    ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", msg->id2);
  }

  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS)
  {
    ROS_INFO("setItem : [ID:%d] [%s:%d]", msg->id1, msg->item1.c_str(), msg->value1);
    ROS_INFO("setItem : [ID:%d] [%s:%d]", msg->id2, msg->item2.c_str(), msg->value2);
  }
  else
  {
    ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupBulkWrite.clearParam();
}

int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t param_goal_position[18];
  uint8_t param_goal_ID[18];
  param_goal_position[0] = 500;

  if (!portHandler->openPort())
  {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE))
  {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  for (i = 0; i <= 18; i++)
  {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)dmx_id[a]->param_goal_ID[0], ADDR_GOAL_POSITION, 4, param_goal_position[0]);
    // dxl_addparam_result = groupBulkWrite.addParam((uint8_t)dmx_id[a], ADDR_GOAL_POSITION, 4, step[a][b]);

    // dxl_comm_result = packetHandler->write2ByteTxRx(
    //     portHandler, dmx_id[i], ADDR_GOAL_POSITION, 500, &dxl_error);
  }
  dxl_comm_result = groupBulkWrite.txPacket();

  // while (1)
  // {
  //   for (int a = 0; a <= 18; a++)
  //   {
  //     for (int b = 4; b <= 10; b++)
  //     {
  //       // dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->dmx_id[a], ADDR_GOAL_POSITION, 4, step[a][b]);
  //       dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->dmx_id[a], ADDR_GOAL_POSITION, 4, 500);
  //     }
  //   }
  //   dxl_comm_result = groupBulkWrite.txPacket();
  // }

  ros::init(argc, argv, "bulk_read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/bulk_get_item", bulkGetItemCallback);
  ros::Subscriber bulk_set_item_sub = nh.subscribe("/bulk_set_item", 10, bulkSetItemCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
