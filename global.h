#ifndef GLOBAL_H
#define GLOBAL_H

// 机器人型号
#define ROBOT_ZHICHANG      0
#define ROBOT_NABOTE        1

// 连接模式
#define LINK_MODBUS_TCP     0
//#define LINK_NORMAL_ASCII   1
//#define LINK_NORMAL_RTU     2
#define LINK_KAWASAKI       3
//#define LINK_CUSTOM         4

#define USE_PARENTHESES_INSTEAD_QUOTATION       //json协议用括号代替引号
#define DEL_SPACE_AND_LINEN                     //json协议去掉\n

#define MODBUS_REGISTERS_MAXNUM      0x200//125    //寄存器最大值//MODBUS_MAX_READ_REGISTERS每次连续读取最大值

//智昌寄存器
#define MODBUS_ADD_VERSION      0           //版本号
#define MODBUS_ADD_DELAY        1           //延迟时间
#define MODBUS_ADD_SEARCHSTAT   2           //搜索焊缝状态
#define MODBUS_ADD_WELD_Y_POS   3           //焊缝Y轴坐标
#define MODBUS_ADD_WELD_Z_POS   4           //焊缝Z轴坐标
#define MODBUS_ADD_WELD_W_SIZE  5           //焊缝宽度尺寸
#define MODBUS_ADD_WELD_H_SIZE  6           //焊缝高度尺寸
#define MODBUS_ADD_POS          30          //6维坐标位置存储地址0x1e
#define MODBUS_ADD_MOVETO       50          //机器人移动位置存储地址0x32
#define MODBUS_ADD_LASER        257         //激光状态存储地址0x101
#define MODBUS_ADD_TASKNUM      258         //任务号存储地址0x102

//摩卡纳伯特寄存器
#define MODBUS_NABO_TASKNUM      0x00           //任务号存储地址0
#define MODBUS_NABO_LASER        0x01           //激光状态存储地址1
#define MODBUS_NABO_SEARCH       0x02           //寻位状态存储地址2
#define MODBUS_NABO_WELDING      0x03           //跟踪状态存储地址3
#define MODBUS_NABO_SEARCHSTAT   0x10           //搜索焊缝状态
#define MODBUS_NABO_WELD_X_POS   0x11           //焊缝X轴坐标
#define MODBUS_NABO_WELD_Y_POS   0x12           //焊缝Y轴坐标
#define MODBUS_NABO_WELD_Z_POS   0x13           //焊缝Z轴坐标


#define ASK_GETPOS_ASCII        "GetPos:"   //请求坐标
#define ASE_GETPOS_ASCII        "Pos:"      //回复坐标
#define ASK_MOVETO_ASCII        "Moveto:"   //请求移动
#define ASE_MOVETO_ASCII        "Moved:"    //回复移动
#define ASK_MOVETOSP_ASCII      "speed:"    //请求移动速度
#define ASE_MOVETOSP_ASCII      "speed:"    //请求移动速度


#define RTU_MARK_ID_NUM         6               //RTU头文件包字符数
#define ASK_GETPOS_RTU          {1,2,3,4,5,6}   //请求坐标
#define ASE_GETPOS_RTU          {6,5,4,3,2,1}   //回复坐标
#define ASK_MOVETO_RTU          {3,2,1,1,2,3}   //请求移动
#define ASE_MOVETO_RTU          {3,2,1,3,2,1}   //回复移动


#define ASK_LASER_KEY_KAWASAKI          "leaser"            //请求激光KEY
#define ASK_LASER_OPEN_KAWASAKI         "open"              //请求开激光
#define ASK_LASER_CLOSE_KAWASAKI        "close"             //请求关激光
#define ASE_LASER_KEY_KAWASAKI          "leaser"            //答复激光KEY
#define ASE_LASER_OPENOK_KAWASAKI       "open ok"           //开激光成功
#define ASE_LASER_OPENNG_KAWASAKI       "open ng"           //开激光失败
#define ASE_LASER_CLOSEOK_KAWASAKI      "close ok"          //关激光成功
#define ASE_LASER_CLOSENG_KAWASAKI      "close ng"          //关激光失败

#define ASK_CAMERA_KEY_KAWASAKI         "camera"            //请求相机KEY
#define ASK_CAMERA_OPEN_KAWASAKI        "open"              //请求开相机
#define ASK_CAMERA_CLOSE_KAWASAKI       "close"             //请求关相机
#define ASE_CAMERA_KEY_KAWASAKI         "camera"            //答复相机KEY
#define ASE_CAMERA_OPENOK_KAWASAKI      "open ok"           //开相机成功
#define ASE_CAMERA_OPENNG_KAWASAKI      "open ng"           //开相机失败
#define ASE_CAMERA_CLOSEOK_KAWASAKI     "close ok"          //关相机成功
#define ASE_CAMERA_CLOSENG_KAWASAKI     "close ng"          //关相机失败

#define ASK_TASKNUM_KEY_KAWASAKI        "tasknum"           //设置任务号KEY
#define ASE_TASKNUM_KEY_KAWASAKI        "tasknum"           //答复任务号KEY可
#define ASE_TASKNUM_SETOK_KAWASAKI      " ok"               //设置任务号成功
#define ASE_TASKNUM_SETNG_KAWASAKI      " ng"               //设置任务号失败

#define ASK_DELAY_KEY_KAWASAKI          "getdelay"          //请求获取延迟KEY
#define ASK_DELAY_ONCE_KAWASAKI         "once"              //请求获取延迟一次
#define ASE_DELAY_KEY_KAWASAKI          "getdelay"          //答复获取延迟一次
#define ASE_DELAY_FAILED_KAWASAKI       "failed"            //答复获取延迟失败

#define ASK_SEARCHSTAT_KEY_KAWASAKI     "getsearchstat"     //请求获取搜索状态KEY
#define ASK_SEARCHSTAT_ONCE_KAWASAKI    "once"              //请求获取搜索状态一次
#define ASE_SEARCHSTAT_KEY_KAWASAKI     "getsearchstat"     //答复获取搜索状态一次
#define ASE_SEARCHSTAT_FAILED_KAWASAKI  "failed"            //答复获取搜索状态失败

#define ASK_POS2_KEY_KAWASAKI           "getpos2"           //请求获取2维坐标KEY
#define ASK_POS2_ONCE_KAWASAKI          "once"              //请求获取2维坐标一次
#define ASE_POS2_KEY_KAWASAKI           "getpos2"           //答复获取2维坐标一次
#define ASE_POS2_FAILED_KAWASAKI        "failed"            //答复获取2维坐标失败

#define ASK_SIZE2_KEY_KAWASAKI          "getsize2"          //请求获取2维尺寸KEY
#define ASK_SIZE2_ONCE_KAWASAKI         "once"              //请求获取2维尺寸一次
#define ASE_SIZE2_KEY_KAWASAKI          "getsize2"          //答复获取2维尺寸一次
#define ASE_SIZE2_FAILED_KAWASAKI       "failed"            //答复获取2维尺寸失败

#define ASK_POS6_KEY_KAWASAKI           "getpos6"           //请求获取6维坐标KEY
#define ASK_POS6_ONCE_KAWASAKI          "once"              //请求获取6维坐标一次
#define ASE_POS6_KEY_KAWASAKI           "getpos6"           //答复获取6维坐标一次
#define ASE_POS6_FAILED_KAWASAKI        "failed"            //答复获取6维坐标失败


#endif // GLOBAL_H
