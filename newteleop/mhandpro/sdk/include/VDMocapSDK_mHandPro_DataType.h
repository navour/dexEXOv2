#pragma once
#ifndef VDMOCAPSDK_MHANDPRO_DATATYPE_H
#define VDMOCAPSDK_MHANDPRO_DATATYPE_H

#define NODES_HAND 20
#define NODES_BODY 23

#define PC_FINGERS_VIRTUAL 5

namespace mHandProDevice
{
	/**
	* @brief
	*   hand nodes name and its index
	*/
	typedef enum HANDNODES
	{
		HN_Hand = 0,
		HN_ThumbFinger,
		HN_ThumbFinger1,
		HN_ThumbFinger2,
		HN_IndexFinger,
		HN_IndexFinger1,
		HN_IndexFinger2,
		HN_IndexFinger3,
		HN_MiddleFinger,
		HN_MiddleFinger1,
		HN_MiddleFinger2,
		HN_MiddleFinger3,
		HN_RingFinger,
		HN_RingFinger1,
		HN_RingFinger2,
		HN_RingFinger3,
		HN_PinkyFinger,
		HN_PinkyFinger1,
		HN_PinkyFinger2,
		HN_PinkyFinger3,
	}_HandNodes_;

	typedef enum SENSORSTATE {
		SS_NONE = 0,                    //未设此传感器节点
		SS_Well,                        //正常
		SS_NoData,                      //无数据
		SS_UnReady,                     //初始化中
		SS_BadMag,                      //磁干扰
	}_SensorState_;

	typedef enum GLOVEMODE {
		GM_NONE = 0,
		GM_RightGlove,
		GM_LeftGlove,
		GM_BothGloves,
	}_GloveMode_;

	typedef enum WORLDSPACE {
		WS_Geo = 0,                        //表示世界坐标系为地理坐标系
		WS_Unity,                          //表示世界坐标系为Unity世界坐标系
		WS_UE4,                            //表示世界坐标系为UE4世界坐标系
	}_WorldSpace_;

	typedef enum CALIBRATIONMODE {
		CM_Apose = 0, //Apose标定模式，所述Apose：站立，双腿伸直平行，双脚掌指向正前方，双手自然垂下平行且掌心相对，手指伸直，大拇指与食指呈45度夹角关系，另外在标定过程中不可往前或往后倾斜，也不可弯腰驼背。
		CM_Ppose,   //Ppose标定，所述Ppose：在Apose站立的基础上，左右手同时水平朝正前方平举且平行，掌心向下，手指伸直，大拇指与食指呈45度夹角关系，基于Apose标定后的pose标定，用于手部的标定。
	}_CalibrationMode_;

	typedef enum CALIBRATIONSTATE {
		CS_UnStart = 0,                    //未开始标定状态
		CS_InPose,                         //处于pose标定状态
		CS_Successed,                      //标定成功
		CS_Failed,                         //标定失败
	}_CalibrationState_;

	typedef enum FREQUENCY {
		HZ_60 = 60,
		HZ_72 = 72,
		HZ_80 = 80,
		HZ_96 = 96,
		HZ_120 = 120,
	}_Frequency_;

	typedef enum CONNECTSTATE {
		CONNECTED_NONE = 0,
		CONNECTED_RightGlove,
		CONNECTED_LeftGlove,
		CONNECTED_BothGloves,
	}_ConnectState_;

	typedef enum TREMOR {
		TREMOR_NONE = 0,	//没有震感
		TREMOR_01 = 1,         //震感1
		TREMOR_02 = 2,         //震感2
		TREMOR_03 = 3,         //震感3
		TREMOR_04 = 4,         //震感4
		TREMOR_05 = 5,         //震感5
		TREMOR_06 = 6,         //震感6
		TREMOR_07 = 7,         //震感7
		TREMOR_08 = 8,         //震感8
	}_Tremor_;

	typedef enum GESTURE {
		GESTURE_NONE = 0,  //未知手势
		GESTURE_1,         //食指伸直，其它手指握拢（指向）
		GESTURE_2,         //剪刀手
		GESTURE_3,         //OK
		GESTURE_4,         //四
		GESTURE_5,         //掌（布）
		GESTURE_6,         //六
		GESTURE_7,         //七
		GESTURE_8,         //九
		GESTURE_9,         //手枪
		GESTURE_10,        //暂无
		GESTURE_11,        //比心
		GESTURE_12,        //大拇指、食指、小指伸直，其它手指握拢（爱你）
		GESTURE_13,        //摇滚
		GESTURE_14,        //赞
		GESTURE_15,        //抓（拿）
		GESTURE_16,        //握拳（石头）
		GESTURE_17,        //手枪
		GESTURE_18,        //暂无
		GESTURE_19,        //踩
		GESTURE_20,        //竖中指
		GESTURE_21,        //竖尾指
		GESTURE_22,        //三
	}_Gesture_;



	typedef struct MAGCORRECTRESULT
	{
		bool isFinished = false;  //是否结束
		bool isHaveSucceed = false;  //是否有校准成功的传感器，若为false，则下面的数据无效
		float progress = 0;
		int failedNodesLength = 0;
		_HandNodes_ failedNodes[NODES_HAND] = { HN_Hand };
	}_MagCorrectResult_;

	typedef struct DGMAGCORRECTRESULT
	{
		_GloveMode_ glove;                           //GM_RightGlove 或 GM_LeftGlove
		_MagCorrectResult_ RmagCorrectResult;
		_MagCorrectResult_ LmagCorrectResult;
	}_DGMagCorrectResult_;

	typedef struct CALIBRATIONPROGRESS {
		_CalibrationState_ state;
		float progress;                    //在 state 为 CS_Apose 或 CS_OKpose 时有效
	}_CalibrationProgress_;

	typedef struct GLOVEMOCAPDATA {
		_GloveMode_ glove;                           //GM_RightGlove 或 GM_LeftGlove
		bool isUpdate = 0;                               //true表示设备数据已更新
		int frameIndex = -1;                              //当前帧序号
		float devicePower = 0;                           //设备电量[0, 1]
		int frequency = -1/*HZ*/;                         //设备数据传输频率,单位HZ
		_SensorState_ sensorState[NODES_HAND];       //各节点传感器工作状态（按_HandNodes_序号排列）
		float gyr[NODES_HAND][3] = { 0 }/*xyz-m2/s*/;        //各节点角速度，单位m^2/s
		float acc[NODES_HAND][3] = { 0 }/*xyz-m2/s*/;        //各节点去重力加速度后加速度，单位m^2/s
		float velocity[NODES_HAND][3] = { 0 }/*xyz-m/s*/;    //各节点速度，单位m/s
		float position[NODES_HAND][3] = { 0 }/*xyz-m*/;      //各节点坐标（按序号排列）,单位m
		float quaternion[NODES_HAND][4] = { 0 }/*wxyz*/;     //各节点四元数（按序号排列）
	}_GloveMocapData_;

	//增加虚拟点
	typedef struct GLOVEMOCAPDATA_WITH_VIRTUAL {
		_GloveMode_ glove;                           //GM_RightGlove 或 GM_LeftGlove
		bool isUpdate = 0;                               //true表示设备数据已更新
		int frameIndex = -1;                              //当前帧序号
		float devicePower = 0;                           //设备电量[0, 1]
		int frequency = -1/*HZ*/;                         //设备数据传输频率,单位HZ
		_SensorState_ sensorState[NODES_HAND];       //各节点传感器工作状态（按_HandNodes_序号排列）
		float gyr[NODES_HAND][3] = { 0 }/*xyz-m2/s*/;        //各节点角速度，单位m^2/s
		float acc[NODES_HAND][3] = { 0 }/*xyz-m2/s*/;        //各节点去重力加速度后加速度，单位m^2/s
		float velocity[NODES_HAND][3] = { 0 }/*xyz-m/s*/;    //各节点速度，单位m/s
		float position[NODES_HAND][3] = { 0 }/*xyz-m*/;      //各节点坐标（按序号排列）,单位m
		float quaternion[NODES_HAND][4] = { 0 }/*wxyz*/;     //各节点四元数（按序号排列）

		float positionVirtual[PC_FINGERS_VIRTUAL][3] = { 0 };	//指尖坐标

	}_GloveMocapDataWithVirtual_;


	//
	typedef struct VERSION
	{
		unsigned char Project_Name[26] = { 0 };
		unsigned char Author_Organization[128] = { 0 };
		unsigned char Author_Domain[26] = { 0 };
		unsigned char Author_Maintainer[26] = { 0 };
		unsigned char Version[26] = { 0 };
		unsigned char Version_Major;
		unsigned char Version_Minor;
		unsigned char Version_Patch;
	}_Version_;

	typedef void(*GLOVEMOCAPDATA_CALLBACK)(_GloveMocapData_ gmd_R, _GloveMocapData_ gmd_L); //定义函数回调指针

	typedef void(*GLOVEMOCAPDATA_VIRTUAL_CALLBACK)(_GloveMocapDataWithVirtual_ gmd_R, _GloveMocapDataWithVirtual_ gmd_L);

	typedef void(*GLOVEBREAK_CALLBACK)(_GloveMode_ glove);

	typedef void(*GLOVEMOCAPDATA_CALLBACK_Mult)(_GloveMocapData_ gmd_R, _GloveMocapData_ gmd_L, int index); //定义函数回调指针

	typedef void(*GLOVEMOCAPDATA_VIRTUAL_CALLBACK_Mult)(_GloveMocapDataWithVirtual_ gmd_R, _GloveMocapDataWithVirtual_ gmd_L, int index);

	typedef void(*GLOVEBREAK_CALLBACK_Mult)(_GloveMode_ glove, int index);


}//end namespace
#endif
