#pragma once
#ifndef LIBCODETEST_H
#define LIBCODETEST_H

#include <iostream>
#include "./include/VDMocapSDK_mHandPro_DataType.h"

using namespace std;
using namespace mHandProDevice;

typedef struct
{
    bool updataFlag = false;
    int frameIndex = 0;
    float devicePower = 0;
    int frequency = 0;
    float quaternion[20][4] = {{0}};
    float position[20][3] = {{0}};
}DEVICEINFOSTRUCT;

static DEVICEINFOSTRUCT rhandInfo;
static DEVICEINFOSTRUCT lhandInfo;

static int connStatus;
// bool MagCorrectState = false;    //�Ƿ�������У׼״̬       ��У׼��ɺ���Ҫ�����豸������Ч��Ҳ���Բ��sign out������������
// int MagCorrectTime = 10;		//����		����У׼��30s			���У׼��75s

class libCodeTest
{
public:
	libCodeTest(const std::string& libPath);
	~libCodeTest();


private:
/*---------------------------------------- SDK API----------------------------------------------------*/
    // Get SDK Version information
    typedef void(*GetVersionInfo)(_Version_ *version);
    GetVersionInfo _GetVersionInfo;

    // MocapData Callback
    typedef void(*GLOVEMOCAPDATA_CALLBACK)(_GloveMocapData_ gmd_R, _GloveMocapData_ gmd_L);
    typedef  void(*SetGloveDataCallBackFunc)(GLOVEMOCAPDATA_CALLBACK function);
    SetGloveDataCallBackFunc _SetGloveDataCallBackFunc;

    //Virtual
    typedef void(*GLOVEMOCAPDATA_VIRTUAL_CALLBACK)(_GloveMocapDataWithVirtual_ gmd_R, _GloveMocapDataWithVirtual_ gmd_L);
    typedef  void(*SetGloveDataWithVirtualCallBackFunc)(GLOVEMOCAPDATA_VIRTUAL_CALLBACK function);
    SetGloveDataWithVirtualCallBackFunc _SetGloveDataWithVirtualCallBackFunc;

    //PLUS VIRTUAL
    typedef void(*GLOVEMOCAPDATA_PLUS_VIRTUAL_CALLBACK)(_GloveMocapDataWithVirtual_ gmd_R, _GloveMocapDataWithVirtual_ gmd_L,int index);
    typedef  void(*SetGloveDataWithVirtualCallBackFunc_multi)(GLOVEMOCAPDATA_PLUS_VIRTUAL_CALLBACK function);
    SetGloveDataWithVirtualCallBackFunc_multi _SetGloveDataWithVirtualCallBackFunc_multi;

    // Glove Break Callback
    typedef void(*GLOVEBREAK_CALLBACK)(_GloveMode_ glove);
    typedef  void(*SetGloveBreakCallBackFunc)(GLOVEBREAK_CALLBACK function);
    SetGloveBreakCallBackFunc _SetGloveBreakCallBackFunc;

    // Initial SDK, Must be used first
    typedef void(*Initial)(_WorldSpace_ WorldSpace, float InitialPosition_RHand[NODES_HAND][3], float InitialPosition_LHand[NODES_HAND][3]);
    Initial _Initial;

    //PLUS
    typedef void(*Initial_multi)(_WorldSpace_ WorldSpace, float InitialPosition_RHand[NODES_HAND][3], float InitialPosition_LHand[NODES_HAND][3],int Index);
    Initial_multi _Initial_multi;

    //Connect Gloves, the gloves will be connected to your computer
    typedef _ConnectState_(*Connect)();
    Connect _Connect;

    typedef _ConnectState_(*Connect_multi)(int Index);
    Connect_multi _Connect_multi;

    //DisConnect Gloves
    typedef void(*DisConnect)();
    DisConnect _DisConnect;

    typedef void(*SetHandDimension)(bool Dimension);
    SetHandDimension _SetHandDimension;

    //Get MocapData
    typedef void(*RecvMocapData)(_GloveMocapData_ *data);
    RecvMocapData _RecvMocapData;

    //Get Connect State
    typedef _ConnectState_(*GetConnectState)();

    // information in MocapData
    typedef void(*GetFrequency)(int* fs_r, int* fs_l);
    typedef void(*GetGlovePower)(float* GR_power, float* GL_power);

    // Get current gesture
    typedef void(*GetGesture)(_Gesture_ &gesture_R, _Gesture_ &gesture_L);
    GetGesture _GetGesture;

    typedef void(*GetTremor)(_Tremor_ *tremor_R, _Tremor_ *tremor_L);

    typedef void(*SetTremor)(_Tremor_ tremor_R, _Tremor_ tremor_L);
    SetTremor _SetTremor;



    // set Frequency [60(default) 72 80 96 120]
    typedef void(*SetFrequency)(_Frequency_ frequency);

    // Set the coordinates of each node under the initial Pose of hands model.
    typedef bool(*SetModelNodesPositionInInitialPose)(float InitialPosition_RHand[NODES_HAND][3], float InitialPosition_LHand[NODES_HAND][3]);
    SetModelNodesPositionInInitialPose _SetModelNodesPositionInInitialPose;

    typedef void(*FastCalibration)(_GloveMode_ whichGlove);

    typedef void(*StartCalibration)(_CalibrationMode_ calibrationMode, float quat_EndCalibration_root[4]);
    StartCalibration _StartCalibration;

    typedef void(*CancelCalibration)();

    typedef _CalibrationProgress_(*GetCalibrationProgress)();
    GetCalibrationProgress _GetCalibrationProgress;

    typedef bool(*SetShakeDegreeForCalibration)();

    /*-------------------------------------�����ƺ���		start--------------------------------------------------*/
    //��ʼ������У׼
    typedef bool(*StartMagCorrect)();
    StartMagCorrect startMagCorrect;

    //ȡ��������У׼
    typedef void(*CancelMagCorrect)();
    CancelMagCorrect cancelMagCorrect;

    //����������У׼
    typedef void(*EndMagCorrect)();
    EndMagCorrect endMagCorrect;

    typedef bool(*GetMagCorrectResult)(_MagCorrectResult_* magCorrectResultR, _MagCorrectResult_* magCorrectResultL);
    GetMagCorrectResult getMagCorrectResult;

    typedef bool(*GetDGMagCorrectResult)(_DGMagCorrectResult_* _DGMagCorrectResult_);
    GetDGMagCorrectResult getDGMagCorrectResult;

    /*-------------------------------------�����ƺ���		end--------------------------------------------------*/

    bool MagCorrectState = false;    //�Ƿ�������У׼״̬       ��У׼��ɺ���Ҫ�����豸������Ч��Ҳ���Բ��sign out������������
    int MagCorrectTime = 10;		//����		����У׼��30s			���У׼��75s


    // user use data
    // Use Geo Axis(O-XYZ) Initial Nodes Position to Demo
    float InitialNodesPosition_r[NODES_HAND][3] = {
    #if(USE_MODE == USE_EXIST)
        {0.748, 0, 1.597},
        {0.782, 0.042, 1.6},
        {0.817, 0.077, 1.6},
        {0.842, 0.101, 1.6},
        {0.792, 0.026, 1.604},
        {0.862, 0.04, 1.603},
        {0.912, 0.04, 1.601},
        {0.939, 0.04, 1.599},
        {0.794, 0.01, 1.604},
        {0.864, 0.014, 1.603},
        {0.917, 0.014, 1.6},
        {0.951, 0.014, 1.597},
        {0.793, -0.001, 1.605},
        {0.856, -0.008, 1.604},
        {0.903, -0.008, 1.6},
        {0.935, -0.008, 1.597},
        {0.791, -0.016, 1.604},
        {0.847, -0.031, 1.603},
        {0.884, -0.031, 1.601},
        {0.908, -0.031, 1.6}
    #else
        {0,0,0},
    #endif
    };
    float InitialNodesPosition_l[NODES_HAND][3] = {
    #if(USE_MODE == USE_EXIST)
        {-0.748, 0, 1.597},
        {-0.782, 0.042, 1.6},
        {-0.817, 0.077, 1.6},
        {-0.842, 0.102, 1.6},
        {-0.792, 0.026, 1.604},
        {-0.862, 0.04, 1.603},
        {-0.912, 0.04, 1.601},
        {-0.939, 0.04, 1.599},
        {-0.794, 0.01, 1.604},
        {-0.864, 0.014, 1.603},
        {-0.917, 0.014, 1.6},
        {-0.951, 0.014, 1.597},
        {-0.793, -0.001, 1.605},
        {-0.856, -0.008, 1.604},
        {-0.903, -0.008, 1.601},
        {-0.935, -0.008, 1.599},
        {-0.791, -0.016, 1.604},
        {-0.847, -0.031, 1.603},
        {-0.884, -0.03, 1.601},
        {-0.908, -0.031, 1.6}
    #else
        {0,0,0},
    #endif
    };
    _WorldSpace_ WorldSpace = WS_Geo;
    _Version_ *version = new _Version_;

    GLOVEMOCAPDATA_CALLBACK callbackFunc = nullptr;

    _GloveMocapData_ g_data_r = *new _GloveMocapData_;
    _GloveMocapData_ g_data_l = *new _GloveMocapData_;

    /*-------------------------------------------- user data --------------------------------------------*/
    static void GetMocapData(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l);
    static void GetFPS(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l);
    static void GetQuaternion(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l);
    static void GetPostition(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l);

    static void GetPostitionVirtual(_GloveMocapDataWithVirtual_ gmd_r, _GloveMocapDataWithVirtual_ gmd_l);

    static void GetPostitionVirtual_Mult(_GloveMocapDataWithVirtual_ gmd_r, _GloveMocapDataWithVirtual_ gmd_l,int index);

    static void GloveBreak(_GloveMode_ glove);
    void debugCalibrationProgress();

public:
    void* handle;
    void showVersionAction();
    void connectAction();
    void disconnectAction();
    void autoGetDataAction();
    void autoGetFPSAction();
    void autoGetQuaternionAction();
    void autoGetPostitionAction();
    void MagCorrectAction();
    void PosCorrectAction();

    _ConnectState_ ConnectState;


};

#endif
