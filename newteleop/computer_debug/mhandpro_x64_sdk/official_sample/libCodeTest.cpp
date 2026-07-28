#include <dlfcn.h>
#include <string.h>
#include <thread>
#include <unistd.h>
#include "libCodeTest.h"

libCodeTest::libCodeTest(const std::string& libPath)
{
    // load share library
    handle = dlopen(libPath.c_str(), RTLD_LAZY);    // library path
    if (!handle) 
    {
        std::cerr << "Failed to load library: " << dlerror() << std::endl;
        return;
    }

    // close all error when opened the library
    dlerror();

    // get function pointer
    _GetVersionInfo = (GetVersionInfo) dlsym(handle,"GetVersionInfo");
    _SetGloveDataCallBackFunc = (SetGloveDataCallBackFunc) dlsym(handle,"SetGloveDataCallBackFunc");

    //指尖
    _SetGloveDataWithVirtualCallBackFunc = (SetGloveDataWithVirtualCallBackFunc) dlsym(handle,"SetGloveDataWithVirtualCallBackFunc");

    //PLUS VIRTUAL
    _SetGloveDataWithVirtualCallBackFunc_multi = (SetGloveDataWithVirtualCallBackFunc_multi) dlsym(handle,"SetGloveDataWithVirtualCallBackFunc_multi");

    _SetGloveBreakCallBackFunc = (SetGloveBreakCallBackFunc) dlsym(handle,"SetGloveBreakCallBackFunc");

    _SetTremor = (SetTremor) dlsym(handle,"SetTremor");

    _Initial = (Initial) dlsym(handle,"Initial");

    _Initial_multi = (Initial_multi) dlsym(handle,"Initial_multi");

    _Connect = (Connect) dlsym(handle,"Connect");

    _Connect_multi = (Connect_multi) dlsym(handle,"Connect_multi");

    _SetHandDimension = (SetHandDimension) dlsym(handle,"SetHandDimension");

    _DisConnect = (DisConnect) dlsym(handle,"DisConnect");
    _SetModelNodesPositionInInitialPose = (SetModelNodesPositionInInitialPose) dlsym(handle,"SetModelNodesPositionInInitialPose");
    _StartCalibration = (StartCalibration) dlsym(handle,"StartCalibration");
    _RecvMocapData = (RecvMocapData) dlsym(handle,"RecvMocapData");
    _GetCalibrationProgress = (GetCalibrationProgress) dlsym(handle,"GetCalibrationProgress");
    _GetGesture = (GetGesture) dlsym(handle,"GetGesture");

    startMagCorrect = (StartMagCorrect) dlsym(handle,"StartMagCorrect");
    cancelMagCorrect = (CancelMagCorrect) dlsym(handle,"CancelMagCorrect");
    endMagCorrect = (EndMagCorrect) dlsym(handle,"EndMagCorrect");
    getMagCorrectResult = (GetMagCorrectResult) dlsym(handle,"GetMagCorrectResult");
    getDGMagCorrectResult = (GetDGMagCorrectResult) dlsym(handle,"GetDGMagCorrectResult");

    const char* error = dlerror();
	
    if (error) 
    {
        std::cerr << "Failed to find the function: " << error << std::endl;
        dlclose(handle);
		return;
    }

#if 1
	// initial test
	if (_Initial)
    {
        std::cout << "SDK initial..." << std::endl;
    }
    // Initial Gloves
    _Initial(WorldSpace, InitialNodesPosition_r, InitialNodesPosition_l);
#endif

#if 0
    if (_Initial_multi)
    {
        std::cout << "SDK initial MULT..." << std::endl;
    }
    // Initial Gloves
    _Initial_multi(WorldSpace, InitialNodesPosition_r, InitialNodesPosition_l, 0);
#endif

	//set Break Callback Function
    if (_SetGloveBreakCallBackFunc)
    {
        std::cout << "SetGloveBreakCallBackFunc exist!" << std::endl;
    }
    _SetGloveBreakCallBackFunc(GloveBreak);

    // close library
    // dlclose(handle);
}

libCodeTest::~libCodeTest()
{
	if (handle) 
	{
		// close library
		dlclose(handle);
	}
}

//Glove Break Callback
void libCodeTest::GloveBreak(_GloveMode_ glove)
{
    if (glove == GM_RightGlove) 
    {
        if (connStatus == CONNECTED_RightGlove) 
        {
            connStatus = CONNECTED_NONE;
            std::cout << "====== The both gloves disconnected improperly ======" << std::endl;
        }
        else if (connStatus == CONNECTED_BothGloves) 
        {
            connStatus = CONNECTED_LeftGlove;
            std::cout << "====== The right gloves disconnected improperly======" << std::endl;
        }
        std::cout << "====== The right gloves disconnected improperly======" << std::endl;
    }
    else if (glove == GM_LeftGlove) 
    {
        if (connStatus == CONNECTED_LeftGlove) 
        {
            connStatus = CONNECTED_NONE;
            std::cout << "====== The both gloves disconnected improperly ======" << std::endl;
        }
        else if (connStatus == CONNECTED_BothGloves) 
        {
            connStatus = CONNECTED_RightGlove;
            std::cout << "====== The left gloves disconnected improperly======" << std::endl;
        }
        std::cout << "====== The left gloves disconnected improperly======" << std::endl;
    }
    else if (glove == GM_BothGloves) 
    {
        connStatus = CONNECTED_NONE;
        std::cout << "====== The both gloves is improperly disconnected ======" << std::endl;
    }
}

void libCodeTest::debugCalibrationProgress()
{
	while (true)
	{
		usleep(10);
		_CalibrationProgress_ cp = _GetCalibrationProgress();
		if (cp.state != CS_UnStart) {
			printf("cp: state: %d   progress: %.2f\n", cp.state, cp.progress);
			if (cp.state == CS_Successed) { break; }
		}
	}
}


// callback function of getting all mocapdata
void libCodeTest::GetMocapData(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l)  // Do not call functions in an interface inside a callback
{
    if(connStatus == CONNECTED_NONE)
    {
        // std::cout << " GetMocapData CONNECTED_NONE" << std::endl;
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ frameIndex: " + to_string(gmd_r.frameIndex) + "\n";
        rhandInfo.frameIndex = gmd_r.frameIndex;
        str += "++++++++++++ devicePower: " + to_string(gmd_r.devicePower) + "\n";
        rhandInfo.devicePower = gmd_r.devicePower;
        str += "++++++++++++ frequency: " + to_string(gmd_r.frequency) + "\n";
        rhandInfo.frequency = gmd_r.frequency;
        str += "++++++++++++ quaternion:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "++++++++++++         ";
            for (int i = 0; i < 4; i++) { str += to_string(gmd_r.quaternion[ii][i]) + " "; rhandInfo.quaternion[ii][i] = gmd_r.quaternion[ii][i];}
            str += "\n";
        }
        str += "++++++++++++ RightHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_r.position[ii][i]) + " "; rhandInfo.position[ii][i] = gmd_r.position[ii][i];}
            str += "\n";
        }
    }
    if (gmd_l.isUpdate&&lUpdataFlag) {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ frameIndex: " + to_string(gmd_l.frameIndex) + "\n";
        lhandInfo.frameIndex = gmd_l.frameIndex;
        str += "------------ devicePower: " + to_string(gmd_l.devicePower) + "\n";
        lhandInfo.devicePower = gmd_l.devicePower;
        str += "------------ frequency: " + to_string(gmd_l.frequency) + "\n";
        lhandInfo.frequency = gmd_l.frequency;
        str += "------------ quaternion:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "------------         ";
            for (int i = 0; i < 4; i++) { str += to_string(gmd_l.quaternion[ii][i]) + " "; lhandInfo.quaternion[ii][i] = gmd_l.quaternion[ii][i];}
            str += "\n";
        }
        str += "------------ LeftHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_l.position[ii][i]) + " "; lhandInfo.position[ii][i] = gmd_l.position[ii][i];}
            str += "\n";
        }
    }
    std::cout<<str<<std::endl;
}

// callback function of getting fps
void libCodeTest::GetFPS(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l)  // Do not call functions in an interface inside a callback
{
    if(connStatus == CONNECTED_NONE)
    {
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ frameIndex: " + to_string(gmd_r.frameIndex) + "\n";
    }
    if (gmd_l.isUpdate&&lUpdataFlag) 
    {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ frameIndex: " + to_string(gmd_l.frameIndex) + "\n";
    }
    std::cout<<str<<std::endl;
}

// callback function of getting quaternion
void libCodeTest::GetQuaternion(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l)  //Do not call functions in an interface inside a callback
{
    if(connStatus == CONNECTED_NONE)
    {
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ quaternion:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "++++++++++++         ";
            for (int i = 0; i < 4; i++) { str += to_string(gmd_r.quaternion[ii][i]) + " "; rhandInfo.quaternion[ii][i] = gmd_r.quaternion[ii][i];}
            str += "\n";
        }
    }
    if (gmd_l.isUpdate&&lUpdataFlag) 
    {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ quaternion:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "------------         ";
            for (int i = 0; i < 4; i++) { str += to_string(gmd_l.quaternion[ii][i]) + " "; lhandInfo.quaternion[ii][i] = gmd_l.quaternion[ii][i];}
            str += "\n";
        }
    }
    std::cout<<str<<std::endl;
}

// callback function of getting postition
void libCodeTest::GetPostition(_GloveMocapData_ gmd_r, _GloveMocapData_ gmd_l)  // Do not call functions in an interface inside a callback
{
    if(connStatus == CONNECTED_NONE)
    {
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ RightHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_r.position[ii][i]) + " "; rhandInfo.position[ii][i] = gmd_r.position[ii][i];}
            str += "\n";
        }
    }
    if (gmd_l.isUpdate&&lUpdataFlag) {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ LeftHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_l.position[ii][i]) + " "; lhandInfo.position[ii][i] = gmd_l.position[ii][i];}
            str += "\n";
        }
    }
    std::cout<<str<<std::endl;  
}

//添加指尖坐标
void libCodeTest::GetPostitionVirtual(_GloveMocapDataWithVirtual_ gmd_r, _GloveMocapDataWithVirtual_ gmd_l)  
{
    if(connStatus == CONNECTED_NONE)
    {
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ RightHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_r.position[ii][i]) + " "; rhandInfo.position[ii][i] = gmd_r.position[ii][i];}
            str += "\n";
        }
    }
    if (gmd_l.isUpdate&&lUpdataFlag) {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ LeftHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_l.position[ii][i]) + " "; lhandInfo.position[ii][i] = gmd_l.position[ii][i];}
            str += "\n";
        }
    }
    
#if 1
	// 右手数据
	if (gmd_r.isUpdate) {
		cout << "========================================" << endl;
		cout << "right fingertip (frame index" << gmd_r.frameIndex << ")" << endl;
		cout << "========================================" << endl;

		for (int i = 0; i < PC_FINGERS_VIRTUAL; i++) {
			printf("%.3f %.3f %.3f\n",
				gmd_r.positionVirtual[i][0],
				gmd_r.positionVirtual[i][1],
				gmd_r.positionVirtual[i][2]);
		}
	}

	// 左手数据
	if (gmd_l.isUpdate) {
		cout << "\n========================================" << endl;
		cout << "left fingertip (frame index" << gmd_l.frameIndex << ")" << endl;
		cout << "========================================" << endl;

		for (int i = 0; i < PC_FINGERS_VIRTUAL; i++) {
			printf("%.3f %.3f %.3f\n",
				gmd_l.positionVirtual[i][0],
				gmd_l.positionVirtual[i][1],
				gmd_l.positionVirtual[i][2]);
		}
    }
#endif

    std::cout<<str<<std::endl;
}

void libCodeTest::GetPostitionVirtual_Mult(_GloveMocapDataWithVirtual_ gmd_r, _GloveMocapDataWithVirtual_ gmd_l,int index)  
{
    if(connStatus == CONNECTED_NONE)
    {
        return;
    }
    string str = "";
    bool rUpdataFlag = false;
    bool lUpdataFlag = false;

    rUpdataFlag = (connStatus == CONNECTED_RightGlove)?true:false;
    rUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:rUpdataFlag;
    lUpdataFlag = (connStatus == CONNECTED_LeftGlove)?true:false;
    lUpdataFlag = (connStatus == CONNECTED_BothGloves)?true:lUpdataFlag;
    
    
    if (gmd_r.isUpdate&&rUpdataFlag) 
    {
        rhandInfo.updataFlag = true;
        str += "++++++ RightHand Data ++++++\n";
        str += "++++++++++++ RightHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_r.position[ii][i]) + " "; rhandInfo.position[ii][i] = gmd_r.position[ii][i];}
            str += "\n";
        }
    }
    if (gmd_l.isUpdate&&lUpdataFlag) {
        lhandInfo.updataFlag = true;
        str += "------ LeftHand Data ------\n";
        str += "------------ LeftHand position:\n";
        for (int ii = 0; ii < NODES_HAND; ii++)
        {
            str += "                   ";
            for (int i = 0; i < 3; i++) { str += to_string(gmd_l.position[ii][i]) + " "; lhandInfo.position[ii][i] = gmd_l.position[ii][i];}
            str += "\n";
        }
    }
    

	// 右手数据
	if (gmd_r.isUpdate) {
		cout << "========================================" << endl;
		cout << "right fingertip (frame index" << gmd_r.frameIndex << ")" << endl;
		cout << "========================================" << endl;

		for (int i = 0; i < PC_FINGERS_VIRTUAL; i++) {
			printf("%.3f %.3f %.3f\n",
				gmd_r.positionVirtual[i][0],
				gmd_r.positionVirtual[i][1],
				gmd_r.positionVirtual[i][2]);
		}
	}

	// 左手数据
	if (gmd_l.isUpdate) {
		cout << "\n========================================" << endl;
		cout << "left fingertip (frame index" << gmd_l.frameIndex << ")" << endl;
		cout << "========================================" << endl;

		for (int i = 0; i < PC_FINGERS_VIRTUAL; i++) {
			printf("%.3f %.3f %.3f\n",
				gmd_l.positionVirtual[i][0],
				gmd_l.positionVirtual[i][1],
				gmd_l.positionVirtual[i][2]);
		}
	}

    std::cout<<str<<std::endl;
}

// show sdk version
void libCodeTest::showVersionAction()
{
	if (_GetVersionInfo)
    {
        std::cout << "...show SDK Version..." << std::endl;
        _GetVersionInfo(version);
        cout << "********************************************************" <<endl;
        cout << "Project Name : "<< (const char*)(version->Project_Name) << endl;
        cout << "Author Organization : " << (const char*)(version->Author_Organization) << endl;
        cout << "Author_Domainr : " << (const char*)version->Author_Domain << endl;
        cout << "Author_Maintainer : " << (const char*)version->Author_Maintainer << endl;
        cout << "Version : " << (const char*)version->Version << endl;
        // cout << "Version_Major : " << (version->Version_Major) << endl;
        // cout << "Version_Minor : " << version->Version_Minor << endl;
        // cout << "Version_Patch : " << version->Version_Patch << endl;
        cout << "********************************************************\n" << endl;
    }
    else 
    {
        std::cout << "...SDK unload..." << std::endl;
        return;
    }
}

// connect device
void libCodeTest::connectAction()
{
    connStatus = (int)_Connect();

    //connStatus = (int)_Connect_multi(0);

    if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== Connection Fail ======" << endl;
        cout << "Please check the serial port permission or hardware connection !" << endl;
        cout << " " << endl;
    }
    else if (connStatus == CONNECTED_RightGlove) 
	{
        
        cout << "****** The right glove is connected ******" << endl;
    }
    else if (connStatus == CONNECTED_LeftGlove) 
	{
        cout << "****** The left glove is connected ******" << endl;
    }
    else if (connStatus == CONNECTED_BothGloves) 
	{
        cout << "****** The both gloves is connected ******" << endl;
    }

    _SetHandDimension(true);

    _SetTremor(TREMOR_06,TREMOR_06);
}

// diconnect device
void libCodeTest::disconnectAction()
{
	if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== DisConnect fail ======" << endl;
        cout << "====== Current connect status : DisConnect ======" << endl;
    }
    else if (connStatus == CONNECTED_RightGlove) 
	{
        _DisConnect();
        cout << "====== RightGlove DisConnecting ======" << endl;
    }
    else if (connStatus == CONNECTED_LeftGlove) 
	{
        _DisConnect();
        cout << "====== LeftGlove DisConnecting ======" << endl;
    }
    else if (connStatus == CONNECTED_BothGloves) 
	{
        _DisConnect();
        cout << "====== BothGloves DisConnecting ======" << endl;
    }
}

// get sdk all MocapData
void libCodeTest::autoGetDataAction()
{
    if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== Current status is disconnect ======" << endl;
        cout << "====== Please connect device first! ======" << endl;
    }
    _SetGloveDataCallBackFunc(GetMocapData);
}

// get sdk fps
void libCodeTest::autoGetFPSAction()
{
    if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== Current status is disconnect ======" << endl;
        cout << "====== Please connect device first! ======" << endl;
    }
    _SetGloveDataCallBackFunc(GetFPS);
}

// get sdk quaternion
void libCodeTest::autoGetQuaternionAction()
{
    if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== Current status is disconnect ======" << endl;
        cout << "====== Please connect device first! ======" << endl;
    }
    _SetGloveDataCallBackFunc(GetQuaternion);
}

void libCodeTest::autoGetPostitionAction()
{
    if (connStatus == CONNECTED_NONE) 
	{
        cout << "====== Current status is disconnect ======" << endl;
        cout << "====== Please connect device first! ======" << endl;
    }

    //_SetGloveDataCallBackFunc(GetPostition);

    //virtual
    _SetGloveDataWithVirtualCallBackFunc(GetPostitionVirtual);

    //plus virtual
    //_SetGloveDataWithVirtualCallBackFunc_multi(GetPostitionVirtual_Mult);
}

void libCodeTest::MagCorrectAction()
{
    /*****************************Magnetic calibration start**************************************************/
    _MagCorrectResult_ magCorrectresultR, magCorrectresultL;
    _DGMagCorrectResult_ dgMagCorrectResult;
    if (MagCorrectState)
    {
        //calibration start
        cout << "start magnetic calibration after 3s " << endl;
        sleep(3);

        startMagCorrect();
        cout << "start" << endl;

        // Collect data, move away from the magnetic field, calibrate in a box or do a gymnastics calibration
        while (MagCorrectTime--)    
        {
            cout << MagCorrectTime << "s left in magnetic calibration" << endl;
            sleep(1);
        }

        // endMagCorrect();     

        int magret = getDGMagCorrectResult(&dgMagCorrectResult);
        float progress = 0;
        while (!magret)
        {
            magret = getDGMagCorrectResult(&dgMagCorrectResult);
            if (dgMagCorrectResult.glove == GM_BothGloves)
            {
                progress = dgMagCorrectResult.LmagCorrectResult.progress + dgMagCorrectResult.RmagCorrectResult.progress;
            }
            else if (dgMagCorrectResult.glove == GM_RightGlove)
            {
                progress = dgMagCorrectResult.RmagCorrectResult.progress;
            }
            else if (dgMagCorrectResult.glove == GM_LeftGlove)
            {
                progress = dgMagCorrectResult.LmagCorrectResult.progress ;
            }
            cout << "progress	is	" << progress << endl;
        }

        // Determine whether the sensor is successfully calibrated according to the return result

        if (dgMagCorrectResult.glove == GM_BothGloves)    //  At least one success
        {
            cout << "Double-handed magnetic calibration complete" << endl;
            if (dgMagCorrectResult.RmagCorrectResult.failedNodesLength > 0)
            {

                for (int i = 0; i < dgMagCorrectResult.RmagCorrectResult.failedNodesLength; i++)
                {
                    cout << "Right hand calibration failure sensors are" << dgMagCorrectResult.RmagCorrectResult.failedNodes[i] << endl;

                }

                for (int i = 0; i < dgMagCorrectResult.LmagCorrectResult.failedNodesLength; i++)
                {
                    cout << "Left hand calibration failure sensors are" << dgMagCorrectResult.LmagCorrectResult.failedNodes[i] << endl;

                }


            }
        }
        else if (dgMagCorrectResult.glove == GM_RightGlove)
        {
            cout << "Right hand magnetic calibration complete" << endl;
            if (dgMagCorrectResult.RmagCorrectResult.failedNodesLength > 0)
            {

                for (int i = 0; i < dgMagCorrectResult.RmagCorrectResult.failedNodesLength; i++)
                {
                    cout << "Right hand calibration failure sensors are" << dgMagCorrectResult.RmagCorrectResult.failedNodes[i] << endl;

                }

            }
        }
        else if (dgMagCorrectResult.glove == GM_LeftGlove)
        {
            cout << "Left hand magnetic calibration complete" << endl;
            if (dgMagCorrectResult.LmagCorrectResult.failedNodesLength > 0)
            {

                for (int i = 0; i < dgMagCorrectResult.LmagCorrectResult.failedNodesLength; i++)
                {
                    cout << "Left hand calibration failure sensors are" << dgMagCorrectResult.LmagCorrectResult.failedNodes[i] << endl;

                }
            }
        }
        else
        {
            cout << "Magnetic calibration failed. Please stay away from magnetic field" << endl;      //None of the magnetic calibrations were successful
        }

    }
    MagCorrectState = false;
    /*****************************magnetic calibration end**************************************************/

}

void libCodeTest::PosCorrectAction()
{   
    // Start Calibration
    _CalibrationMode_ calibrationMode = CM_Ppose;
    float quat_EndCalibration_root[4] = { 0 };
    _StartCalibration(calibrationMode, quat_EndCalibration_root);
    std::thread thr1([this]() { this->debugCalibrationProgress(); });
    // thread thr1(debugCalibrationProgress);
    thr1.join();
}
