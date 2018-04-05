/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
#ifndef _SDK_EXAMPLE_BD_H_
#define _SDK_EXAMPLE_BD_H_

#include <ihm.h>
#include <DecoderManager.h>
#include <libARCommands/ARCommands.h>
#include <FLandFaceDetection.h>

typedef struct
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
} BD_PCMD_t;

typedef struct _ARDrone3CameraData_t_
{
    int tilt;
    int pan;
} BD_Cam_t;

typedef struct _ARDrone3Position_t_
{
    double latitude;
    double longitude;
    double altitude;
} BD_Position_t;

typedef struct _ARDrone3Speed_t_
{
    float speedX;
    float speedY;
    float speedZ;
    long long int timestamp_usec;
} BD_Speed_t;

typedef struct _ARDrone3Attitude_t_
{
    float roll;
    float pitch;
    float yaw;
    long long int timestamp_usec;
} BD_Attitude_t;

typedef struct READER_THREAD_DATA_t READER_THREAD_DATA_t;

typedef struct RawFrame_t RawFrame_t;
typedef struct
{
    ARNETWORKAL_Manager_t *alManager;
    ARNETWORK_Manager_t *netManager;
    ARSTREAM_Reader_t *streamReader;
    ARSAL_Thread_t looperThread;
    ARSAL_Thread_t rxThread;
    ARSAL_Thread_t txThread;
    ARSAL_Thread_t videoTxThread;
    ARSAL_Thread_t videoRxThread;
    int d2cPort;
    int c2dPort;
    int arstreamFragSize;
    int arstreamFragNb;
    int arstreamAckDelay;
    uint8_t *videoFrame;
    uint32_t videoFrameSize;
    
    BD_PCMD_t dataPCMD;
    BD_Cam_t dataCam;
    BD_Position_t dataPosition;
    BD_Speed_t dataSpeed;
    BD_Attitude_t dataAttitude;
    
    double altitude;
    
    ARCODECS_Manager_t *decoder;
    int decodingCanceled;
    ARSAL_Thread_t decodingThread;
    
    int hasReceivedFirstIFrame;
    RawFrame_t **freeRawFramePool;
    int rawFramePoolCapacity;
    int lastRawFrameFreeIdx;
    RawFrame_t **rawFrameFifo;
    int fifoReadIdx;
    int fifoWriteIdx;
    
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState;
    eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName;    
    
    FILE *video_out;
    FILE *video_outRGB;
    FILE *IMU_out;
    FILE *Quaternions_out;
    
    //write synced IMU data to file
    bool imuSpeed;
    bool imuAngle;
            
    ARSAL_Mutex_t mutex;
    
    ARSAL_Thread_t *readerThreads;
    READER_THREAD_DATA_t *readerThreadsData;
    int run;
    
    IHM_t *ihm;
} BD_MANAGER_t;

struct READER_THREAD_DATA_t
{
    BD_MANAGER_t *deviceManager;
    int readerBufferId;
};

/** Connection part **/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/** Network part **/
int startNetwork (BD_MANAGER_t *deviceManager);
void stopNetwork (BD_MANAGER_t *deviceManager);
void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);

/** Video part **/
int startVideo (BD_MANAGER_t *deviceManager);
void stopVideo (BD_MANAGER_t *deviceManager);
uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom);

/** decoding part **/
int startDecoder (BD_MANAGER_t *deviceManager);
void stopDecoder (BD_MANAGER_t *deviceManager);
int getNextDataCallback(uint8_t **data, void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);

/** Commands part **/
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
int sendPCMD(BD_MANAGER_t *deviceManager);
int sendCameraOrientation(BD_MANAGER_t *deviceManager);
int sendDate(BD_MANAGER_t *deviceManager);
int sendAllStates(BD_MANAGER_t *deviceManager);
int sendAllSettings(BD_MANAGER_t *deviceManager);
int sendFlatTrim(BD_MANAGER_t *deviceManager);
int sendTakeoff(BD_MANAGER_t *deviceManager);
int sendLanding(BD_MANAGER_t *deviceManager);
int sendEmergency(BD_MANAGER_t *deviceManager);
int sendBeginStream(BD_MANAGER_t *deviceManager);

/** Commands callback part **/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager);
void unregisterARCommandsCallbacks();
void batteryStateChangedCallback (uint8_t percent, void *custom);
void sensorsStatesListChangedCallback(eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName, uint8_t state, void *custom);
void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom);
void attitudeChangedCallback(float roll, float pitch, float yaw, void *custom);
void speedChangedCallback(float speedX, float speedY, float speedZ, void *custom);
void pilotingStatePositionChangedCallback(double latitude, double longitude, double altitude, void *custom);
void altitudeChangedCallback(double altitude, void *custom);
void gpsStateChangedCallback(eARCOMMANDS_ARDRONE3_GPSSETTINGSSTATE_GPSUPDATESTATECHANGED_STATE state, void *custom);
void gpsFixStateChangedCallback(uint8_t fixed, void *custom);
void gpsNumberOfSatelliteChangedCallback(uint8_t numberOfSatellite, void *custom);

/** IHM callbacks **/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData);
int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);

/* Search and Detect Faces */
void Search4Face(BD_MANAGER_t *deviceManager);
void TrackFace(CvPoint3D32f faceCenter, BD_MANAGER_t *deviceManager);

char *concat(char *s1, char *s2);

int framenumber = 0;

//#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
//
//// RGB -> YUV
//#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
//#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
//#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)
//
//// YUV -> RGB
//#define C(Y) ( (Y) - 16  )
//#define D(U) ( (U) - 128 )
//#define E(V) ( (V) - 128 )
//
//#define YUV2R(Y, U, V) CLIP(( 298 * C(Y)              + 409 * E(V) + 128) >> 8)
//#define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
//#define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U)              + 128) >> 8)
//
//// RGB -> YCbCr
//#define CRGB2Y(R, G, B) CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16)
//#define CRGB2Cb(R, G, B) CLIP((36962 * (B - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)
//#define CRGB2Cr(R, G, B) CLIP((46727 * (R - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)
//
//// YCbCr -> RGB
//#define CYCbCr2R(Y, Cb, Cr) CLIP( Y + ( 91881 * Cr >> 16 ) - 179 )
//#define CYCbCr2G(Y, Cb, Cr) CLIP( Y - (( 22544 * Cb + 46793 * Cr ) >> 16) + 135)
//#define CYCbCr2B(Y, Cb, Cr) CLIP( Y + (116129 * Cb >> 16 ) - 226 )

FILE *frameProcessing;



#endif /* _SDK_EXAMPLE_BD_H_ */
