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
/**
 * @file BebopDroneDecodeStream.c
 * @brief This file contains sources about basic arsdk example decoding video stream from a BebopDrone with ffmpeg
 * @date 08/01/2015
 */

/**
 * Updated by Hayyan Daoud (hayyan.d@gmail.com)
 * Video received is converted into RGB frames then these frames are processed to detect faces and control the drone to maintain face in center of image
 * @date 11/05/2015
 */

/**
* Updated by Hayyan Daoud (hayyan.d@gmail.com)
* Face detection and controlling the drone to maintain face in center of image is removed as this version intended for SLAMM only
* @date 04/04/2018
*/


/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
#include <sys/time.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "BebopDroneDecodeStream.h"
#include "yuv.h"


/*****************************************
 *
 *             define :
 *
 *****************************************/
#define TAG "BebopDroneReceiveStream"
#define BD_IP_ADDRESS "192.168.42.1"
#define BD_DISCOVERY_PORT 44444
#define BD_C2D_PORT 54321 // should be read from Json
#define BD_D2C_PORT 43210

#define BD_NET_CD_NONACK_ID 10
#define BD_NET_CD_ACK_ID 11
#define BD_NET_CD_EMERGENCY_ID 12
#define BD_NET_CD_VIDEO_ACK_ID 13
#define BD_NET_DC_NAVDATA_ID 127
#define BD_NET_DC_EVENT_ID 126
#define BD_NET_DC_VIDEO_DATA_ID 125

#define BD_NET_DC_VIDEO_FRAG_SIZE 65000
#define BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG 4

#define BD_RAW_FRAME_BUFFER_SIZE 50
#define BD_RAW_FRAME_POOL_SIZE 50

#define ERROR_STR_LENGTH 2048



int frameWidth = 640;
int frameHeight = 368;

//To get RGB frames before writing to v4l2 device
struct SwsContext *img_convert_ctx = NULL;

bool bUseRGB = true;
CvVideoWriter* videoWriter;


//Time stamp for IMU readings
struct timeval timer_usec;
const long EPSILON = 189000; //usec, to relate attitude and speed measurement - Based on experiments on Bebop the interval of new measurements is 200ms
//long long int timestamp_usec; //moved to dataSpeed and dataAttitude structs

int getNextDataCallback(uint8_t **data, void *customData);
void* Decode_RunDataThread(void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);
/**
 * @brief Component of a frame.
 */
struct RawFrame_t
{
    uint8_t *data; /**< data buffer*/
    uint32_t size; /**< size of the buffer */
    uint8_t isIframe;
};

/* YUV reading in opencv and converting to rgb*/
    struct YUV_Capture cap;
    enum YUV_ReturnValue retr;
    IplImage *rgb;

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

static ARNETWORK_IOBufferParam_t c2dParams[] = {
    /* Non-acknowledged commands. */
    {
        .ID = BD_NET_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = BD_NET_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = BD_NET_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Video ACK (Initialized later) */
    {
        .ID = BD_NET_CD_VIDEO_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        .sendingWaitTimeMs = 0,
        .ackTimeoutMs = 0,
        .numberOfRetry = 0,
        .numberOfCell = 0,
        .dataCopyMaxSize = 0,
        .isOverwriting = 0,
    },
};
static const size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

static ARNETWORK_IOBufferParam_t d2cParams[] = {
    {
        .ID = BD_NET_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = BD_NET_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Video data (Initialized later) */
    {
        .ID = BD_NET_DC_VIDEO_DATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        .sendingWaitTimeMs = 0,
        .ackTimeoutMs = 0,
        .numberOfRetry = 0,
        .numberOfCell = 0,
        .dataCopyMaxSize = 0,
        .isOverwriting = 0,
    },
};
static const size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

static int commandBufferIds[] = {
    BD_NET_DC_NAVDATA_ID,
    BD_NET_DC_EVENT_ID,
};
static const size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

int gIHMRun = 0;
char gErrorStr[ERROR_STR_LENGTH];



// reader thread
void *readerRun (void* data)
{
    BD_MANAGER_t *deviceManager = NULL;
    int bufferId = 0;
    int failed = 0;
    
    // Allocate some space for incoming data.
    const size_t maxLength = 128 * 1024;
    void *readData = malloc (maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        // get thread data.
        if (data != NULL)
        {
            bufferId = ((READER_THREAD_DATA_t *)data)->readerBufferId;
            deviceManager = ((READER_THREAD_DATA_t *)data)->deviceManager;
            
            if (deviceManager == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        while (deviceManager->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;
            
            // read data
            netError = ARNETWORK_Manager_ReadDataWithTimeout (deviceManager->netManager, bufferId, readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNETWORK_Manager_ReadDataWithTimeout () failed : %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }
            
            if (!skip)
            {
                // Forward data to the CommandsManager
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeBuffer ((uint8_t *)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer ((uint8_t *)readData, length, msg, sizeof(msg));
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARCOMMANDS_Decoder_DecodeBuffer () failed : %d %s", cmdError, msg);
                }
            }
        }
    }
    
    if (readData != NULL)
    {
        free (readData);
        readData = NULL;
    }
    
    return NULL;
}

// decoder thread
void* Decode_RunDataThread(void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARCODECS_ERROR error;
    ARCODECS_Manager_Frame_t *decodedFrame = NULL;
    while (!deviceManager->decodingCanceled)
    {
        ARSAL_Mutex_Lock (&(deviceManager->mutex));
        RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
        ARSAL_Mutex_Unlock (&(deviceManager->mutex));
        
        if (rawFrame != NULL && rawFrame->isIframe == 1)
        {
            deviceManager->hasReceivedFirstIFrame = 1;
        }
        
        if (rawFrame != NULL)
        {
            if (deviceManager->hasReceivedFirstIFrame)
            {
                decodedFrame = ARCODECS_Manager_Decode(deviceManager->decoder, &error);
            }
            
            if (decodedFrame != NULL)
            {
                //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Frame has been decoded ");
                
                // this part is only needed to copy the YUV frame into the file
//                frameWidth = decodedFrame->width;
//                frameHeight = decodedFrame->height;
                
                int pic_size = avpicture_get_size(PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);
		uint8_t *decodedOut = malloc(pic_size);
		AVFrame *avFrame = avcodec_alloc_frame();
		
		//When RGB is used
		uint8_t *decodedOutRGB;
		int pic_sizeRGB;

                
                // in case your install of FFMpeg supports av_image_copy_to_buffer, you can replace the AVFrame creation and initialisation by this block
                /*uint8_t *src_data[4];
                src_data[0] = decodedFrame->componentArray[0].data;
                src_data[1] = decodedFrame->componentArray[1].data;
                src_data[2] = decodedFrame->componentArray[2].data;
                src_data[3] = NULL;
                
                int src_linesize[4];
                src_linesize[0] = decodedFrame->componentArray[0].lineSize;
                src_linesize[1] = decodedFrame->componentArray[1].lineSize;
                src_linesize[2] = decodedFrame->componentArray[2].lineSize;
                src_linesize[3] = 0;
                
                av_image_copy_to_buffer(decodedOut, pic_size,
                                        (const uint8_t *const *)src_data, src_linesize,
                                        PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, 1);*/
                
               
                
                if (avFrame != NULL)
                {
                    avFrame->width = decodedFrame->width;
                    avFrame->height = decodedFrame->height;
                    avFrame->format = AV_PIX_FMT_YUV420P;
                                     

                    avpicture_fill((AVPicture*)avFrame, NULL, PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);
                    avFrame->linesize[0] = decodedFrame->componentArray[0].lineSize;
                    avFrame->linesize[1] = decodedFrame->componentArray[1].lineSize;
                    avFrame->linesize[2] = decodedFrame->componentArray[2].lineSize;
                    
                    avFrame->data[0] = decodedFrame->componentArray[0].data;
                    avFrame->data[1] = decodedFrame->componentArray[1].data;
                    avFrame->data[2] = decodedFrame->componentArray[2].data;
                    
                    avpicture_layout((AVPicture*)avFrame, PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, decodedOut, pic_size);                                    
                    
                    if(bUseRGB)
                    {
			pic_sizeRGB = avpicture_get_size(PIX_FMT_RGB24, decodedFrame->width, decodedFrame->height);                
			decodedOutRGB = malloc(pic_sizeRGB);
			AVFrame *avFrameRGB = avcodec_alloc_frame();
			if(avFrameRGB != NULL)
			{
				avFrameRGB->width = decodedFrame->width;
		            	avFrameRGB->height = decodedFrame->height;
		            	avFrameRGB->format = AV_PIX_FMT_RGB24;
		                
		                avpicture_fill((AVPicture *)avFrameRGB, decodedOutRGB, PIX_FMT_RGB24, decodedFrame->width, decodedFrame->height);
		                                        
		                img_convert_ctx = sws_getContext(decodedFrame->width, decodedFrame->height, PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);                        
		                sws_scale(img_convert_ctx, avFrame->data, avFrame->linesize, 0, decodedFrame->height, avFrameRGB->data, avFrameRGB->linesize);
		                
		                avpicture_layout((AVPicture*)avFrameRGB, PIX_FMT_RGB24, decodedFrame->width, decodedFrame->height, decodedOutRGB, pic_sizeRGB); 
                        }
			avcodec_free_frame(&avFrameRGB);
                    }
                    
                    
                    avcodec_free_frame(&avFrame);
                    
                    
                
                }
                
                if (decodedOut != NULL)
                {
                    //writeAscii(decodedOut, pic_size);
                    fwrite(decodedOut, pic_size, 1, deviceManager->video_out); 
                    if(bUseRGB)
                        if(decodedOutRGB != NULL)
                            fwrite(decodedOutRGB, pic_sizeRGB, 1, deviceManager->video_outRGB);
                }
                
                free (decodedOut);
                decodedOut = NULL;
                
                if(bUseRGB)
                {
                    free (decodedOutRGB);
                    decodedOutRGB = NULL;
                }
                
                
            }
            
            ARSAL_Mutex_Lock (&(deviceManager->mutex));
            putRawFrameBackToPool(deviceManager, deviceManager->fifoReadIdx);
            deviceManager->fifoReadIdx = (deviceManager->fifoReadIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
            ARSAL_Mutex_Unlock (&(deviceManager->mutex));
        }
    }
    
    return (void*)0;
}


// looper thread
void *looperRun (void* data)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)data;
    
    if(deviceManager != NULL)
    {
        while (deviceManager->run)
        {
            sendPCMD(deviceManager);
            
            sendCameraOrientation(deviceManager);
            
            usleep(50000);
        }
    }
    
    return NULL;
}

int main (int argc, char *argv[])
{
 
    //Test OpenCV
    //IplImage* img = cvLoadImage("/home/hdaoud/Pictures/Leo-October.png",CV_LOAD_IMAGE_COLOR);
    //
    //  if (!img)
    //  {
    //    printf("Image can NOT Load!!!\n");
    //    return 1;
    //  }
    //
    //  cvNamedWindow("Display Image",CV_WINDOW_AUTOSIZE);
    //  cvShowImage("Display Image", img);
    //
    //  cvWaitKey(0);
    //  cvReleaseImage(&img);

  
    /* local declarations */
    int failed = 0;
    BD_MANAGER_t *deviceManager = malloc(sizeof(BD_MANAGER_t));
    
     
    //videoWriter = cvCreateVideoWriter("./images/BebopRGBFrames.avi", CV_FOURCC('M','J','P','G'), 6/1, cvSize(frameWidth,frameHeight), 1);
    
        

    pid_t child = 0;
    
    // fork the process to launch ffplay
    if ((child = fork()) == 0)
    {

        // redirect stdout and stderr of mplayer to dev/null to avoid messing with ncurse
        int stdout_fd = open("/dev/null", O_RDONLY);
        if (stdout_fd == -1)
            return -1;
        dup2(stdout_fd, STDOUT_FILENO);
        close(stdout_fd);
        
        int stderr_fd = open("/dev/null", O_RDONLY);
        if (stderr_fd == -1)
            return -1;
        dup2(stderr_fd, STDERR_FILENO);
        close(stderr_fd);


        execlp("mplayer", "mplayer", "./video_decoded_fifo", "-demuxer", "rawvideo", "-rawvideo", "w=640:h=368:fps=30:format=i420", ">/dev/null", "2>/dev/null", NULL);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer.");
        return -1;
    }
    
    if (deviceManager == NULL)
    {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "deviceManager alloc error !");
    }

    if (!failed)
    {
        ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Starting --");

        
        // initialize jsMnager
        deviceManager->alManager = NULL;
        deviceManager->netManager = NULL;
        deviceManager->streamReader = NULL;
        deviceManager->looperThread = NULL;
        deviceManager->rxThread = NULL;
        deviceManager->txThread = NULL;
        deviceManager->videoRxThread = NULL;
        deviceManager->videoTxThread = NULL;
        deviceManager->d2cPort = BD_D2C_PORT;
        deviceManager->c2dPort = BD_C2D_PORT; //deviceManager->c2dPort = 0; // Should be read from json
        deviceManager->arstreamAckDelay = 0; // Should be read from json
        deviceManager->arstreamFragSize = BD_NET_DC_VIDEO_FRAG_SIZE; // Should be read from json
        deviceManager->arstreamFragNb   = BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG; // Should be read from json

        deviceManager->video_out = fopen("./video_decoded_fifo", "w");
	if(bUseRGB)
        	deviceManager->video_outRGB = fopen("./video_decoded_fifo_RGB","w");
        deviceManager->IMU_out = fopen("./IMU_Data","w");
        deviceManager->Quaternions_out = fopen("./Quaternions","w");
            
        frameProcessing = fopen("./video_decoded", "w");//Not in use..
        
        deviceManager->decoder = NULL;
        deviceManager->decodingCanceled = 1;
        deviceManager->decodingThread = NULL;
        
        deviceManager->ihm = NULL;
        
        deviceManager->hasReceivedFirstIFrame = 0;
        
        deviceManager->freeRawFramePool = NULL;
        deviceManager->rawFramePoolCapacity = 0;
        deviceManager->lastRawFrameFreeIdx = 0;
        
        deviceManager->rawFrameFifo = NULL;
        deviceManager->fifoReadIdx = 0;
        deviceManager->fifoWriteIdx = BD_RAW_FRAME_BUFFER_SIZE - 1;
        deviceManager->run = 1;
        
        deviceManager->dataPCMD.flag = 0;
        deviceManager->dataPCMD.roll = 0;
        deviceManager->dataPCMD.pitch = 0;
        deviceManager->dataPCMD.yaw = 0;
        deviceManager->dataPCMD.gaz = 0;
        
        deviceManager->dataCam.tilt = 0;
        deviceManager->dataCam.pan = 0;
        
        deviceManager->dataAttitude.roll = 0;
        deviceManager->dataAttitude.pitch = 0;
        deviceManager->dataAttitude.yaw = 0;
        deviceManager->dataAttitude.timestamp_usec = 0;
        
        deviceManager->dataPosition.latitude = 0;
        deviceManager->dataPosition.longitude = 0;
        deviceManager->dataPosition.altitude = 0;
        
        deviceManager->dataSpeed.speedX = 0;
        deviceManager->dataSpeed.speedY = 0;
        deviceManager->dataSpeed.speedZ = 0;
        deviceManager->dataSpeed.timestamp_usec = 0;
        
        deviceManager->imuAngle = false;
        deviceManager->imuSpeed = false;
        
        deviceManager->flyingState = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
        deviceManager->sensorName = ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAX;
                
    }
    
    if (!failed)
    {
ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Connecting to Drone --");
        failed = ardiscoveryConnect (deviceManager);
    }

    if (!failed)
    {
ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Reading Config --");
        ARSTREAM_Reader_InitStreamDataBuffer (&d2cParams[2], BD_NET_DC_VIDEO_DATA_ID, deviceManager->arstreamFragSize, deviceManager->arstreamFragNb);
        ARSTREAM_Reader_InitStreamAckBuffer (&c2dParams[3], BD_NET_CD_VIDEO_ACK_ID);
    }

    if (!failed)
    {
ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Start Network --");
        /* start */
        failed = startNetwork (deviceManager);
    }
    
    if (!failed)
    {
        failed = startDecoder(deviceManager);
    }

    if (!failed)
    {    
        failed = startVideo (deviceManager);
        
        //Initialize YUV and get return result
        if(frameWidth && frameHeight)
        {
            retr = YUV_init(frameProcessing, frameWidth, frameHeight, &cap);
            if(retr != YUV_OK)
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,"Error in YUV initialization");
            else
            {
                rgb = cvCreateImage(cvSize(frameWidth,frameHeight), IPL_DEPTH_8U, 3);
                assert(rgb);
            }
        }
    }
    
    if (!failed)
    {
        int cmdSend = sendDate(deviceManager);
        failed = !cmdSend;
    }
    
    if (!failed)
    {
        int cmdSend = sendAllSettings(deviceManager);
        failed = !cmdSend;
    }
    
    if (!failed)
    {
        int cmdSend = sendAllStates(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        int cmdSend = sendBeginStream(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        // allocate reader thread array.
        deviceManager->readerThreads = calloc(numOfCommandBufferIds, sizeof(ARSAL_Thread_t));
        
        if (deviceManager->readerThreads == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // allocate reader thread data array.
        deviceManager->readerThreadsData = calloc(numOfCommandBufferIds, sizeof(READER_THREAD_DATA_t));
        
        if (deviceManager->readerThreadsData == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads data failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // Create and start reader threads.
        int readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
        {
            // initialize reader thread data
            deviceManager->readerThreadsData[readerThreadIndex].deviceManager = deviceManager;
            deviceManager->readerThreadsData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];
            
            if (ARSAL_Thread_Create(&(deviceManager->readerThreads[readerThreadIndex]), readerRun, &(deviceManager->readerThreadsData[readerThreadIndex])) != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of reader thread failed.");
                failed = 1;
            }
        }
    }
    
    if (!failed)
    {
        // Create and start looper thread.
        if (ARSAL_Thread_Create(&(deviceManager->looperThread), looperRun, deviceManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of looper thread failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        deviceManager->ihm = IHM_New (&onInputEvent);
        if (deviceManager->ihm != NULL)
        {
            gErrorStr[0] = '\0';
            ARSAL_Print_SetCallback (customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM
            
            IHM_PrintHeader(deviceManager->ihm, "-- Bebop Drone Decode Video Stream --");
            registerARCommandsCallbacks (deviceManager);
            IHM_setCustomData(deviceManager->ihm, deviceManager);
            
           gIHMRun = 1;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        IHM_PrintInstruction(deviceManager->ihm, "Use Arrow keys to move (Roll/Pitch) ; \n'w' = up ; 's' = down ; 'a' = yaw left ; 'd' = yaw right; \nSpacebar to take off/land; 'f' Flat Trim ; \n'k' = move camera down ; 'i' = move camera up ; 'j' = move camera left ; 'l' = move camera right ;\n'm' = EMERGENCY\n'esc' to quit");
        
        while (gIHMRun)
        {
            usleep(50);
        }
        
        IHM_PrintInfo(deviceManager->ihm, "Disconnecting ...");
    }

    if (deviceManager != NULL)
    {
        deviceManager->run = 0; // break threads loops

        // Stop looper Thread
        if (deviceManager->looperThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->looperThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->looperThread));
            deviceManager->looperThread = NULL;
        }
        
        /* stop */
        if (deviceManager->readerThreads != NULL)
        {
            // Stop reader Threads
            int readerThreadIndex = 0;
            for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
            {
                if (deviceManager->readerThreads[readerThreadIndex] != NULL)
                {
                    ARSAL_Thread_Join(deviceManager->readerThreads[readerThreadIndex], NULL);
                    ARSAL_Thread_Destroy(&(deviceManager->readerThreads[readerThreadIndex]));
                    deviceManager->readerThreads[readerThreadIndex] = NULL;
                }
            }
            
            // free reader thread array
            free (deviceManager->readerThreads);
            deviceManager->readerThreads = NULL;
        }
        
        if (deviceManager->readerThreadsData != NULL)
        {
            // free reader thread data array
            free (deviceManager->readerThreadsData);
            deviceManager->readerThreadsData = NULL;
        }

        stopVideo (deviceManager);
        stopDecoder (deviceManager);
        stopNetwork (deviceManager);
        
        fclose(deviceManager->video_out);
	if(bUseRGB)
        	fclose(deviceManager->video_outRGB);
        fclose(deviceManager->IMU_out);
        fclose(deviceManager->Quaternions_out);
        fclose (frameProcessing);//For YUV2RGB - Not in use!
        
        free (deviceManager);

        YUV_cleanup(&cap, rgb);
        cvReleaseVideoWriter(&videoWriter);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    if (child > 0)
    {
        kill(child, SIGKILL);
    }

    return 0;
}

/************************** Connection part **************************/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");
    
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, deviceManager, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData : %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }
    
    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, BD_DISCOVERY_PORT, BD_IP_ADDRESS);
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection : %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }
    
    ARDISCOVERY_Connection_Delete(&discoveryData);
    
    return failed;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    
    if ((dataTx != NULL) && (dataTxSize != NULL) && (deviceManager != NULL))
    {
        *dataTxSize = sprintf((char *)dataTx, "{ \"%s\": %d,\n \"%s\": \"%s\",\n \"%s\": \"%s\" }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, deviceManager->d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "toto",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "tata") + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }
    
    return err;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    
    if ((dataRx != NULL) && (dataRxSize != 0) && (deviceManager != NULL))
    {
        char *json = malloc(dataRxSize + 1);
        strncpy(json, (char *)dataRx, dataRxSize);
        json[dataRxSize] = '\0';
        
        //read c2dPort ...
        
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "    - ReceiveJson:%s ", json);
        
        free(json);
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }
    
    return err;
}

/************************** Network part **************************/
int startNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARNetwork");

    // Create the ARNetworkALManager
    deviceManager->alManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        // Initilize the ARNetworkALManager
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(deviceManager->alManager, BD_IP_ADDRESS, BD_C2D_PORT, BD_D2C_PORT, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create the ARNetworkManager.
        deviceManager->netManager = ARNETWORK_Manager_New(deviceManager->alManager, numC2dParams, c2dParams, numD2cParams, d2cParams, pingDelay, onDisconnectNetwork, deviceManager, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->rxThread), ARNETWORK_Manager_ReceivingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->txThread), ARNETWORK_Manager_SendingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Tx thread failed.");
            failed = 1;
        }
    }

    // Print net error
    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWorkAL Error : %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWork Error : %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}

void stopNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARNetwork");

    // ARNetwork cleanup
    if (deviceManager->netManager != NULL)
    {
        ARNETWORK_Manager_Stop(deviceManager->netManager);
        if (deviceManager->rxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->rxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->rxThread));
            deviceManager->rxThread = NULL;
        }

        if (deviceManager->txThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->txThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->txThread));
            deviceManager->txThread = NULL;
        }
    }

    if (deviceManager->alManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(deviceManager->alManager);
    }

    ARNETWORK_Manager_Delete(&(deviceManager->netManager));
    ARNETWORKAL_Manager_Delete(&(deviceManager->alManager));
}

void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "onDisconnectNetwork ...");
    gIHMRun = 0;
}

/************************** Video part **************************/
int startVideo(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARSTREAM_ERROR err = ARSTREAM_OK;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARStream");

    deviceManager->videoFrameSize = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
    deviceManager->videoFrame = malloc (deviceManager->videoFrameSize);
    if (deviceManager->videoFrame == NULL)
    {
        failed = 1;
    }
    
    if (! failed)
    {
        deviceManager->streamReader = ARSTREAM_Reader_New (deviceManager->netManager, BD_NET_DC_VIDEO_DATA_ID, BD_NET_CD_VIDEO_ACK_ID, frameCompleteCallback, deviceManager->videoFrame, deviceManager->videoFrameSize, deviceManager->arstreamFragSize, deviceManager->arstreamAckDelay, deviceManager, &err);
        if (err != ARSTREAM_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during ARStream creation : %s", ARSTREAM_Error_ToString(err));
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->videoRxThread), ARSTREAM_Reader_RunDataThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->videoTxThread), ARSTREAM_Reader_RunAckThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Tx thread failed.");
            failed = 1;
        }
    }

    return failed;
}

void stopVideo(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARSTREAM_ERROR err = ARSTREAM_OK;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARStream");

    if (deviceManager->streamReader)
    {
        ARSTREAM_Reader_StopReader(deviceManager->streamReader);

        // Optionnal, but better for speed:
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        if (deviceManager->videoRxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoRxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoRxThread));
            deviceManager->videoRxThread = NULL;
        }
        if (deviceManager->videoTxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoTxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoTxThread));
            deviceManager->videoTxThread = NULL;
        }

        ARSTREAM_Reader_Delete (&(deviceManager->streamReader));
    }
}

uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom)
{
    uint8_t *ret = NULL;
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)custom;
    
    switch(cause)
    {
        case ARSTREAM_READER_CAUSE_FRAME_COMPLETE:
        {
            //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Did receive a frame");
            
            RawFrame_t *freeFrame = getFrameFromData(deviceManager, frame);
            if (freeFrame != NULL)
            {
                freeFrame->data = frame;
                freeFrame->size = frameSize;
                freeFrame->isIframe = isFlushFrame;
            }
            
            if (isFlushFrame)
            {
                // since we received an iFrame, flush the fifo
                flushFifo(deviceManager);
            }

            if (freeFrame != NULL)
            {
                // push the received frame into the fifo
                freeFrame->data = frame;
                freeFrame->size = frameSize;
                
                addFreeRawFrameToFifo(deviceManager, freeFrame);
            }
            
            // get free frame for the next frame
            RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
            ret = freeRawFrame->data;
            *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
        }

            break;
        case ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL:
            /* This case should not happen, as we've allocated a frame pointer to the maximum possible size. */
        {
            RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
            ret = freeRawFrame->data;
            *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
        }
            break;
        case ARSTREAM_READER_CAUSE_COPY_COMPLETE:
            /* Same as before ... but return value are ignored, so we just do nothing */
            break;
        case ARSTREAM_READER_CAUSE_CANCEL:
            /* Called when the library closes, return values ignored, so do nothing here */
            break;
        default:
            break;
    }

    return ret;
}

/************************** Commands part **************************/
int sendPCMD(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    // Send pcmd command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingPCMD(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataPCMD.flag, (uint8_t)deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch, (uint8_t)deviceManager->dataPCMD.yaw, (uint8_t)deviceManager->dataPCMD.gaz, 0);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendCameraOrientation(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    // Send camera orientation command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3CameraOrientation(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataCam.tilt, (uint8_t)deviceManager->dataCam.pan);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendDate(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send date");
    
    time_t t;
    struct tm* timeinfo;
    char curntdt [10];
    char curtime [14];
    time(&t);
    timeinfo = localtime(&t);
    
    strftime(curntdt,10,"%F",timeinfo);
    sprintf(curtime, "'T'%d%d%d+0800", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    // Send date command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, curntdt);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    if (sentStatus)
    {
        
        // Send time command
        cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, curtime);//"'T'101533+0200");
        if (cmdError == ARCOMMANDS_GENERATOR_OK)
        {
            netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
        }
        
        if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
            sentStatus = 0;
        }
    }
    
    return sentStatus;
}

int sendAllSettings(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all settings");
    
    // Send get all settings command
    cmdError = ARCOMMANDS_Generator_GenerateCommonSettingsAllSettings(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all settings command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;    
}

int sendAllStates(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all states");
    
    // Send get all states command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendBeginStream(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Streaming Begin");
    
    // Send Streaming begin command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendFlatTrim(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Flat Trim");
    
    // Send flat trim command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingFlatTrim(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send flat trim command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;    
}

int sendTakeoff(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send take off");
    
    // Send take off command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingTakeOff(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send takeoff command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendLanding(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send landing");
    
    // Send landing command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingLanding(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send landing command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendEmergency(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Emergency");
    
    // Send emergency command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingEmergency(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_EMERGENCY_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send emergency command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "    - arnetworkCmdCallback %d, cause:%d ", buffer_id, cause);
    
    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }
    
    return retval;
}

/************************** Commands callback part **************************/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager)
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback(batteryStateChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(flyingStateChangedCallback, deviceManager);    
    // ADD HERE THE CALLBACKS YOU ARE INTERESTED IN
    ARCOMMANDS_Decoder_SetCommonCommonStateSensorsStatesListChangedCallback(sensorsStatesListChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStatePositionChangedCallback(pilotingStatePositionChangedCallback, deviceManager); 
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(speedChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback(attitudeChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback(altitudeChangedCallback, deviceManager);
    //ARCOMMANDS_Decoder_SetARDrone3GPSSettingsStateGPSUpdateStateChangedCallback(gpsStateChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3GPSSettingsStateGPSFixStateChangedCallback(gpsFixStateChangedCallback, deviceManager);
    //ARCOMMANDS_Decoder_SetARDrone3GPSStateNumberOfSatelliteChangedCallback(gpsNumberOfSatelliteChangedCallback, deviceManager);
}

void unregisterARCommandsCallbacks ()
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetCommonCommonStateSensorsStatesListChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStatePositionChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback(NULL, NULL);
    //ARCOMMANDS_Decoder_SetARDrone3GPSSettingsStateGPSUpdateStateChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3GPSSettingsStateGPSFixStateChangedCallback(NULL, NULL);
    //ARCOMMANDS_Decoder_SetARDrone3GPSStateNumberOfSatelliteChangedCallback(NULL, NULL);
}

void batteryStateChangedCallback (uint8_t percent, void *custom)
{
    // callback of changing of battery level
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {
        IHM_PrintBattery (deviceManager->ihm, percent);
    }
}

void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    if (deviceManager != NULL)
    {
        deviceManager->flyingState = state;
        
        switch (deviceManager->flyingState) {
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : landed");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_TAKINGOFF:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : taking off");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : hovering");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : flying");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : landing");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_EMERGENCY:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : emergency");
                break;
            default:
                break;
        }
    }
}

void sensorsStatesListChangedCallback(eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName, uint8_t state, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    char *sensorState = malloc(15);
    sprintf(sensorState, "%d", state);
    
    if (deviceManager != NULL)
    {
        deviceManager->sensorName = sensorName;
        
        switch(deviceManager->sensorName)
        {
            case ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_IMU:
                IHM_PrintSensorState(deviceManager->ihm, concat("IMU: ", sensorState));
                break;
            case ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_BAROMETER:
                IHM_PrintSensorState(deviceManager->ihm, concat("Barometer: ", sensorState));
                break;
            case ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_ULTRASOUND:
                IHM_PrintSensorState(deviceManager->ihm, concat("Ultrasound: ", sensorState));
                break;
            case ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_GPS:
                IHM_PrintSensorState(deviceManager->ihm, concat("GPS: ", sensorState));
                break;
            case ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAGNETOMETER:
                IHM_PrintSensorState(deviceManager->ihm, concat("Magnetometer: ", sensorState));
                break;
            default:
                break;
        }
    }
    
    free(sensorState);
}

void pilotingStatePositionChangedCallback(double latitude, double longitude, double altitude, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
    {        
        deviceManager->dataPosition.latitude = latitude;
        deviceManager->dataPosition.longitude = longitude;
        deviceManager->dataPosition.altitude = altitude;
        
        if(deviceManager->ihm != NULL)
        {
            IHM_PrintPosition(deviceManager->ihm, latitude, longitude, altitude);
        }
    }
}

void speedChangedCallback(float speedX, float speedY, float speedZ, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
    {
        deviceManager->dataSpeed.speedX = speedX;
        deviceManager->dataSpeed.speedY = speedY;
        deviceManager->dataSpeed.speedZ = speedZ;
        if(!gettimeofday(&timer_usec, NULL))
            deviceManager->dataSpeed.timestamp_usec = ((long long int)timer_usec.tv_sec)* 100000011 + (long long int)timer_usec.tv_usec;
        else
            deviceManager->dataSpeed.timestamp_usec = -1;
        
        deviceManager->imuSpeed = true;
        
        if(deviceManager->imuSpeed && deviceManager->imuAngle)
        {
                           
            char* str = malloc(256);

            long dt = deviceManager->dataSpeed.timestamp_usec - deviceManager->dataAttitude.timestamp_usec;
            
            //sprintf(str, "%d\n", dt);
            //fwrite(str, strlen(str)*sizeof(char), 1, deviceManager->Quaternions_out);

            //if the time difference is less than EPSILON the measurements of attitude and speed are concurrent
            if( abs(dt) < EPSILON )
            {
                deviceManager->imuSpeed = false;
                deviceManager->imuAngle = false;

                //sprintf(str, "s: %lld %f %f %f\n", timestamp_usec, speedX, speedY, speedZ);
                sprintf(str, "%lld %f %f %f %f %f %f\n", deviceManager->dataSpeed.timestamp_usec, speedX, speedY, speedZ, 
                        deviceManager->dataAttitude.roll, deviceManager->dataAttitude.pitch, deviceManager->dataAttitude.yaw);


                fwrite(str, strlen(str) * sizeof(char), 1, deviceManager->IMU_out);
                
                //Use MadgwickAHRS to calculate attitude 
                float* res = MadgwickAHRSupdateIMU(deviceManager->dataAttitude.pitch, deviceManager->dataAttitude.roll, deviceManager->dataAttitude.yaw,
                        speedX, speedY, speedZ);
                sprintf(str, "%lld %f %f %f %f\n", deviceManager->dataSpeed.timestamp_usec, res[0], res[1], res[2], res[3]);
                fwrite(str, strlen(str)*sizeof(char), 1, deviceManager->Quaternions_out);
            }
            
        }
        
        if(deviceManager->ihm != NULL)
        {
            IHM_PrintSpeed(deviceManager->ihm, speedX, speedY, speedZ);
        }
    }
}

void attitudeChangedCallback(float roll, float pitch, float yaw, void *custom)
{   
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
    {
        deviceManager->dataAttitude.roll = roll;
        deviceManager->dataAttitude.pitch = pitch;
        deviceManager->dataAttitude.yaw = yaw;
        if(!gettimeofday(&timer_usec, NULL))
            deviceManager->dataAttitude.timestamp_usec = ((long long int)timer_usec.tv_sec)* 100000011 + (long long int)timer_usec.tv_usec;
        else
            deviceManager->dataAttitude.timestamp_usec = -1;
        
        deviceManager->imuAngle = true;
        
        if(deviceManager->imuSpeed && deviceManager->imuAngle)
        {     
            char* str = malloc(256);
           
            int dt = (int)(deviceManager->dataSpeed.timestamp_usec - deviceManager->dataAttitude.timestamp_usec);

            //if the time difference is less than EPSILON the measurements of attitude and speed are concurrent
            if( abs(dt) < EPSILON )
            {
                deviceManager->imuSpeed = false;
                deviceManager->imuAngle = false;

                sprintf(str, "%lld %f %f %f %f %f %f\n", deviceManager->dataSpeed.timestamp_usec, 
                        deviceManager->dataSpeed.speedX, deviceManager->dataSpeed.speedY, deviceManager->dataSpeed.speedZ, 
                        roll, pitch, yaw);

                fwrite(str, strlen(str) * sizeof(char), 1, deviceManager->IMU_out);
                
                float* res = MadgwickAHRSupdateIMU(pitch, roll, yaw,
                        deviceManager->dataSpeed.speedX, deviceManager->dataSpeed.speedY, deviceManager->dataSpeed.speedZ);
                
                sprintf(str, "%lld %f %f %f %f\n", deviceManager->dataSpeed.timestamp_usec, res[0], res[1], res[2], res[3]);
                fwrite(str, strlen(str)*sizeof(char), 1, deviceManager->Quaternions_out);
            }
                        
        }
        
        if(deviceManager->ihm != NULL)
        {
            IHM_PrintAttitude(deviceManager->ihm, roll, pitch, yaw);
        }
    } 
}

void altitudeChangedCallback(double altitude, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
    {
        deviceManager->altitude = altitude;
        if(deviceManager->ihm != NULL)
        {
            IHM_PrintAltitude(deviceManager->ihm, altitude);
        }
    }
}

void gpsStateChangedCallback(eARCOMMANDS_ARDRONE3_GPSSETTINGSSTATE_GPSUPDATESTATECHANGED_STATE state, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
    {
        switch(state)
        {
            case ARCOMMANDS_ARDRONE3_GPSSETTINGSSTATE_GPSUPDATESTATECHANGED_STATE_UPDATED: IHM_PrintGPSState(deviceManager->ihm, "Updated");
                break;
            case ARCOMMANDS_ARDRONE3_GPSSETTINGSSTATE_GPSUPDATESTATECHANGED_STATE_INPROGRESS: IHM_PrintGPSState(deviceManager->ihm, "In Progress");
                break;
            case ARCOMMANDS_ARDRONE3_GPSSETTINGSSTATE_GPSUPDATESTATECHANGED_STATE_FAILED: IHM_PrintGPSState(deviceManager->ihm, "Failed");
                break;
            default: IHM_PrintGPSState(deviceManager->ihm, " "); break;
        }

    }
    
}

void gpsFixStateChangedCallback(uint8_t fixed, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if (deviceManager != NULL)
        IHM_PrintGPSFix(deviceManager->ihm, fixed);
}

void gpsNumberOfSatelliteChangedCallback(uint8_t numberOfSatellite, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    if (deviceManager != NULL)
        IHM_PrintGPSState(deviceManager->ihm, numberOfSatellite);
}



/************************** Decoding part **************************/
int startDecoder(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    
    // create ffmpeg decoder
    if (!failed)
    {
        eARCODECS_ERROR error;
        deviceManager->decoder = ARCODECS_Manager_New (getNextDataCallback, deviceManager, &error);
        if (error != ARCODECS_OK)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        if (ARSAL_Mutex_Init (&(deviceManager->mutex)) != 0)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // prepare the pool : a large number of frame allocated
        deviceManager->freeRawFramePool = malloc(sizeof(RawFrame_t*) * BD_RAW_FRAME_POOL_SIZE);
        if (deviceManager->freeRawFramePool != NULL)
        {
            deviceManager->rawFramePoolCapacity = BD_RAW_FRAME_POOL_SIZE;
        }
        else
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool init");
        }
    }
    
    if (!failed)
    {
        // Allocate all frames of the pool
        int i = 0;
        for (i = 0; (!failed) && (i < deviceManager->rawFramePoolCapacity); i++)
        {
            RawFrame_t *rawFrame = malloc(sizeof(RawFrame_t));
            if (rawFrame != NULL)
            {
                rawFrame->size = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
                rawFrame->isIframe = 0;
                rawFrame->data = malloc (rawFrame->size);
                if (rawFrame->data == NULL)
                {
                    failed = 1;
                }
                else
                {
                    deviceManager->freeRawFramePool[i] = rawFrame;
                }
            }
            else
            {
                failed = 1;
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool object init");
            }
        }
        
        if (!failed)
        {
            deviceManager->lastRawFrameFreeIdx = 0;
        }
    }
    
    if (!failed)
    {
        // Init the frame fifo
        deviceManager->rawFrameFifo = calloc (BD_RAW_FRAME_BUFFER_SIZE, sizeof(RawFrame_t*));
        if (deviceManager->rawFrameFifo == NULL)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // Create the decoding thread
        deviceManager->decodingCanceled = 0;
        if (ARSAL_Thread_Create(&(deviceManager->decodingThread), Decode_RunDataThread, deviceManager) != 0)
        {
            deviceManager->decodingCanceled = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video decodingThread failed.");
            failed = 1;
        }
    }
    
    return failed;
}

void stopDecoder(BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Stop decoding");
    if (deviceManager->decodingCanceled == 0)
    {
        deviceManager->decodingCanceled = 1;
        ARSAL_Thread_Join(deviceManager->decodingThread, NULL);
        ARSAL_Thread_Destroy(&(deviceManager->decodingThread));
        deviceManager->decodingThread = NULL;
    }
    
    if (deviceManager->videoFrame)
    {
        free (deviceManager->videoFrame);
        deviceManager->videoFrame = NULL;
    }
    
    if (deviceManager->freeRawFramePool != NULL)
    {
        flushFifo(deviceManager);
        int i = 0;
        for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
        {
            free(deviceManager->freeRawFramePool[i]);
            deviceManager->freeRawFramePool[i] = NULL;
        }
        
        free(deviceManager->freeRawFramePool);
        deviceManager->freeRawFramePool = NULL;
    }

    if (deviceManager->rawFrameFifo)
    {
        free (deviceManager->rawFrameFifo);
        deviceManager->rawFrameFifo = NULL;
    }

    if (deviceManager->mutex != NULL)
    {
        ARSAL_Mutex_Destroy(&deviceManager->mutex);
        deviceManager->mutex = NULL;
    }

    if (deviceManager->decoder != NULL)
    {
        ARCODECS_Manager_Delete(&deviceManager->decoder);
    }
}

RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager)
{
    // get a free raw frame in the pool
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *freeRawFrame = NULL;
    if (deviceManager->lastRawFrameFreeIdx < deviceManager->rawFramePoolCapacity)
    {
        freeRawFrame = deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx];
        if (freeRawFrame == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Free frame is null.");
        }

        deviceManager->lastRawFrameFreeIdx++;
    }
    else
    {
        // there is no more free raw frame in the pool, create a new one and add it to the pool
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "No more frame free, need to alloc a new one and add it to the pool");
        freeRawFrame = malloc(sizeof(RawFrame_t));
        if (freeRawFrame != NULL)
        {
            RawFrame_t **freeRawFramePoolReallocated = realloc(deviceManager->freeRawFramePool, deviceManager->rawFramePoolCapacity + 1);
            if (freeRawFramePoolReallocated != NULL)
            {
                deviceManager->freeRawFramePool = freeRawFramePoolReallocated;
                deviceManager->rawFramePoolCapacity++;
                
                deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx] = freeRawFrame;
                deviceManager->lastRawFrameFreeIdx++;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return freeRawFrame;
}

RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data)
{
    // get the frame which has the given data
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = NULL;
    int i = 0;
    for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
    {
        RawFrame_t *currentRawFrame = deviceManager->freeRawFramePool[i];
        if (currentRawFrame != NULL && currentRawFrame->data != NULL)
        {
            if (currentRawFrame->data == data)
            {
                rawFrame = currentRawFrame;
                break;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return rawFrame;
}

void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame)
{
    // put the frame from the pool to the Fifo
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    int idxToWrite = (deviceManager->fifoWriteIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
    
    // put the old frame on the free frame pool
    if (deviceManager->rawFrameFifo[idxToWrite] != NULL)
    {
        putRawFrameBackToPool(deviceManager, idxToWrite);
    }
    
    // put the new rawFrame at the write idx
    deviceManager->rawFrameFifo[idxToWrite] = rawFrame;
    deviceManager->fifoWriteIdx = idxToWrite;
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void flushFifo(BD_MANAGER_t *deviceManager)
{
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    
    int shouldContinue = 1;
    int currentRawFrameIdx = deviceManager->fifoReadIdx;
    do
    {
        if (deviceManager->rawFrameFifo[currentRawFrameIdx] != NULL)
        {
            putRawFrameBackToPool(deviceManager, currentRawFrameIdx);
        }
        currentRawFrameIdx = (currentRawFrameIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
    } while (currentRawFrameIdx != deviceManager->fifoReadIdx);
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx)
{
    // remove the frame from the fifo and put it back to the pool => declare the frame as unused
    if (deviceManager->rawFrameFifo[fifoIdx] != NULL)
    {
        deviceManager->lastRawFrameFreeIdx--;
        
        deviceManager->rawFrameFifo[fifoIdx] = NULL;
    }
}

int getNextDataCallback(uint8_t **data, void *customData)
{
    // this callback is used by the decoder to get the frame to decode
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    int size = 0;
    
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    
    if (data != NULL && rawFrame != NULL)
    {
        *data = rawFrame->data;
        size = rawFrame->size;
        
    }
    return size;
}


/************************** IHM callbacks **************************/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData)
{
    // Manage IHM input events
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    
    switch (event)
    {
        case IHM_INPUT_EVENT_EXIT:
            gIHMRun = 0;
            break;
        case IHM_INPUT_EVENT_FLAT_TRIM:
            if(deviceManager != NULL)
            {
                if(deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED)
                {
                    sendFlatTrim(deviceManager);
                }
            }
            break;
        case IHM_INPUT_EVENT_EMERGENCY:
            if(deviceManager != NULL)
            {
                sendEmergency(deviceManager);
            }
            break;
        case IHM_INPUT_EVENT_TAKEOFF_LANDING:
            if(deviceManager != NULL)
            {
                if (deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED)
                {
                    sendTakeoff(deviceManager);
                }
                else if ((deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING) ||
                         (deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING))
                {
                    sendLanding(deviceManager);
                }
                    
            }
            break;
        case IHM_INPUT_EVENT_FORWARD:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.flag = 1;
                deviceManager->dataPCMD.pitch = 50;
            }
            break;
        case IHM_INPUT_EVENT_BACK:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.flag = 1;
                deviceManager->dataPCMD.pitch = -50;
            }
            break;
        case IHM_INPUT_EVENT_RIGHT:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.flag = 1;
                deviceManager->dataPCMD.roll = 50;
            }
            break;
        case IHM_INPUT_EVENT_LEFT:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.flag = 1;
                deviceManager->dataPCMD.roll = -50;
            }
            break;
        case IHM_INPUT_EVENT_YAW_RIGHT:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.yaw = 50;
            }
            break;
        case IHM_INPUT_EVENT_YAW_LEFT:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.yaw = -50;
            }
            break;
        case IHM_INPUT_EVENT_UP:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.gaz = 50;
            }
            break;
        case IHM_INPUT_EVENT_DOWN:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.gaz = -50;
            }
            break;
        case IHM_INPUT_EVENT_CAM_UP:
            if(deviceManager != NULL)
            {
                deviceManager->dataCam.tilt += 2;
                if (deviceManager->dataCam.tilt > 80)
                {
                    deviceManager->dataCam.tilt = 80;
                }
            }
            break;
        case IHM_INPUT_EVENT_CAM_DOWN:
            if(deviceManager != NULL)
            {
                deviceManager->dataCam.tilt -= 2;
                if (deviceManager->dataCam.tilt < -80)
                {
                    deviceManager->dataCam.tilt = -80;
                }
            }
            break;
        case IHM_INPUT_EVENT_CAM_RIGHT:
            if(deviceManager != NULL)
            {
                deviceManager->dataCam.pan += 2;
                if (deviceManager->dataCam.pan > 80)
                {
                    deviceManager->dataCam.pan = 80;
                }
            }
            break;
        case IHM_INPUT_EVENT_CAM_LEFT:
            if(deviceManager != NULL)
            {
                deviceManager->dataCam.pan -= 2;
                if (deviceManager->dataCam.pan < -80)
                {
                    deviceManager->dataCam.pan = -80;
                }
            }
            break;
        case IHM_INPUT_EVENT_NONE:
            if(deviceManager != NULL)
            {
                deviceManager->dataPCMD.flag = 0;
                deviceManager->dataPCMD.roll = 0;
                deviceManager->dataPCMD.pitch = 0;
                deviceManager->dataPCMD.yaw = 0;
                deviceManager->dataPCMD.gaz = 0;
            }
            break;
        default:
            break;
    }
}

int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    // Custom callback used when ncurses is runing for not disturb the IHM
    
    if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0))
    {
        // save the last Error
        vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
        gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
    }
    
    return 1;
}

char *concat(char *s1, char *s2)
{
    size_t len1 = strlen(s1);
    size_t len2 = strlen(s2);
    char *result = malloc(len1+len2+1);//+1 for the zero-terminator
    if(result == NULL)
        return "ERROR";
    memcpy(result, s1, len1);
    memcpy(result+len1, s2, len2+1);//+1 to copy the null-terminator
    return result;
}
