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
 * @file ihm.c
 * @brief This file contains sources about ncurses IHM used by arsdk example "BebopDroneDecodeStream"
 * @date 15/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <ctype.h>

#include <libARSAL/ARSAL.h>

#include "ihm.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define HEADER_X 0
#define HEADER_Y 0

#define INSTRUCTION_X 0
#define INSTRUCTION_Y 2

#define BATTERY_X 0
#define BATTERY_Y 10

#define INFO_X 0
#define INFO_Y 12

#define SENSOR_X 0
#define SENSOR_Y 24

#define POSITION_X 0
#define POSITION_Y 14

#define SPEED_X 0
#define SPEED_Y 16

#define ATTITUDE_X 0
#define ATTITUDE_Y 18

#define ALTITUDE_X 0
#define ALTITUDE_Y 20

#define GPS_X 0
#define GPS_Y 22


/*****************************************
 *
 *             private header:
 *
 ****************************************/
void *IHM_InputProcessing(void *data);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

IHM_t *IHM_New (IHM_onInputEvent_t onInputEventCallback)
{
    int failed = 0;
    IHM_t *newIHM = NULL;
    
    // check parameters
    if (onInputEventCallback == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        //  Initialize IHM
        newIHM = malloc(sizeof(IHM_t));
        if (newIHM != NULL)
        {
            //  Initialize ncurses
            newIHM->mainWindow = initscr();
            newIHM->inputThread = NULL;
            newIHM->run = 1;
            newIHM->onInputEventCallback = onInputEventCallback;
            newIHM->customData = NULL;
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        //raw();                  // Line buffering disabled
        cbreak();
        keypad(stdscr, TRUE);
        noecho();               // Don't echo() while we do getch
        timeout(100);
        
        refresh();
    }
    
    if (!failed)
    {
        //start input thread
        if(ARSAL_Thread_Create(&(newIHM->inputThread), IHM_InputProcessing, newIHM) != 0)
        {
            failed = 1;
        }
    }
    
    if (failed)
    {
        IHM_Delete (&newIHM);
    }

    return  newIHM;
}

void IHM_Delete (IHM_t **ihm)
{
    //  Clean up

    if (ihm != NULL)
    {
        if ((*ihm) != NULL)
        {
            (*ihm)->run = 0;
            
            if ((*ihm)->inputThread != NULL)
            {
                ARSAL_Thread_Join((*ihm)->inputThread, NULL);
                ARSAL_Thread_Destroy(&((*ihm)->inputThread));
                (*ihm)->inputThread = NULL;
            }
            
            delwin((*ihm)->mainWindow);
            (*ihm)->mainWindow = NULL;
            endwin();
            refresh();
            
            free (*ihm);
            (*ihm) = NULL;
        }
    }
}

void IHM_setCustomData(IHM_t *ihm, void *customData)
{
    if (ihm != NULL)
    {
        ihm->customData = customData;
    }
}

void *IHM_InputProcessing(void *data)
{
    IHM_t *ihm = (IHM_t *) data;
    int key = 0;
    
    if (ihm != NULL)
    {
        while (ihm->run)
        {
            key = getch();
            
            key = tolower(key);
            
            if (key == 27) // escape character
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EXIT, ihm->customData);
                }
            }
            else if(key == KEY_UP)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
                }
            }
            else if(key == KEY_DOWN)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_BACK, ihm->customData);
                }
            }
            else if(key == KEY_LEFT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_LEFT, ihm->customData);
                }
            }
            else if(key == KEY_RIGHT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                }
            }
            else if(key == 'm')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EMERGENCY, ihm->customData);
                }
            }
            else if(key == 'w')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_UP, ihm->customData);
                }
            }
            else if(key == 's')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_DOWN, ihm->customData);
                }
            }
            else if(key == 'a')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_LEFT, ihm->customData);
                }
            }
            else if(key == 'd')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_RIGHT, ihm->customData);
                }
            }
            else if(key == 'i')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_UP, ihm->customData);
                }
            }
            else if(key == 'k')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_DOWN, ihm->customData);
                }
            }
            else if(key == 'j')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_LEFT, ihm->customData);
                }
            }
            else if(key == 'l')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_RIGHT, ihm->customData);
                }
            }
            else if(key == ' ')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_TAKEOFF_LANDING, ihm->customData);
                }
            }
            else if(key == 'f')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_FLAT_TRIM, ihm->customData);
                }
            }
            else
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                }
            }
            
            usleep(10);
        }
    }
    
    return NULL;
}

void IHM_PrintHeader(IHM_t *ihm, char *headerStr)
{
    if (ihm != NULL)
    {
        move(HEADER_Y, 0);   // move to begining of line
        clrtoeol();          // clear line
        mvprintw(HEADER_Y, HEADER_X, headerStr);
    }
}

void IHM_PrintInstruction(IHM_t *ihm, char *instructionStr)
{
    if (ihm != NULL)
    {
        move(INSTRUCTION_Y, 0);   // move to begining of line
        clrtoeol();          // clear line
        mvprintw(INSTRUCTION_Y, INSTRUCTION_X, instructionStr);
    }
}

void IHM_PrintInfo(IHM_t *ihm, char *infoStr)
{
    if (ihm != NULL)
    {
        move(INFO_Y, 0);    // move to begining of line
        clrtoeol();         // clear line
        mvprintw(INFO_Y, INFO_X, infoStr);
    }
}

void IHM_PrintSensorState(IHM_t *ihm, char *sensorStateStr)
{
    if(ihm != NULL)
    {
        int sensorY = SENSOR_Y;
        
        //Print the state of each sensor at different line
        switch(sensorStateStr[0])
        {
            //case 'I': break; // If it's IMU data keep it at beginning.
            case 'B': sensorY += 1; break;//Barometer data at second line.
            case 'U': sensorY += 2; break;//Ultrasound data at third line.
            case 'G': sensorY += 3; break;//GPS data at 4th line
            case 'M': sensorY += 4; break;//Magnetometer data at 5th line.
            case 'E'://Error in malloc to concat two strings.  
            default: break;
        }
        
        move(sensorY, 0);
        clrtoeol();
        mvprintw(sensorY, SENSOR_X, sensorStateStr);
    }
}

void IHM_PrintBattery(IHM_t *ihm, uint8_t percent)
{
    if (ihm != NULL)
    {
        move(BATTERY_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(BATTERY_Y, BATTERY_X, "Battery: %d", percent);
    }
}

void IHM_PrintPosition(IHM_t *ihm, double latitude, double longitude, double altitude)
{
    if (ihm != NULL)
    {
        move(POSITION_Y, 0);
        clrtoeol();
        mvprintw(POSITION_Y, POSITION_X, "Latitude: %9.6f, Longitude: %9.6f, Altitude: %6.3f", latitude, longitude, altitude);
    }
}

void IHM_PrintSpeed(IHM_t *ihm, double speedX, double speedY, double speedZ)
{
    if (ihm != NULL)
    {
        move(SPEED_Y, 0);
        clrtoeol();
        if(speedX < -100 || speedX > 100) speedX=0;
        if(speedY < -100 || speedY > 100) speedY=0;
        if(speedZ < -100 || speedZ > 100) speedZ=0;
        
        mvprintw(SPEED_Y, SPEED_X, "SpeedX: %.3f, SpeedY: %.3f, SpeedZ: %.3f", speedX, speedY, speedZ);
    }
}

void IHM_PrintAttitude(IHM_t *ihm, double roll, double pitch, double yaw)
{
    if (ihm != NULL)
    {
        move(ATTITUDE_Y, 0);
        clrtoeol();
        
        if(roll < -100 || roll > 100) roll = 0;
        if(pitch < -100 || pitch > 100) pitch = 0;
        if(yaw < -100 || yaw > 100) yaw = 0;
        
        mvprintw(ATTITUDE_Y, ATTITUDE_X, "roll: %.3f, pitch: %.3f, yaw: %.3f", roll, pitch, yaw);
    }
}

void IHM_PrintAltitude(IHM_t *ihm, double altitude)
{
    if(ihm != NULL)
    {
        move(ATTITUDE_Y, 0);
        clrtoeol();
        
        if(altitude < 0 || altitude > 100) altitude = 0;
        
        mvprintw(ALTITUDE_Y, ALTITUDE_X, "Altitude: %.3f", altitude);
    }
}

void IHM_PrintGPSState(IHM_t *ihm, uint8_t nos)
{
    if(ihm != NULL)
    {
        move(GPS_Y, 0);
        clrtoeol();
        mvprintw(GPS_Y, GPS_X, "GPS Status: %d", nos);
    }
}

void IHM_PrintGPSFix(IHM_t *ihm, uint8_t fix)
{
    if(ihm != NULL)
    {
        move(GPS_Y-1, 0);
        clrtoeol();
        mvprintw(GPS_Y-1, GPS_X, "GPS Fix: %d", fix);
    }
}
