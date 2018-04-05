/*
 * https://github.com/chelyaev/opencv-yuv
 */

#include "yuv.h"
#include <libavcodec/avcodec.h>

/* YUV Methods */
enum YUV_ReturnValue YUV_init(FILE *fin, size_t w, size_t h, struct YUV_Capture *out)
{
    if (!fin || w % 2 == 1 || h % 2 == 1)
        return YUV_PARAMETER_ERROR;

    out->fin = fin;
    out->width = w;
    out->height = h;

    out->ycrcb = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
    out->y = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    out->cb = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    out->cr = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    out->cb_half = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);
    out->cr_half = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);

    if 
    (
       out->ycrcb == NULL
       ||
       out->y == NULL
       ||
       out->cb == NULL
       ||
       out->cr == NULL
       ||
       out->cb_half == NULL
       ||
       out->cr_half == NULL
    )
    {
        YUV_cleanup(out, NULL);
        return YUV_OUT_OF_MEMORY;
    }

    return YUV_OK;
}

enum YUV_ReturnValue YUV_read(struct YUV_Capture *cap, int YLS)
{
    size_t bytes_read;
    size_t npixels;
    npixels = YLS*cap->height;
 
    if(fseek(cap->fin, -npixels, SEEK_CUR ) == 0)
    {
        bytes_read = fread(cap->y->imageData, sizeof(uint8_t), npixels, cap->fin);

        if (bytes_read == 0)
            return YUV_EOF;
        else if (bytes_read != npixels)
            return YUV_IO_ERROR;

        bytes_read = fread(cap->cb_half->imageData, sizeof(uint8_t), npixels/4, cap->fin);
        if (bytes_read != npixels/4)
            return YUV_IO_ERROR;

        bytes_read = fread(cap->cr_half->imageData, sizeof(uint8_t), npixels/4, cap->fin);
        if (bytes_read != npixels/4)
            return YUV_IO_ERROR;

        cvResize(cap->cb_half, cap->cb, CV_INTER_CUBIC);
        cvResize(cap->cr_half, cap->cr, CV_INTER_CUBIC);
        cvMerge(cap->y, cap->cr, cap->cb, NULL, cap->ycrcb);

        return YUV_OK;
    }
    else
    {
        return YUV_IO_ERROR;
    }
    
}

void YUV_cleanup(struct YUV_Capture *cap, IplImage *rgb)
{
    if (!cap)
        return;

    if (cap->ycrcb) 
        free(cap->ycrcb);
    if (cap->y)
        free(cap->y);
    if (cap->cb)
        free(cap->cb);
    if (cap->cr)
        free(cap->cr);
    if (cap->cb_half)
        free(cap->cb_half);
    if (cap->cr_half)
        free(cap->cr_half);
    if (rgb)
        free(rgb);

}

int yuv2rgb(AVFrame *pict, int width, int height, struct YUV_Capture cap, IplImage *rgb)
{
    int x, y,r,g,b;
        
    /*Y*/
    for(y=0; y < height; y++)
        for(x=0; x < width; x++)
        {
           
            r = pict->data[0][y * pict->linesize[0] + x];
            if (r > 255) 
                r = 255;
            
            if (r < 0) 
                r = 0;
            
            cap.y->imageData[y * width + x] = r ;
//            RImage->imageData[y * width + x] = r >255 ? 255 : r < 0 ? 0 : r;
            
        }
    
    /*Y & V*/
    for(y=0; y < height/2; y++)
        for(x=0; x < width/2; x++)
        {
            g = pict->data[1][y * pict->linesize[1] + x];
            b = pict->data[2][y * pict->linesize[2] + x];
            
            if (g > 255) 
                g = 255;

            if (b > 255) 
                b = 255;

            if (g < 0) 
                g = 0;

            if (b < 0) 
                b = 0;
            
            cap.cb_half->imageData[y * width/2 + x] = g ;
            cap.cr_half->imageData[y * width/2 + x] = b ;
            
        }
    
    cvResize(cap.cb_half, cap.cb, CV_INTER_CUBIC);
    cvResize(cap.cr_half, cap.cr, CV_INTER_CUBIC);
    
    //fprintf(frameProcessing,"\nendframe\n");
    
    cvMerge(cap.y, cap.cb, cap.cr, NULL, cap.ycrcb);
    cvCvtColor(cap.ycrcb, rgb, CV_YUV2RGB);
    
    //cvShowImage("Bebop OpenCV Stream", rgb);
    //cvWaitKey(10);
        
        
    
//    cvSaveImage("opencvFINAL.png", RGBImage, 0); 
    
     
    //fclose(frameProcessing);
    
    return 1;
 
}

int copyYUVdata(AVFrame* avFrame, struct YUV_Capture *cap, IplImage *rgb)
{
    cap->y->imageData = avFrame->data[0];
    cap->cb_half->imageData = avFrame->data[1];
    cap->cr_half->imageData = avFrame->data[2];
    
    cvResize(cap->cb_half, cap->cb, CV_INTER_CUBIC);
    cvResize(cap->cr_half, cap->cr, CV_INTER_CUBIC);
    cvMerge(cap->y, cap->cr, cap->cb, NULL, cap->ycrcb);
    
    cvCvtColor(cap->ycrcb, rgb, CV_YUV2RGB); 
    cvShowImage("Bebop OpenCV Stream", rgb);
    cvWaitKey(5);
    //cvSaveImage("opencv01.png", rgb, 0);
    
    return 1;
}

int writeAscii(uint8_t * frame, int size)
{
    int i,j;
    IplImage *RGBImage = cvCreateImage(cvSize(640,368), IPL_DEPTH_8U, 3);
    
    for( i = 0, j=0; i < 640 * 368 * 3; i+=6, j+=4)
    {
        RGBImage->imageData[i] = frame[j] + frame[j+3]*((1 - 0.299)/0.615);
        RGBImage->imageData[i+1] = frame[j] - frame[j+1]*((0.114*(1-0.114))/(0.436*0.587)) - frame[j+3]*((0.299*(1 - 0.299))/(0.615*0.587));
        RGBImage->imageData[i+2] = frame[j] + frame[j+1]*((1 - 0.114)/0.436);
        RGBImage->imageData[i+3] = frame[j+2] + frame[j+3]*((1 - 0.299)/0.615);
        RGBImage->imageData[i+4] = frame[j+2] - frame[j+1]*((0.114*(1-0.114))/(0.436*0.587)) - frame[j+3]*((0.299*(1 - 0.299))/(0.615*0.587));
        RGBImage->imageData[i+5] = frame[j+2] + frame[j+1]*((1 - 0.114)/0.436);
    }
    
    cvShowImage("Bebop OpenCV Stream", RGBImage);
    cvWaitKey(5);
    
//    for(i=0;i<size;i++)
//        fprintf(frameProcessing,"%u\n", frame[i]);
//    fclose(frameProcessing);
    
}