/*
 * https://github.com/chelyaev/opencv-yuv
 */

#ifndef YUV_H
#define YUV_H

#include <stdio.h>
#include <stdint.h>
#include <opencv/cv.h>
#include <libavcodec/avcodec.h>

/**
 * Is returned by the public API functions.
 */
enum YUV_ReturnValue
{
    YUV_OK = 0,             /**< Function terminated correctly >*/
    YUV_PARAMETER_ERROR,    /**< The parameters passed to the function were
                                 incorrect */
    YUV_OUT_OF_MEMORY,      /**< The function ran out of memory >*/
    YUV_IO_ERROR,           /**< The function encountered an error reading
                                 data from disk, or reached EOF earlier than 
                                 was expected (premature end of frame) >*/
    YUV_EOF                 /**< The function reached EOF (not premature) >*/
};

/**
 * Used to capture YUV frames from a file on disk.
 *
 * @see YUV_init
 * @see YUV_read
 */
struct YUV_Capture
{
    FILE *fin;              /**< The input file pointer >*/
    size_t width;           /**< The width of the frame, in pixels >*/
    size_t height;          /**< The height of the frame, in pixels >*/
    
    IplImage *y;            /**< Used internally. >*/
    IplImage *cb;           /**< Used internally. >*/
    IplImage *cr;           /**< Used internally. >*/
    IplImage *cb_half;      /**< Used internally. >*/
    IplImage *cr_half;      /**< Used internally. >*/
    IplImage *ycrcb;        /**< The most-recently image (width x height, 24 
                                 bit).  Stored in YCrCb order. >*/
};

/**
 * Initialize a YUV_Capture instance.  Allocates memory used for internal
 * processing.  After initialization, frames can be read using YUV_read.
 *
 * @param[in] fin   The file to read from.  If the function succeeds, the
 *                  capture will own this pointer from that point onwards.
 * @param[in] w     The frame width, in pixels.
 * @param[in] h     The frame height, in pixels.
 * @param[out] out  The instance to initialize.
 */
enum YUV_ReturnValue YUV_init(FILE *fin, size_t w, size_t h, struct YUV_Capture *out);

/**
 * Read a single frame from a previously-instantiated YUV_Capture instance.
 * 
 * @param[in,out] cap   The capture to read from.  The result frame is stored 
 *                      in cap-ycrcb.
 *
 * @returns YUV_OK if it's OK to keep reading.  YUV_EOF if EOF was reached and
 * no further reading is possible.  The capture should be cleaned up.
 * @see YUV_cleanup
 */
enum YUV_ReturnValue YUV_read(struct YUV_Capture *cap, int YLineSize);

/**
 * Deallocate the memory allocated during initialization.
 */
void YUV_cleanup(struct YUV_Capture *cap, IplImage *rgb);
int yuv2rgb(AVFrame *pict, int width, int height, struct YUV_Capture cap, IplImage *rgb);
int copyYUVdata(AVFrame* avFrame, struct YUV_Capture *cap, IplImage *rgb);
int writeAscii(uint8_t * frame, int size);

#endif
