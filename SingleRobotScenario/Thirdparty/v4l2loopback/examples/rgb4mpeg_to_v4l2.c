/*
 * Copy a YUV4MPEG stream to a v4l2 output device.
 * The stream is read from standard input.
 * The device can be specified as argument; it defaults to /dev/video0.
 *
 * Example using mplayer as a producer for the v4l2loopback driver:
 *
 * $ mkfifo /tmp/pipe
 * $ ./yuv4mpeg_to_v4l2 < /tmp/pipe &
 * $ mplayer movie.mp4 -vo yuv4mpeg:file=/tmp/pipe
 *
 * Copyright (C) 2011  Eric C. Cooper <ecc@cmu.edu>
 * Released under the GNU General Public License
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

//#include <opencv/cv.h>
//#include "yuv.h"


char *prog;

char *device;
int dev_fd;

int frame_width=640;
int frame_height=368;
int frame_bytes=353280;
int rgb_frame_bytes=706560;

void
usage(void)
{
	fprintf(stderr, "Usage: %s [/dev/videoN]\n", prog);
	exit(1);
}

void
process_args(int argc, char **argv)
{
	prog = argv[0];
	switch (argc) {
	case 1:
		device = "/dev/video0";
		break;
	case 2:
		device = argv[1];
		break;
	default:
		usage();
		break;
	}
}

void
sysfail(char *msg)
{
	perror(msg);
	exit(1);
}

void
fail(char *msg)
{
	fprintf(stderr, "%s: %s\n", prog, msg);
	exit(1);
}

void
copy_frames(void)
{
	char *frame;

	frame = malloc(rgb_frame_bytes);
	if (frame == NULL) fail("cannot malloc frame");

	//int correctFrame = 0;
	//int corruptFrame = 0;
    	while (1) 
    	{
		int size_c = fread(frame, 1, rgb_frame_bytes, stdin);
		//printf("%d\n",size_c);
		//if(size_c == rgb_frame_bytes)
		//{
		//	correctFrame++;
		//	printf("%d corrupt frame\n Correct Frame: %d\n",corruptFrame, correctFrame);
		//	corruptFrame = 0;
		//}
		//else
		//if(size_c != 0)
		//{
		//	printf("%d correct frame\n size = %d\n",correctFrame,size_c);
		//	correctFrame = 0;
		//	corruptFrame++;
		//}
			
		if ( size_c != rgb_frame_bytes) {
      			free(frame);
			fail("malformed frame");
    		}
		else 
		{
			//TODO: convert yuv frame to rgb -> conversion done in BebopDroneDecodeStream, now reading RGB24 frames from fifo
		//if(size_c != 0)
			if (write(dev_fd, frame, rgb_frame_bytes) != rgb_frame_bytes) {
      				free(frame);
				sysfail("write");
			}
    		}
		//usleep(300);
	}

  free(frame);
}

#define vidioc(op, arg) \
	if (ioctl(dev_fd, VIDIOC_##op, arg) == -1) \
		sysfail(#op); \
	else

void
open_video(void)
{
	struct v4l2_format v;

	dev_fd = open(device, O_RDWR);
	if (dev_fd == -1) sysfail(device);
	v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	vidioc(G_FMT, &v);
	v.fmt.pix.width = frame_width;
	v.fmt.pix.height = frame_height;
	v.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	v.fmt.pix.sizeimage = rgb_frame_bytes;
	vidioc(S_FMT, &v);
}


int
main(int argc, char **argv)
{
	process_args(argc, argv);
	open_video();
	copy_frames();
	return 0;
}
