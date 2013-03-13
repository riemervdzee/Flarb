#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
using namespace std;

#include "flarb_rgbcamera/cCamera.h"

#define JPEG_QUALITY 70


// TODO Get this in the cCamera class, problem is the rogue functions below
// (my_init_destination and etc.)
#define BLOCK_SIZE 1280 * 800 * 3
std::vector<uint8_t> my_buffer;
jpeg_destination_mgr dest;



/**
  Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV

  \param width width of image
  \param height height of image
  \param src source
  \param dst destination
*/
static void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int line, column;
	unsigned char *py, *pu, *pv;
	//unsigned char *tmp = dst;
	unsigned char dst_r, dst_g, dst_b;

	/* In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr. 
	Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. */
	py = src;
	pu = src + 1;
	pv = src + 3;

	#define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

	for (line = 0; line < height; ++line) {
		for (column = 0; column < width; ++column) {
			dst_r = CLIP((double)*py + 1.402*((double)*pv-128.0));
			dst_g = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));      
			dst_b = CLIP((double)*py + 1.772*((double)*pu-128.0));
			
			*dst++ = dst_r;
			*dst++ = dst_g;
			*dst++ = dst_b;

			// increase py every time
			py += 2;

			// increase pu,pv every second time
			if ((column & 1)==1) {
				pu += 4;
				pv += 4;
			}
		}
	}

	#undef CLIP
}

/**
  Print error message and terminate programm with EXIT_FAILURE return code.
  \param s error message to print
*/
static void errno_exit(const char* s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit(EXIT_FAILURE);
}

/**
  Do ioctl and retry if error was EINTR ("A signal was caught during the ioctl() operation."). Parameters are the same as on ioctl.

  \param fd file descriptor
  \param request request
  \param argp argument
  \returns result from ioctl
*/
static int ioctl_retry(int fd, int request, void* argp)
{
	int r;

	do r = ioctl( fd, request, argp);
	while ( -1 == r && EINTR == errno);

	return r;
}

/* TODO search alternative... */
void my_init_destination(j_compress_ptr cinfo)
{
    my_buffer.resize(BLOCK_SIZE);
    cinfo->dest->next_output_byte = &my_buffer[0];
    cinfo->dest->free_in_buffer = my_buffer.size();
}

boolean my_empty_output_buffer(j_compress_ptr cinfo)
{
    size_t oldsize = my_buffer.size();
    my_buffer.resize(oldsize + BLOCK_SIZE);
    cinfo->dest->next_output_byte = &my_buffer[oldsize];
    cinfo->dest->free_in_buffer = my_buffer.size() - oldsize;
    return true;
}

void my_term_destination(j_compress_ptr cinfo)
{
    my_buffer.resize(my_buffer.size() - cinfo->dest->free_in_buffer);
}
/* TODO get rid of above*/

/**
  Write image to jpeg file.

  \param img image to write
*/
void cCamera::jpegWrite(unsigned char* img)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	JSAMPROW row_pointer[1];

	// create jpeg data
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);

	// Set dest info
	cinfo.dest = &dest;
	cinfo.dest->init_destination    = &my_init_destination;
	cinfo.dest->empty_output_buffer = &my_empty_output_buffer;
	cinfo.dest->term_destination    = &my_term_destination;

	// set image parameters
	cinfo.image_width = _width;
	cinfo.image_height = _height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;

	// set jpeg compression parameters to default
	jpeg_set_defaults(&cinfo);

	// and then adjust quality setting
	jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);

	// start compress 
	jpeg_start_compress(&cinfo, TRUE);

	// feed data
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	// finish compression
	jpeg_finish_compress(&cinfo);

	// destroy jpeg data
	jpeg_destroy_compress(&cinfo);
}

/**
  read single frame
*/

int cCamera::frameRead(void)
{
	struct v4l2_buffer buf = {};

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == ioctl_retry(_fileHandle, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
			case EAGAIN:
			return 0;

			case EIO:
			// Could ignore EIO, see spec

			// fall through
			default:
			errno_exit("VIDIOC_DQBUF");
		}
	}

	assert (buf.index < _mmBuffersAmount);

	unsigned char* src = (unsigned char*)_mmBuffers[buf.index].start;
	//unsigned char* dst = (unsigned char*)malloc(width*height*3*sizeof(char));

	// convert from YUV422 to RGB888
	YUV422toRGB888( _width, _height, src, _tmp_buffer);

	// write jpeg to mem
	jpegWrite( _tmp_buffer);

	if (-1 == ioctl_retry( _fileHandle, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");


	return 1;
}

/** 
  mainloop: read frames and process them
*/
void cCamera::mainLoop(void)
{
	unsigned int count;

	count = 1;

	while (count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(_fileHandle, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select( _fileHandle + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;

				errno_exit("select");
			}

			if (0 == r) {
				fprintf (stderr, "select timeout\n");
				exit(EXIT_FAILURE);
			}

			if (frameRead())
				break;

			/* EAGAIN - continue select loop. */
		}
	}
}

/**
  initialize device
*/
void cCamera::deviceInit(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap = {};
	struct v4l2_crop crop;
	struct v4l2_format fmt = {};
	unsigned int min;

	if (-1 == ioctl_retry( _fileHandle, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",_devicePath);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n",_devicePath);
		exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming i/o\n",_devicePath);
		exit(EXIT_FAILURE);
	}


	/* Select video input, video standard and tune here. */
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == ioctl_retry( _fileHandle, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == ioctl_retry( _fileHandle, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
				case EINVAL:
				/* Cropping not supported. */
				break;
				default:
				/* Errors ignored. */
				break;
			}
		}
	} // Errors ignored

	// v4l2_format
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = _width;
	fmt.fmt.pix.height      = _height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;

	if (-1 == ioctl_retry( _fileHandle, VIDIOC_S_FMT, &fmt))
	errno_exit("VIDIOC_S_FMT");

	/* Note VIDIOC_S_FMT may change width and height. */
	if (_width != fmt.fmt.pix.width) {
		_width  = fmt.fmt.pix.width;
		fprintf(stderr,"Image width set to %i by device %s.\n",_width,_devicePath);
	}

	if (_height != fmt.fmt.pix.height) {
		_height  = fmt.fmt.pix.height;
		fprintf(stderr,"Image height set to %i by device %s.\n",_height,_devicePath);
	}

	// Now we got the width and height, allocate the rgb buffer
	_tmp_buffer = (unsigned char*) malloc( _width * _height * 3 * sizeof( char));

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;

	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	// Get mmap buffers
	struct v4l2_requestbuffers req = {};

	req.count               = 4;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if (-1 == ioctl_retry( _fileHandle, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support memory mapping\n", _devicePath);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", _devicePath);
		exit(EXIT_FAILURE);
	}

	_mmBuffers = (mmap_buffer*) calloc(req.count, sizeof(*_mmBuffers));

	if (!_mmBuffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (_mmBuffersAmount = 0; _mmBuffersAmount < req.count; ++_mmBuffersAmount) {
		struct v4l2_buffer buf = {};

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = _mmBuffersAmount;

		if (-1 == ioctl_retry( _fileHandle, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		_mmBuffers[_mmBuffersAmount].length = buf.length;
		_mmBuffers[_mmBuffersAmount].start =
		mmap ( NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fileHandle, buf.m.offset);

		if (MAP_FAILED == _mmBuffers[_mmBuffersAmount].start)
			errno_exit("mmap");
	}
}

// Functions executed at the beginning and end of the Application
bool cCamera::Create( const char* DevicePath, int width, int height)
{
	// Init values
	_devicePath = DevicePath;
	_width  = width;
	_height = height;

	// Open device handle
	_fileHandle = open( _devicePath, O_RDWR | O_NONBLOCK, 0);
	if ( _fileHandle < 0) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", _devicePath, errno, strerror (errno));
		exit(EXIT_FAILURE);
	}

	// Init device
	deviceInit();


	// Set buffers to mmap capturing
	for (unsigned int i = 0; i < _mmBuffersAmount; ++i) {
		struct v4l2_buffer buf = {};

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		if (-1 == ioctl_retry( _fileHandle, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
	}

	// start capturing
	// (TODO, we might be able to implement stop/resume on this?)
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl_retry( _fileHandle, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");

	return true;
}
void cCamera::Destroy()
{
	// stop capturing
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl_retry( _fileHandle, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");

	// Free buffers
	for ( unsigned int i = 0; i < _mmBuffersAmount; ++i)
		if (-1 == munmap ( _mmBuffers[i].start, _mmBuffers[i].length))
			errno_exit("munmap");

	free( _mmBuffers);

	if( _tmp_buffer != NULL)
		free( _tmp_buffer);

	// Close device handle
	if (-1 == close (_fileHandle))
		errno_exit("close");

	_fileHandle = -1;
}

// Get an image from the camera
sensor_msgs::CompressedImage cCamera::getImage()
{
	// Executing this once, guarantees a buffer in my_buffer
	mainLoop();

	sensor_msgs::CompressedImage msg;

	// TODO shizzle me nizzle
	//msg.header.timestamp = 0;
	msg.header.frame_id  = "1";
	msg.format = "jpeg";

	msg.data = my_buffer;

	return msg;
}
