/*
Compile: gcc scan_mt_skip.c -ldmtx -lv4l2 -lm -pthread -o scan_skip
Data-matrix decoding module.
Takes one argument: the video device
eg. ./scan_mt_skip /dev/video0

Multi-threaded version of the data matrix decoding module.

Strategy:
-> The main thread initializes NUM_THREADS threads, and they are set to wait until signalled
-> The main thread now starts reading frames. After it reads one frame, it looks for an idle thread.
-> As soon as an idle thread is found, it is signalled to wake up and process the frame. Once the processing is done,
it goes back to sleep and waits for another signal.
-> This keeps repeating.

*/

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <time.h>
#include <linux/videodev2.h>
#include <stdlib.h>
#include <math.h>
#include <dmtx.h>
#include <time.h>
#include <pthread.h>

#define USE_LIBV4L

#ifdef USE_LIBV4L
#include <libv4l2.h>
#else
#include <linux/ioctl.h>
#include <sys/mman.h>
#define v4l2_open open
#define v4l2_close close
#define v4l2_ioctl ioctl
#define v4l2_read read
#define v4l2_mmap mmap
#define v4l2_munmap munmap
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define FRAME_WIDTH 			320
#define FRAME_HEIGHT 			240
#define DMTX_REGION_TIMEOUT 	30
#define NUM_THREADS 			5
#define EDGE_THRESHOLD          70
#define BARCODE_THRESHOLD       3000


// Thread variables
pthread_mutex_t 	wait_mutex[NUM_THREADS];
pthread_cond_t 		wait_cv[NUM_THREADS];
pthread_mutex_t     print_mutex;

// The data that is passed to threads
typedef struct t_data {
	char **buf;
	pthread_mutex_t *wait_mutex;
	pthread_cond_t *wait_cv;
} thread_data;


void* myDmtxDecode(void *th_data);

//  =====================================================================
int lErrno()
{
  return errno;
}

//  =====================================================================
char* lError()
{
  return strerror(errno);
}

//  =====================================================================
char* lbool2str(int b)
{
  if (b)
    return "True";
  
  return "False";
}

//  =====================================================================
int lBufferDiff(char* b1, char* b2, int size, int format)
{
  int i     = 0;
  int count = 0;
  
  for (i = 0; i < size; i++)
  {
    if (b1[i] != b2[i])
    {
      count++;
    }
  }

  return count;
}

//  =====================================================================
int lSetFPS(int fd, int numerator, int denominator) {
    struct v4l2_streamparm setfps;
    memset(&setfps, 0, sizeof(struct v4l2_streamparm));
    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = numerator;
    setfps.parm.capture.timeperframe.denominator = denominator;
    return v4l2_ioctl(fd, VIDIOC_S_PARM, &setfps); 
}

int lOpen(char* dev)
{
  int fd  = v4l2_open(dev, O_RDWR);
  return fd;
}

//  =====================================================================
int lClose(int fd)
{
  v4l2_close(fd);
  return 0;
}

//  =====================================================================
int lQueryCaps(int fd, struct v4l2_capability* caps)
{
  return v4l2_ioctl(fd, VIDIOC_QUERYCAP, caps);
}

//  =====================================================================
int lEnumInput(int fd, struct v4l2_input* input)
{
  return v4l2_ioctl(fd, VIDIOC_ENUMINPUT, input);
}

//  =====================================================================
int lEnumFormat(int fd, struct v4l2_fmtdesc* input)
{
  return v4l2_ioctl(fd, VIDIOC_ENUM_FMT, input);
}

//  =====================================================================
int lSetInput(int fd, int input)
{
  return v4l2_ioctl(fd, VIDIOC_S_INPUT, &input );
}

//  =====================================================================
int lSetStandard(int fd, v4l2_std_id std)
{
  return v4l2_ioctl(fd, VIDIOC_S_STD, &std );
}

//  =====================================================================
v4l2_std_id letStandard(int fd)
{
        v4l2_std_id std = -1;
  int result = v4l2_ioctl(fd, VIDIOC_G_STD, &std );
  if (result == -1)
    return -1;
  return std;
}

//  =====================================================================
int lGetFormat(int fd, struct v4l2_format* f)
{
  return v4l2_ioctl(fd, VIDIOC_G_FMT, f);
}

//  =====================================================================
int lSetFormat(int fd, struct v4l2_format* f)
{
  return v4l2_ioctl(fd, VIDIOC_S_FMT, f);
}

//  =====================================================================
int lRead(int fd, char* buf, int bufsize)
{
  return v4l2_read(fd, buf, bufsize);
}

//  =====================================================================
int lRequestBuffers(int fd, struct v4l2_requestbuffers* f)
{
  return v4l2_ioctl(fd, VIDIOC_REQBUFS, f);
}

//  =====================================================================
int lQueryBuf(int fd, struct v4l2_buffer* f)
{
  return v4l2_ioctl(fd, VIDIOC_QUERYBUF, f);
}

//  =====================================================================
int lQueue(int fd, struct v4l2_buffer* f)
{
  return v4l2_ioctl(fd, VIDIOC_QBUF, f);
}

//  =====================================================================
int lDequeue(int fd, struct v4l2_buffer* f)
{
  return v4l2_ioctl(fd, VIDIOC_DQBUF, f);
}

//  =====================================================================
int lStreamOn(int fd, int type)
{
  return v4l2_ioctl(fd, VIDIOC_STREAMON, &type);
}

//  =====================================================================
int lStreamOff(int fd, int type)
{
  return v4l2_ioctl(fd, VIDIOC_STREAMOFF, &type);
}

//  =====================================================================
void* lMMap(int fd, size_t size, off_t offset)
{
  return v4l2_mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
}

//  =====================================================================
int lMUnmap(void* start, size_t length)
{
  return v4l2_munmap(start, length);
}

int mayHaveBarcode(char *pxl)
{
    /**
    * Returns 1 if the frame may have a barcode, 0 otherwise
    * This O(n) check is essential to quickly skip frames that do not contain
    * a barcode.
    */

    int i;
    int pix1, pix2;
    int row_size = FRAME_WIDTH * 3;
    int score = 0;

    for(i = 0; i < FRAME_WIDTH * FRAME_HEIGHT * 3; i++)
    {
        if( (i+3) % row_size == 0)
            continue;
        pix1 = (pxl[i] + pxl[i+1] + pxl[i+2]) / 3;
        pix2 = (pxl[i+3] + pxl[i+4] + pxl[i+5]) / 3;

        if(abs(pix1 - pix2) >= EDGE_THRESHOLD)
            score++;
    }

    if(score >= BARCODE_THRESHOLD)
        return 1;
    else return 0;
}

int main(int argc, char *argv[])
{
	int 		fd, res, i, rc, count, index;
	struct  	v4l2_format fmt;

	// Each thread has access to its own frame buffer, to avoid unnecessary mutex complications 
	char 		*buf[NUM_THREADS];				
	pthread_t 	threads[NUM_THREADS];

	pthread_mutex_init(&print_mutex, NULL);

    // Open the video device and set some properties
    fd = lOpen(argv[1]);
    lSetFPS(fd, 1, 30);
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = FRAME_WIDTH;
    fmt.fmt.pix.height      = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    res = lSetFormat(fd, &fmt);
    
    // Initialize the threads
	for(i = 0; i < NUM_THREADS; i++)
	{
		// Initialize the mutex and condition variables
		pthread_mutex_init(&wait_mutex[i], NULL);
    	pthread_cond_init (&wait_cv[i], NULL);

    	// Set buffer to NULL
    	//buf[i] = NULL;

        // Pre-allocate memory to buffer
        buf[i] = (char *) calloc(fmt.fmt.pix.sizeimage, sizeof(char));

    	// Set up the thread data structure
    	thread_data *temp = (thread_data *) malloc(sizeof(thread_data));
    	temp-> buf = &buf[i];
    	temp-> wait_mutex = &wait_mutex[i];
    	temp-> wait_cv = &wait_cv[i];

    	// Initialize thread
    	rc = pthread_create(&threads[i], NULL, myDmtxDecode, (void *) temp);
    	if (rc)
        {
            printf("ERROR; return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
	}

    // Sleep for a bit. Let the threads initialize
    for(i = 0; i < 20000; i++) ;

    // Start capturing frames and decoding!
    count = 0;
    while(1)
    {
        // First, look for an idle thread
        while(1)
        {
            // Keep cycling around until you hit an idle thread
            index = count % NUM_THREADS;
            rc = pthread_mutex_trylock(&wait_mutex[index]);
            if(rc == 0)
            {
                // Lock acquired. Read frame and signal corresponding thread
                
                // Allocate memory to buffer
                // buf[index] = (char *) malloc(fmt.fmt.pix.sizeimage * sizeof(char));
                
                // Read frame into buffer
                lRead(fd, buf[index], fmt.fmt.pix.sizeimage);

                // Signal corresponding thread to grab frame and decode it
                pthread_cond_signal(&wait_cv[index]);
                pthread_mutex_unlock(&wait_mutex[index]);

                count = index + 1;
                
                break;
            }
            else count++;
        }
    }
}

void* myDmtxDecode(void *th_data)
{
	thread_data *tdata = (thread_data *) th_data;

    // Go to sleep
    pthread_mutex_lock(tdata-> wait_mutex);
    pthread_cond_wait(tdata-> wait_cv, tdata-> wait_mutex);

    DmtxVector2     p00, p10, p11, p01;
    DmtxTime        timeout;
    DmtxImage       *img;
    DmtxDecode      *dec;
    DmtxRegion      *reg;
    DmtxMessage     *msg;
    double          rotate;
    int             rotateInt;
    int             height;
    char            *pxl;

    // Keep waiting for signals to decode
    while(1)
    {
        pxl = *(tdata-> buf);
        if(pxl != NULL)
        {
            // Rough check for barcode
            // Allows us to quickly skip empty frames
            if(mayHaveBarcode(pxl))
            {
                // Create DmtxImage
                img = dmtxImageCreate(pxl, FRAME_WIDTH, FRAME_HEIGHT, DmtxPack24bppRGB);
                if(img != NULL)
                {
                    // Set image flip to none
                    dmtxImageSetProp(img, DmtxPropImageFlip, DmtxFlipNone);

                    // Create decode structure
                    dec = dmtxDecodeCreate(img, 1);
                    if(dec != NULL)
                    {
                        // Set timeout
                        timeout = dmtxTimeAdd(dmtxTimeNow(), DMTX_REGION_TIMEOUT);

                        // Find datamatrix region
                        reg = dmtxRegionFindNext(dec, &timeout);
                        if(reg != NULL)
                        {
                            // Decode message, if datamatrix is found in the frame
                            msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
                            if(msg != NULL)
                            {
                                // Locate corners
                                height = dmtxDecodeGetProp(dec, DmtxPropHeight);
                                p00.X = p00.Y = p10.Y = p01.X = 0.0;
                                p10.X = p01.Y = p11.X = p11.Y = 1.0;
                                dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
                                dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
                                dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
                                dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);

                                // Determine orientation
                                rotate = (2 * M_PI) + atan2(p10.Y - p00.Y, p10.X - p00.X);
                                rotateInt = (int)(rotate * 180/M_PI + 0.5);
                                if(rotateInt >= 360)
                                    rotateInt -= 360;

                                // Print out message
                                // Only for debugging purposes
                                pthread_mutex_lock(&print_mutex);
                                printf("\nMessage: %s %d\n", msg->output, pxl[45]);
                                pthread_mutex_unlock(&print_mutex); 

                                dmtxMessageDestroy(&msg);
                            }
                            dmtxRegionDestroy(&reg);
                        }    
                        dmtxDecodeDestroy(&dec);
                    }    
                    dmtxImageDestroy(&img);
                }
            }
            // free(pxl);
        }
    
        // Go back to sleep
        pthread_cond_wait(tdata-> wait_cv, tdata-> wait_mutex);
    }
	
}
