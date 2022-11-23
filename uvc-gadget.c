/*
 * UVC gadget test application
 *
 * Copyright (C) 2010 Ideas on board SPRL <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#define _GNU_SOURCE

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include <libavformat/avformat.h>
#include <libavutil/avutil.h>

#include "uvc.h"

#define TRACE(fmt, ...) fprintf(stderr, "%s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)
//#define TRACE(fmt, ...)

#define V4L2_FOURCC_FMT "%c%c%c%c"
#define V4L2_FOURCC_ARG(fourcc) \
	(fourcc) & 0x7F, \
    ((fourcc) >> 8) & 0x7F, \
    ((fourcc) >> 16) & 0x7F, \
	((fourcc) >> 24) & 0x7F

int64_t monotonic_us(void)
{
    struct timespec ts;
    int status = clock_gettime(CLOCK_MONOTONIC, &ts);

    if(0 != status)
    {
        TRACE("error %s", strerror(errno));
        assert(0);
    }

    return ts.tv_sec * INT64_C(1000000) + ts.tv_nsec / INT64_C(1000);
}

uint64_t timestamp(void)
{
    struct timeval tv;

    if(0 != gettimeofday(&tv, NULL))
    {
        TRACE("error %s", strerror(errno));
        return 0;
    }
    return UINT64_C(1000000) * tv.tv_sec + tv.tv_usec;
}
#if 0
void dump_to_file(const char *data, size_t size, const char *ext)
{
    static int n = 0;
    char buf[255];

    snprintf(buf, sizeof(buf), "dump%d.%s", n, ext);
    FILE *f = fopen(buf, "wb");
    fwrite(data, size, 1, f);
    ++n;
    fclose(f);

    snprintf(buf, 255, "dump.%s", ext);
	FILE *f2 = fopen(buf, "ab");
    fwrite(data, size, 1, f2);
	fclose(f2);
}
#endif

/* Enable debug prints. */
#define ENABLE_BUFFER_DEBUG
#define ENABLE_USB_REQUEST_DEBUG

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

#define clamp(val, min, max)                                                                                           \
    ({                                                                                                                 \
        typeof(val) __val = (val);                                                                                     \
        typeof(min) __min = (min);                                                                                     \
        typeof(max) __max = (max);                                                                                     \
        (void)(&__val == &__min);                                                                                      \
        (void)(&__val == &__max);                                                                                      \
        __val = __val < __min ? __min : __val;                                                                         \
        __val > __max ? __max : __val;                                                                                 \
    })

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))
#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, ((x) >> 24) & 0xff

/*
 * The UVC webcam gadget kernel driver (g_webcam.ko) supports changing
 * the Brightness attribute of the Processing Unit (PU). by default. If
 * the underlying video capture device supports changing the Brightness
 * attribute of the image being acquired (like the Virtual Video, VIVI
 * driver), then we should route this UVC request to the respective
 * video capture device.
 *
 * Incase, there is no actual video capture device associated with the
 * UVC gadget and we wish to use this application as the final
 * destination of the UVC specific requests then we should return
 * pre-cooked (static) responses to GET_CUR(BRIGHTNESS) and
 * SET_CUR(BRIGHTNESS) commands to keep command verifier test tools like
 * UVC class specific test suite of USBCV, happy.
 *
 * Note that the values taken below are in sync with the VIVI driver and
 * must be changed for your specific video capture device. These values
 * also work well in case there in no actual video capture device.
 */
#define PU_BRIGHTNESS_MIN_VAL 0
#define PU_BRIGHTNESS_MAX_VAL 255
#define PU_BRIGHTNESS_STEP_SIZE 1
#define PU_BRIGHTNESS_DEFAULT_VAL 127

/* ---------------------------------------------------------------------------
 * Generic stuff
 */

/* IO methods supported */
enum io_method {
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

/* Buffer representing one video frame */
struct buffer {
    struct v4l2_buffer buf;
    void *start;
    size_t length;
};

/* ---------------------------------------------------------------------------
 * UVC specific stuff
 */

struct uvc_frame_info {
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info {
    unsigned int fcc;
    const struct uvc_frame_info *frames;
};

static const struct uvc_frame_info uvc_frames[] = {
    {480, 640, {333333, 0}},
    {1280, 720, {333333, 0}},
    {1920, 1080, {333333, 0}},
    {2160, 3840, {333333, 0}},
    {0, 0, {0}}
};

static const struct uvc_format_info uvc_formats[] = {
    {V4L2_PIX_FMT_YUYV, uvc_frames},
    {V4L2_PIX_FMT_H264, uvc_frames},
    {V4L2_PIX_FMT_MJPEG, uvc_frames}
};

/* ---------------------------------------------------------------------------
 * V4L2 and UVC device instances
 */

/* Represents a V4L2 based video capture device */
struct v4l2_device {
    /* v4l2 device specific */
    int v4l2_fd;
    int is_streaming;
    char *v4l2_devname;

    /* v4l2 buffer specific */
    enum io_method io;
    struct buffer *mem;
    int nbufs;

    /* v4l2 buffer queue and dequeue counters */
    long long int qbuf_count;
    long long int dqbuf_count;
    uint64_t qbuf_ts;
    uint64_t dqbuf_ts;

    /* uvc device hook */
    struct uvc_device *udev;
};

/* Represents a UVC based video output device */
struct uvc_device {
    /* uvc device specific */
    int uvc_fd;
    int is_streaming;
    int run_standalone;
    char *uvc_devname;

    /* uvc control request specific */

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;
    int control;
    struct uvc_request_data request_error_code;
    unsigned int brightness_val;

    /* uvc buffer specific */
    enum io_method io;
    struct buffer *mem;
    struct buffer *dummy_buf;
    unsigned int nbufs;
    unsigned int fcc;
    unsigned int width;
    unsigned int height;

    unsigned int bulk;
    uint8_t color;
    AVFormatContext *av_ctx;
    AVPacket *av_packet;
    int64_t time_offset_us;
    const char *video_filename;

    /* USB speed specific */
    int mult;
    int burst;
    int maxpkt;
    enum usb_device_speed speed;

    /* uvc specific flags */
    int first_buffer_queued;
    int uvc_shutdown_requested;

    /* uvc buffer queue and dequeue counters */
    unsigned long long int qbuf_count;
    unsigned long long int dqbuf_count;
    uint64_t qbuf_ts;
    uint64_t dqbuf_ts;

    /* v4l2 device hook */
    struct v4l2_device *vdev;
};

/* forward declarations */
static int uvc_video_stream(struct uvc_device *dev, int enable);
static int video_next_frame(struct uvc_device *dev);


static int64_t avpacket_pts(const struct uvc_device *dev)
{
    assert(dev->av_ctx);
    assert(dev->av_packet);

    const AVStream *stream = dev->av_ctx->streams[dev->av_packet->stream_index];
    const AVRational time_base = stream->time_base;

    // make 30fps default if PTS is not provided
    if(AV_NOPTS_VALUE == dev->av_packet->pts) return dev->qbuf_count * 33333;
    return
        av_rescale_q(
            dev->av_packet->pts,
            time_base, (AVRational){1, 1000000 /* us/s */});
}
/* ---------------------------------------------------------------------------
 * V4L2 streaming related
 */

static int v4l2_uninit_device(struct v4l2_device *dev)
{
    TRACE("");

    int i;
    int ret;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        for (i = 0; i < dev->nbufs; ++i) {
            ret = munmap(dev->mem[i].start, dev->mem[i].length);
            if (ret < 0) {
                TRACE("V4L2: munmap failed");
                return ret;
            }
        }

        free(dev->mem);
        break;

    case IO_METHOD_USERPTR:
    default:
        break;
    }

    return 0;
}

static int v4l2_reqbufs_mmap(struct v4l2_device *dev, int nbufs)
{
    TRACE("");
    struct v4l2_requestbuffers req;
    int i = 0;
    int ret;

    CLEAR(req);

    req.count = nbufs;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(dev->v4l2_fd, VIDIOC_REQBUFS, &req);
    if (ret < 0) {
        if (ret == -EINVAL)
            TRACE("V4L2: does not support memory mapping");
        else
            TRACE("V4L2: VIDIOC_REQBUFS error %s (%d).", strerror(errno), errno);
        goto err;
    }

    if (!req.count)
        return 0;

    if (req.count < 2) {
        TRACE("V4L2: Insufficient buffer memory.");
        ret = -EINVAL;
        goto err;
    }

    /* Map the buffers. */
    dev->mem = calloc(req.count, sizeof dev->mem[0]);
    if (!dev->mem) {
        TRACE("V4L2: Out of memory");
        ret = -ENOMEM;
        goto err;
    }

    for (i = 0; i < (int)req.count; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        ret = ioctl(dev->v4l2_fd, VIDIOC_QUERYBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            TRACE(
                "V4L2: VIDIOC_QUERYBUF failed for buf %d: "
                "%s (%d).",
                i, strerror(errno), errno);
            ret = -EINVAL;
            goto err_free;
        }

        dev->mem[i].start =
            mmap(NULL /* start anywhere */, dev->mem[i].buf.length, PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */, dev->v4l2_fd, dev->mem[i].buf.m.offset);

        if (MAP_FAILED == dev->mem[i].start) {
            TRACE("V4L2: Unable to map buffer %u: %s (%d).", i, strerror(errno), errno);
            dev->mem[i].length = 0;
            ret = -EINVAL;
            goto err_free;
        }

        dev->mem[i].length = dev->mem[i].buf.length;
        TRACE("V4L2: Buffer %u mapped at address %p.", i, dev->mem[i].start);
    }

    dev->nbufs = req.count;
    TRACE("V4L2: %u buffers allocated.", req.count);

    return 0;

err_free:
    free(dev->mem);
err:
    return ret;
}

static int v4l2_reqbufs_userptr(struct v4l2_device *dev, int nbufs)
{
    TRACE("");
    struct v4l2_requestbuffers req;
    int ret;

    CLEAR(req);

    req.count = nbufs;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    ret = ioctl(dev->v4l2_fd, VIDIOC_REQBUFS, &req);
    if (ret < 0) {
        if (ret == -EINVAL)
            TRACE("V4L2: does not support user pointer i/o");
        else
            TRACE("V4L2: VIDIOC_REQBUFS error %s (%d).", strerror(errno), errno);
        return ret;
    }

    dev->nbufs = req.count;
    TRACE("V4L2: %u buffers allocated.", req.count);

    return 0;
}

static int v4l2_reqbufs(struct v4l2_device *dev, int nbufs)
{
    TRACE("nbufs: %d io: %d", nbufs, dev->io);
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        ret = v4l2_reqbufs_mmap(dev, nbufs);
        break;

    case IO_METHOD_USERPTR:
        ret = v4l2_reqbufs_userptr(dev, nbufs);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int v4l2_qbuf_mmap(struct v4l2_device *dev)
{
    TRACE("");
    int i;
    int ret;

    for (i = 0; i < dev->nbufs; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        ret = ioctl(dev->v4l2_fd, VIDIOC_QBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            TRACE("V4L2: VIDIOC_QBUF failed : %s (%d).", strerror(errno), errno);
            return ret;
        }

        dev->qbuf_count++;
    }

    return 0;
}

static int v4l2_qbuf(struct v4l2_device *dev)
{
    TRACE("");
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        ret = v4l2_qbuf_mmap(dev);
        break;

    case IO_METHOD_USERPTR:
        /* Empty. */
        ret = 0;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int v4l2_process_data(struct v4l2_device *dev)
{
    int ret;
    struct v4l2_buffer vbuf;
    struct v4l2_buffer ubuf;

    /* Return immediately if V4l2 streaming has not yet started. */
    if (!dev->is_streaming)
        return 0;

    if (dev->udev->first_buffer_queued)
        if (dev->dqbuf_count >= dev->qbuf_count)
            return 0;

    /* Dequeue spent buffer rom V4L2 domain. */
    CLEAR(vbuf);

    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    switch (dev->io) {
    case IO_METHOD_USERPTR:
        vbuf.memory = V4L2_MEMORY_USERPTR;
        break;

    case IO_METHOD_MMAP:
    default:
        vbuf.memory = V4L2_MEMORY_MMAP;
        break;
    }

    ret = ioctl(dev->v4l2_fd, VIDIOC_DQBUF, &vbuf);
    if (ret < 0) {
        TRACE("Unable to dequeue buffer: %s (%d).", strerror(errno), errno);
        return ret;
    }

    dev->dqbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
    {
        uint64_t ts = timestamp();

        uint64_t diff = ts - dev->dqbuf_ts;

        TRACE("%p Dequeueing buffer at V4L2 side = %d " "%" PRIu64 "ms",
              dev, vbuf.index, diff / 1000);
        dev->dqbuf_ts = ts;
    }
#endif

    /* Queue video buffer to UVC domain. */
    CLEAR(ubuf);

    ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    switch (dev->udev->io) {
    case IO_METHOD_MMAP:
        ubuf.memory = V4L2_MEMORY_MMAP;
        ubuf.length = vbuf.length;
        ubuf.index = vbuf.index;
        ubuf.bytesused = vbuf.bytesused;
        break;

    case IO_METHOD_USERPTR:
    default:
        ubuf.memory = V4L2_MEMORY_USERPTR;
        ubuf.m.userptr = (unsigned long)dev->mem[vbuf.index].start;
        ubuf.length = dev->mem[vbuf.index].length;
        ubuf.index = vbuf.index;
        ubuf.bytesused = vbuf.bytesused;
        break;
    }

    ret = ioctl(dev->udev->uvc_fd, VIDIOC_QBUF, &ubuf);
    if (ret < 0) {
        /* Check for a USB disconnect/shutdown event. */
        if (errno == ENODEV) {
            dev->udev->uvc_shutdown_requested = 1;
            TRACE(
                "UVC: Possible USB shutdown requested from "
                "Host, seen during VIDIOC_QBUF");
            return 0;
        } else {
            return ret;
        }
    }

    dev->udev->qbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
    {
        uint64_t ts = timestamp();

        uint64_t diff = ts - dev->qbuf_ts;

        TRACE("%p Queueing buffer at UVC side = %d " "%" PRIu64 "ms",
              dev, ubuf.index, diff / 1000);
        dev->qbuf_ts = ts;
    }
#endif

    if (!dev->udev->first_buffer_queued && !dev->udev->run_standalone) {
        uvc_video_stream(dev->udev, 1);
        dev->udev->first_buffer_queued = 1;
        dev->udev->is_streaming = 1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * V4L2 generic stuff
 */

static int v4l2_get_format(struct v4l2_device *dev)
{
    TRACE("");
    struct v4l2_format fmt;
    int ret;

    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(dev->v4l2_fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0) {
        return ret;
    }

    TRACE("V4L2: Getting current format: %c%c%c%c %ux%u", pixfmtstr(fmt.fmt.pix.pixelformat), fmt.fmt.pix.width,
           fmt.fmt.pix.height);

    return 0;
}

static int v4l2_set_format(struct v4l2_device *dev, struct v4l2_format *fmt)
{
    TRACE("");
    int ret;

    ret = ioctl(dev->v4l2_fd, VIDIOC_S_FMT, fmt);
    if (ret < 0) {
        TRACE("V4L2: Unable to set format %s (%d).", strerror(errno), errno);
        return ret;
    }

    TRACE("V4L2: Setting format to: %c%c%c%c %ux%u", pixfmtstr(fmt->fmt.pix.pixelformat), fmt->fmt.pix.width,
           fmt->fmt.pix.height);

    return 0;
}

static int v4l2_set_ctrl(struct v4l2_device *dev, int new_val, int ctrl)
{
    TRACE("");
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control control;
    int ret;

    CLEAR(queryctrl);

    switch (ctrl) {
    case V4L2_CID_BRIGHTNESS:
        queryctrl.id = V4L2_CID_BRIGHTNESS;
        ret = ioctl(dev->v4l2_fd, VIDIOC_QUERYCTRL, &queryctrl);
        if (-1 == ret) {
            if (errno != EINVAL)
                TRACE(
                    "V4L2: VIDIOC_QUERYCTRL"
                    " failed: %s (%d).",
                    strerror(errno), errno);
            else
                TRACE(
                    "V4L2_CID_BRIGHTNESS is not"
                    " supported: %s (%d).",
                    strerror(errno), errno);

            return ret;
        } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            TRACE("V4L2_CID_BRIGHTNESS is not supported.");
            ret = -EINVAL;
            return ret;
        } else {
            CLEAR(control);
            control.id = V4L2_CID_BRIGHTNESS;
            control.value = new_val;

            ret = ioctl(dev->v4l2_fd, VIDIOC_S_CTRL, &control);
            if (-1 == ret) {
                TRACE("V4L2: VIDIOC_S_CTRL failed: %s (%d).", strerror(errno), errno);
                return ret;
            }
        }
        TRACE("V4L2: Brightness control changed to value = 0x%x", new_val);
        break;

    default:
        /* TODO: We don't support any other controls. */
        return -EINVAL;
    }

    return 0;
}

static int v4l2_start_capturing(struct v4l2_device *dev)
{
    TRACE("");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(dev->v4l2_fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        TRACE("V4L2: Unable to start streaming: %s (%d).", strerror(errno), errno);
        return ret;
    }

    TRACE("V4L2: Starting video stream.");

    return 0;
}

static int v4l2_stop_capturing(struct v4l2_device *dev)
{
    TRACE("");
    enum v4l2_buf_type type;
    int ret;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        ret = ioctl(dev->v4l2_fd, VIDIOC_STREAMOFF, &type);
        if (ret < 0) {
            TRACE("V4L2: VIDIOC_STREAMOFF failed: %s (%d).", strerror(errno), errno);
            return ret;
        }

        break;
    default:
        /* Nothing to do. */
        break;
    }

    return 0;
}
static int v4l2_open(struct v4l2_device **v4l2, char *devname, struct v4l2_format *s_fmt)
{
    TRACE("");
    struct v4l2_device *dev;
    struct v4l2_capability cap;
    int fd;
    int ret = -EINVAL;

    fd = open(devname, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        TRACE("V4L2: device open failed: %s (%d).", strerror(errno), errno);
        return ret;
    }

    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        TRACE("V4L2: VIDIOC_QUERYCAP failed: %s (%d).", strerror(errno), errno);
        goto err;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        TRACE("V4L2: %s is no video capture device", devname);
        goto err;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        TRACE("V4L2: %s does not support streaming i/o", devname);
        goto err;
    }

    dev = calloc(1, sizeof *dev);
    if (dev == NULL) {
        ret = -ENOMEM;
        goto err;
    }

    TRACE("V4L2 device is %s on bus %s", cap.card, cap.bus_info);

    dev->v4l2_fd = fd;

    /* Get the default image format supported. */
    ret = v4l2_get_format(dev);
    if (ret < 0)
        goto err_free;

    /*
     * Set the desired image format.
     * Note: VIDIOC_S_FMT may change width and height.
     */
    ret = v4l2_set_format(dev, s_fmt);
    if (ret < 0)
        goto err_free;

    /* Get the changed image format. */
    ret = v4l2_get_format(dev);
    if (ret < 0)
        goto err_free;

    TRACE("v4l2 open succeeded, file descriptor = %d", fd);

    *v4l2 = dev;

    return 0;

err_free:
    free(dev);
err:
    close(fd);

    return ret;
}

static void v4l2_close(struct v4l2_device *dev)
{
    TRACE("");
    close(dev->v4l2_fd);
    free(dev);
}

/* ---------------------------------------------------------------------------
 * UVC generic stuff
 */

static int uvc_video_set_format(struct uvc_device *dev)
{
    TRACE(V4L2_FOURCC_FMT, V4L2_FOURCC_ARG(dev->fcc));

    struct v4l2_format fmt;
    int ret;

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = dev->width;
    fmt.fmt.pix.height = dev->height;
    fmt.fmt.pix.pixelformat = dev->fcc;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (dev->fcc == V4L2_PIX_FMT_MJPEG || dev->fcc == V4L2_PIX_FMT_H264)
    {
        assert(dev->width);
        assert(dev->height);
		/* required for MMAP to work
		 * this affects the buffer size retuned by MMAP */
        fmt.fmt.pix.sizeimage = dev->width * dev->height;
    }

    ret = ioctl(dev->uvc_fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        TRACE("UVC: Unable to set format %s (%d).", strerror(errno), errno);
        return ret;
    }

    TRACE("UVC: Setting format to: %c%c%c%c %ux%u", pixfmtstr(dev->fcc), dev->width, dev->height);

    return 0;
}

static int uvc_video_stream(struct uvc_device *dev, int enable)
{
    TRACE("");
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int ret;

    if (!enable) {
        ret = ioctl(dev->uvc_fd, VIDIOC_STREAMOFF, &type);
        if (ret < 0) {
            TRACE("UVC: VIDIOC_STREAMOFF failed: %s (%d).", strerror(errno), errno);
            return ret;
        }

        TRACE("UVC: Stopping video stream.");

        return 0;
    }

    ret = ioctl(dev->uvc_fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        TRACE("UVC: Unable to start streaming %s (%d).", strerror(errno), errno);
        return ret;
    }

    TRACE("UVC: Starting video stream.");

    dev->uvc_shutdown_requested = 0;

    return 0;
}

static int uvc_uninit_device(struct uvc_device *dev)
{
    TRACE("");
    unsigned int i;
    int ret;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        for (i = 0; i < dev->nbufs; ++i) {
            ret = munmap(dev->mem[i].start, dev->mem[i].length);
            if (ret < 0) {
                TRACE("UVC: munmap failed");
                return ret;
            }
        }

        free(dev->mem);
        break;

    case IO_METHOD_USERPTR:
    default:
        if (dev->run_standalone) {
            for (i = 0; i < dev->nbufs; ++i)
                free(dev->dummy_buf[i].start);

            free(dev->dummy_buf);
        }
        break;
    }

    return 0;
}
static int uvc_open(struct uvc_device **uvc, char *devname)
{
    TRACE("name: %s", devname);
    struct uvc_device *dev;
    struct v4l2_capability cap;
    int fd;
    int ret = -EINVAL;

    fd = open(devname, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        TRACE("UVC: device open failed: %s (%d).", strerror(errno), errno);
        return ret;
    }

    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        TRACE("UVC: unable to query uvc device: %s (%d)", strerror(errno), errno);
        goto err;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
        TRACE("UVC: %s is no video output device", devname);
        goto err;
    }

    dev = calloc(1, sizeof *dev);
    if (dev == NULL) {
        ret = -ENOMEM;
        goto err;
    }

    TRACE("uvc device is %s on bus %s", cap.card, cap.bus_info);
    TRACE("uvc open succeeded, file descriptor = %d", fd);

    dev->uvc_fd = fd;
    *uvc = dev;

    return 0;

err:
    close(fd);
    return ret;
}

static void uvc_close(struct uvc_device *dev)
{
    TRACE("");
    close(dev->uvc_fd);
    free(dev);
}

/* ---------------------------------------------------------------------------
 * UVC streaming related
 */

static inline void pts_to_v4l2_timestamp(
    const struct uvc_device *dev, struct v4l2_buffer *buf)
{
    assert(buf);

    int64_t now = monotonic_us();


    int64_t pts = avpacket_pts(dev);
    pts += dev->time_offset_us;
    buf->timestamp.tv_usec = pts % 1000000;
    buf->timestamp.tv_sec = pts / 1000000;
    buf->flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

    TRACE("diff %ldms", (pts - now) / 1000);
}

static void uvc_video_fill_buffer(struct uvc_device *dev, struct v4l2_buffer *buf)
{
    unsigned int bpl;
    unsigned int i;
	assert(dev->av_packet);
	assert(dev->av_packet->data);
    assert(dev->av_ctx);
    assert((int)dev->av_ctx->nb_streams > dev->av_packet->stream_index);
    const AVStream *stream = dev->av_ctx->streams[dev->av_packet->stream_index];
    const int extradata_size =
        !dev->qbuf_count && stream->codecpar ? stream->codecpar->extradata_size : 0;
    const uint8_t *extradata =
        !dev->qbuf_count && stream->codecpar ? stream->codecpar->extradata : NULL;
    const int size = extradata_size ? extradata_size : dev->av_packet->size;
    const uint8_t *data = extradata_size ? extradata : dev->av_packet->data;
    assert(size);

    switch (dev->fcc) {
    case V4L2_PIX_FMT_YUYV:
        /* Fill the buffer with video data. */
        bpl = dev->width * 2;
        for (i = 0; i < dev->height; ++i)
            memset(dev->mem[buf->index].start + i * bpl, dev->color++, bpl);

        buf->bytesused = bpl * dev->height;
        break;

    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_H264:
        {
            assert((int)dev->mem[buf->index].length > size);
            pts_to_v4l2_timestamp(dev, buf);
            void *const offset = dev->mem[buf->index].start;
            memcpy(offset, data, size);
            memset(offset + size, 0, (int)dev->mem[buf->index].length - size);
            buf->length = dev->mem[buf->index].length;
            buf->bytesused = size;
            buf->m.userptr = (unsigned long)dev->mem[buf->index].start;
            //dump_to_file(offset, size, "data", "bin");
        }
        break;
    }

    int64_t timestamp_us =
        buf->timestamp.tv_sec * INT64_C(1000000) + buf->timestamp.tv_usec;

    TRACE(
        "qbuf %lld dqbuf %lld"
        " index: %u, type: %u, bytesused: %u, flags: %08X, field: %u"
        " timestamp: %"PRId64 "us, seq: %u",
        dev->qbuf_count, dev->dqbuf_count,
        buf->index,
        buf->type,
        buf->bytesused,
        buf->flags,
        buf->field,
        timestamp_us,
        buf->sequence);
    if(!extradata_size) video_next_frame(dev);
}


static int uvc_video_process(struct uvc_device *dev)
{
    struct v4l2_buffer ubuf;
    struct v4l2_buffer vbuf;
    unsigned int i;
    int ret;
    /*
     * Return immediately if UVC video output device has not started
     * streaming yet.
     */
    if (!dev->is_streaming)
        return 0;

    /* Prepare a v4l2 buffer to be dequeued from UVC domain. */
    CLEAR(ubuf);

    ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ubuf.index = dev->qbuf_count % dev->nbufs;
    switch (dev->io) {
    case IO_METHOD_MMAP:
        ubuf.memory = V4L2_MEMORY_MMAP;
        break;

    case IO_METHOD_USERPTR:
    default:
        ubuf.memory = V4L2_MEMORY_USERPTR;
        break;
    }
    if (dev->run_standalone) {

        int64_t now_us = monotonic_us();

        if(dev->av_packet && now_us - dev->time_offset_us + 40000 /* 40ms */ < avpacket_pts(dev)) return 0;

        ret = ioctl(dev->uvc_fd, VIDIOC_QUERYBUF, &ubuf);
        if(ret < 0)
        {
            TRACE("ERROR: %d", ret);
            return ret;
        }

        if(V4L2_BUF_FLAG_DONE & ubuf.flags)
        {
            /* UVC stanalone setup. */
            ret = ioctl(dev->uvc_fd, VIDIOC_DQBUF, &ubuf);
            if (ret < 0)
            {
                TRACE("ERROR: %d", ret);
                return ret;
            }

            dev->dqbuf_count++;
        }

        if(dev->qbuf_count - dev->dqbuf_count >= dev->nbufs) return 0;


        if(dev->av_packet) uvc_video_fill_buffer(dev, &ubuf);
        else
        {
            uvc_video_stream(dev, 0);
            return 1;
        }

        ret = ioctl(dev->uvc_fd, VIDIOC_QBUF, &ubuf);
        if (ret != 0)
        {
            TRACE("ERROR: %d", ret);
            return ret;
        }

        dev->qbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
        //TRACE("VIDIOC_QBUF %d", ubuf.index);
#endif
    } else {
        /* UVC - V4L2 integrated path. */

        /*
         * Return immediately if V4L2 video capture device has not
         * started streaming yet or if QBUF was not called even once on
         * the UVC side.
         */
        if (!dev->vdev->is_streaming || !dev->first_buffer_queued)
        {
            TRACE(
                "vdev->is_streaming: %d, dev->first_buffer_queued: %d",
                  dev->vdev->is_streaming,
                  dev->first_buffer_queued);
            return 0;
        }

        /*
         * Do not dequeue buffers from UVC side until there are atleast
         * 2 buffers available at UVC domain.
         */
        if (!dev->uvc_shutdown_requested)
            if ((dev->dqbuf_count + 1) >= dev->qbuf_count)
                return 0;

        /* Dequeue the spent buffer from UVC domain */
        ret = ioctl(dev->uvc_fd, VIDIOC_DQBUF, &ubuf);
        if (ret < 0) {
            TRACE("UVC: Unable to dequeue buffer: %s (%d).", strerror(errno), errno);
            return ret;
        }

        if (dev->io == IO_METHOD_USERPTR)
            for (i = 0; i < dev->nbufs; ++i)
                if (ubuf.m.userptr == (unsigned long)dev->vdev->mem[i].start && ubuf.length == dev->vdev->mem[i].length)
                    break;

        dev->dqbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
        TRACE("DeQueued buffer at UVC side=%d", ubuf.index);
#endif

        /*
         * If the dequeued buffer was marked with state ERROR by the
         * underlying UVC driver gadget, do not queue the same to V4l2
         * and wait for a STREAMOFF event on UVC side corresponding to
         * set_alt(0). So, now all buffers pending at UVC end will be
         * dequeued one-by-one and we will enter a state where we once
         * again wait for a set_alt(1) command from the USB host side.
         */
        if (ubuf.flags & V4L2_BUF_FLAG_ERROR) {
            dev->uvc_shutdown_requested = 1;
            TRACE(
                "UVC: Possible USB shutdown requested from "
                "Host, seen during VIDIOC_DQBUF");
            return 0;
        }

        /* Queue the buffer to V4L2 domain */
        CLEAR(vbuf);

        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.memory = V4L2_MEMORY_MMAP;
        vbuf.index = ubuf.index;

        ret = ioctl(dev->vdev->v4l2_fd, VIDIOC_QBUF, &vbuf);
        if (ret < 0)
            return ret;

        dev->vdev->qbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
        TRACE("ReQueueing buffer at V4L2 side = %d", vbuf.index);
#endif
    }

    return 0;
}

static int uvc_video_qbuf_mmap(struct uvc_device *dev)
{
    TRACE("");
    unsigned int i;
    int ret;

    for (i = 0; i < dev->nbufs; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        /* UVC standalone setup. */
        if (dev->run_standalone)
            uvc_video_fill_buffer(dev, &(dev->mem[i].buf));

        ret = ioctl(dev->uvc_fd, VIDIOC_QBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            TRACE("UVC: VIDIOC_QBUF failed : %s (%d).", strerror(errno), errno);
            return ret;
        }

        dev->qbuf_count++;
    }

    return 0;
}

static int uvc_video_qbuf_userptr(struct uvc_device *dev)
{
    TRACE("");
    unsigned int i;
    int ret;

    /* UVC standalone setup. */
    if (dev->run_standalone) {
        for (i = 0; i < dev->nbufs; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.m.userptr = (unsigned long)dev->dummy_buf[i].start;
            buf.length = dev->dummy_buf[i].length;
            buf.index = i;

            ret = ioctl(dev->uvc_fd, VIDIOC_QBUF, &buf);
            if (ret < 0) {
                TRACE("UVC: VIDIOC_QBUF failed : %s (%d).", strerror(errno), errno);
                return ret;
            }

            dev->qbuf_count++;
        }
    }

    return 0;
}

static int uvc_video_qbuf(struct uvc_device *dev)
{
    TRACE("");
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        ret = uvc_video_qbuf_mmap(dev);
        break;

    case IO_METHOD_USERPTR:
        ret = uvc_video_qbuf_userptr(dev);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int uvc_video_reqbufs_mmap(struct uvc_device *dev, int nbufs)
{
    struct v4l2_requestbuffers rb;
    unsigned int i;
    int ret;

    CLEAR(rb);

    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(dev->uvc_fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        if (ret == -EINVAL)
            TRACE("UVC: does not support memory mapping");
        else
            TRACE("UVC: Unable to allocate buffers: %s (%d).", strerror(errno), errno);
        goto err;
    }

    if (!rb.count)
        return 0;

    if (rb.count < 2) {
        TRACE("UVC: Insufficient buffer memory.");
        ret = -EINVAL;
        goto err;
    }

    /* Map the buffers. */
    dev->mem = calloc(rb.count, sizeof dev->mem[0]);
    if (!dev->mem) {
        TRACE("UVC: Out of memory");
        ret = -ENOMEM;
        goto err;
    }

    for (i = 0; i < rb.count; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        ret = ioctl(dev->uvc_fd, VIDIOC_QUERYBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            TRACE(
                "UVC: VIDIOC_QUERYBUF failed for buf %d: "
                "%s (%d).",
                i, strerror(errno), errno);
            ret = -EINVAL;
            goto err_free;
        }
        dev->mem[i].start =
            mmap(NULL /* start anywhere */, dev->mem[i].buf.length, PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */, dev->uvc_fd, dev->mem[i].buf.m.offset);

        if (MAP_FAILED == dev->mem[i].start) {
            TRACE("UVC: Unable to map buffer %u: %s (%d).", i, strerror(errno), errno);
            dev->mem[i].length = 0;
            ret = -EINVAL;
            goto err_free;
        }

        dev->mem[i].length = dev->mem[i].buf.length;
        TRACE("UVC: Buffer %u mapped at address %p len %d", i, dev->mem[i].start, (int)dev->mem[i].length);
    }

    dev->nbufs = rb.count;
    TRACE("UVC: %u buffers allocated.", rb.count);

    return 0;

err_free:
    free(dev->mem);
err:
    return ret;
}

static int uvc_video_reqbufs_userptr(struct uvc_device *dev, int nbufs)
{
    TRACE("");
    struct v4l2_requestbuffers rb;
    unsigned int i, j, bpl = 0, payload_size = 0;
    int ret;

    CLEAR(rb);

    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_USERPTR;

    ret = ioctl(dev->uvc_fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        if (ret == -EINVAL)
            TRACE("UVC: does not support user pointer i/o");
        else
            TRACE("UVC: VIDIOC_REQBUFS error %s (%d).", strerror(errno), errno);
        goto err;
    }

    if (!rb.count)
        return 0;

    dev->nbufs = rb.count;

    if (dev->run_standalone) {
        /* Allocate buffers to hold dummy data pattern. */
        dev->dummy_buf = calloc(rb.count, sizeof dev->dummy_buf[0]);
        if (!dev->dummy_buf) {
            TRACE("UVC: Out of memory");
            ret = -ENOMEM;
            goto err;
        }

        switch (dev->fcc) {
        case V4L2_PIX_FMT_YUYV:
            bpl = dev->width * 2;
            payload_size = dev->width * dev->height * 2;
            break;
        case V4L2_PIX_FMT_MJPEG:
		case V4L2_PIX_FMT_H264:
            {
				assert(dev->width);
				assert(dev->height);
                payload_size = dev->width * dev->height;
            }
            break;
        }

        for (i = 0; i < rb.count; ++i) {
            assert(payload_size);

            dev->dummy_buf[i].length = payload_size;
            dev->dummy_buf[i].start = calloc(1, payload_size);
			TRACE("UVC: buf %p size %d", dev->dummy_buf[i].start, payload_size);

            if (!dev->dummy_buf[i].start) {
                TRACE("UVC: Out of memory");
                ret = -ENOMEM;
                goto err;
            }

            if (V4L2_PIX_FMT_YUYV == dev->fcc)
            {
                for (j = 0; j < dev->height; ++j)
                {
                    assert(bpl);
                    memset(dev->dummy_buf[i].start + j * bpl, dev->color++, bpl);
                }
            }
        }

        dev->mem = dev->dummy_buf;
    }

    return 0;

err:
    return ret;
}

static int uvc_video_reqbufs(struct uvc_device *dev, int nbufs)
{
    TRACE("nbufs: %d, io: %d", nbufs, dev->io);
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        ret = uvc_video_reqbufs_mmap(dev, nbufs);
        break;

    case IO_METHOD_USERPTR:
        ret = uvc_video_reqbufs_userptr(dev, nbufs);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

/*
 * This function is called in response to either:
 * 	- A SET_ALT(interface 1, alt setting 1) command from USB host,
 * 	  if the UVC gadget supports an ISOCHRONOUS video streaming endpoint
 * 	  or,
 *
 *	- A UVC_VS_COMMIT_CONTROL command from USB host, if the UVC gadget
 *	  supports a BULK type video streaming endpoint.
 */
static int uvc_handle_streamon_event(struct uvc_device *dev)
{
    TRACE("");
    int ret;

    ret = uvc_video_reqbufs(dev, dev->nbufs);
    if (ret < 0)
        goto err;

    if (!dev->run_standalone) {
        /* UVC - V4L2 integrated path. */
        if (IO_METHOD_USERPTR == dev->vdev->io) {
            /*
             * Ensure that the V4L2 video capture device has already
             * some buffers queued.
             */
            ret = v4l2_reqbufs(dev->vdev, dev->vdev->nbufs);
            if (ret < 0)
                goto err;
        }

        ret = v4l2_qbuf(dev->vdev);
        if (ret < 0)
            goto err;

        /* Start V4L2 capturing now. */
        ret = v4l2_start_capturing(dev->vdev);
        if (ret < 0)
            goto err;

        dev->vdev->is_streaming = 1;
    }

    /* Common setup. */

    if (dev->run_standalone) {
        uvc_video_stream(dev, 1);
        dev->first_buffer_queued = 1;
        dev->is_streaming = 1;
    }

    return 0;

err:
    return ret;
}

/* ---------------------------------------------------------------------------
 * UVC Request processing
 */

static void
uvc_fill_streaming_control(struct uvc_device *dev, struct uvc_streaming_control *ctrl, int iframe, int iformat)
{
    TRACE("iframe: %d, iformat: %d", iframe, iformat);
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    unsigned int nframes;

    if (iformat < 0)
        iformat = ARRAY_SIZE(uvc_formats) + iformat;
    if (iformat < 0 || iformat >= (int)ARRAY_SIZE(uvc_formats))
        return;
    format = &uvc_formats[iformat];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    if (iframe < 0)
        iframe = nframes + iframe;
    if (iframe < 0 || iframe >= (int)nframes)
        return;
    frame = &format->frames[iframe];

    memset(ctrl, 0, sizeof *ctrl);

    ctrl->bmHint = 1;
    ctrl->bFormatIndex = iformat + 1;
    ctrl->bFrameIndex = iframe + 1;
    ctrl->dwFrameInterval = frame->intervals[0];
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_H264:
        {
            ctrl->dwMaxVideoFrameSize = frame->width * frame->height;
            TRACE("dwMaxVideoFrameSize: %d", ctrl->dwMaxVideoFrameSize);
        }
        break;
    }

    /* TODO: the UVC maxpayload transfer size should be filled
     * by the driver.
     */
    if (!dev->bulk)
        ctrl->dwMaxPayloadTransferSize = (dev->maxpkt) * (dev->mult + 1) * (dev->burst + 1);
    else
        ctrl->dwMaxPayloadTransferSize = ctrl->dwMaxVideoFrameSize;

    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

static void
uvc_events_process_standard(struct uvc_device *dev, struct usb_ctrlrequest *ctrl, struct uvc_request_data *resp)
{
    TRACE("");
    (void)dev;
    (void)ctrl;
    (void)resp;
}

static void uvc_events_process_control(
    struct uvc_device *dev, uint8_t req, uint8_t cs, uint8_t entity_id, uint8_t len, struct uvc_request_data *resp)
{
    switch (entity_id) {
    case 0:
        switch (cs) {
        case UVC_VC_REQUEST_ERROR_CODE_CONTROL:
            TRACE("UVC_VC_REQUEST_ERROR_CODE_CONTROL");
            /* Send the request error code last prepared. */
            resp->data[0] = dev->request_error_code.data[0];
            resp->length = dev->request_error_code.length;
            break;

        default:
            TRACE("NOT SUPPORTED");
            /*
             * If we were not supposed to handle this
             * 'cs', prepare an error code response.
             */
            dev->request_error_code.data[0] = 0x06;
            dev->request_error_code.length = 1;
            break;
        }
        break;

    /* Camera terminal unit 'UVC_VC_INPUT_TERMINAL'. */
    case 1:
        switch (cs) {
        /*
         * We support only 'UVC_CT_AE_MODE_CONTROL' for CAMERA
         * terminal, as our bmControls[0] = 2 for CT. Also we
         * support only auto exposure.
         */
        case UVC_CT_AE_MODE_CONTROL:
            TRACE("UVC_CT_AE_MODE_CONTROL");
            switch (req) {
            case UVC_SET_CUR:
                /* Incase of auto exposure, attempts to
                 * programmatically set the auto-adjusted
                 * controls are ignored.
                 */
                resp->data[0] = 0x01;
                resp->length = 1;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;

            case UVC_GET_INFO:
                /*
                 * TODO: We support Set and Get requests, but
                 * don't support async updates on an video
                 * status (interrupt) endpoint as of
                 * now.
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;

            case UVC_GET_CUR:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                /* Auto Mode â€“ auto Exposure Time, auto Iris. */
                resp->data[0] = 0x02;
                resp->length = 1;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            default:
                TRACE("NOT SUPPORTED");
                /*
                 * We don't support this control, so STALL the
                 * control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * value.
                 */
                dev->request_error_code.data[0] = 0x07;
                dev->request_error_code.length = 1;
                break;
            }
            break;

        default:
            TRACE("NOT SUPPORTED");
            /*
             * We don't support this control, so STALL the control
             * ep.
             */
            resp->length = -EL2HLT;
            /*
             * If we were not supposed to handle this
             * 'cs', prepare a Request Error Code response.
             */
            dev->request_error_code.data[0] = 0x06;
            dev->request_error_code.length = 1;
            break;
        }
        break;

    /* processing unit 'UVC_VC_PROCESSING_UNIT' */
    case 2:
        switch (cs) {
        /*
         * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing
         * Unit, as our bmControls[0] = 1 for PU.
         */
        case UVC_PU_BRIGHTNESS_CONTROL:
            TRACE("UVC_PU_BRIGHTNESS_CONTROL");
            switch (req) {
            case UVC_SET_CUR:
                resp->data[0] = 0x0;
                resp->length = len;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_MIN:
                resp->data[0] = PU_BRIGHTNESS_MIN_VAL;
                resp->length = 2;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_MAX:
                resp->data[0] = PU_BRIGHTNESS_MAX_VAL;
                resp->length = 2;
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_CUR:
                resp->length = 2;
                memcpy(&resp->data[0], &dev->brightness_val, resp->length);
                /*
                 * For every successfully handled control
                 * request set the request error code to no
                 * error
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_INFO:
                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                /*
                 * For every successfully handled control
                 * request, set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_DEF:
                resp->data[0] = PU_BRIGHTNESS_DEFAULT_VAL;
                resp->length = 2;
                /*
                 * For every successfully handled control
                 * request, set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            case UVC_GET_RES:
                resp->data[0] = PU_BRIGHTNESS_STEP_SIZE;
                resp->length = 2;
                /*
                 * For every successfully handled control
                 * request, set the request error code to no
                 * error.
                 */
                dev->request_error_code.data[0] = 0x00;
                dev->request_error_code.length = 1;
                break;
            default:
                TRACE("NOT SUPPORTED");
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                dev->request_error_code.data[0] = 0x07;
                dev->request_error_code.length = 1;
                break;
            }
            break;

        default:
            TRACE("NOT SUPPORTED");
            /*
             * We don't support this control, so STALL the control
             * ep.
             */
            resp->length = -EL2HLT;
            /*
             * If we were not supposed to handle this
             * 'cs', prepare a Request Error Code response.
             */
            dev->request_error_code.data[0] = 0x06;
            dev->request_error_code.length = 1;
            break;
        }

        break;

    default:
        TRACE("NOT SUPPORTED");
        /*
         * If we were not supposed to handle this
         * 'cs', prepare a Request Error Code response.
         */
        dev->request_error_code.data[0] = 0x06;
        dev->request_error_code.length = 1;
        break;
    }
}

static void uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs, struct uvc_request_data *resp)
{
    struct uvc_streaming_control *ctrl;

    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
        return;

    TRACE("streaming request (req %02x cs %02x %s)", req, cs, UVC_VS_PROBE_CONTROL == cs ? "PROBE" : "CONTROL");

    ctrl = (struct uvc_streaming_control *)&resp->data;
    resp->length = sizeof *ctrl;

    switch (req) {
    case UVC_SET_CUR:
        TRACE("UVC_SET_CUR");
        dev->control = cs;
        resp->length = 34;
        break;

    case UVC_GET_CUR:
        TRACE("UVC_GET_CUR");
        if (cs == UVC_VS_PROBE_CONTROL)
            memcpy(ctrl, &dev->probe, sizeof *ctrl);
        else
            memcpy(ctrl, &dev->commit, sizeof *ctrl);
        break;

    case UVC_GET_MIN:
        TRACE("UVC_GET_MIN");
        goto control;
    case UVC_GET_MAX:
        TRACE("UVC_GET_MAX");
        goto control;
    case UVC_GET_DEF:
        TRACE("UVC_GET_DEF");
control:
        uvc_fill_streaming_control(dev, ctrl, req == UVC_GET_MAX ? -1 : 0, req == UVC_GET_MAX ? -1 : 0);
        break;

    case UVC_GET_RES:
        TRACE("UVC_GET_RES");
        CLEAR(ctrl);
        break;

    case UVC_GET_LEN:
        TRACE("UVC_GET_LEN");
        resp->data[0] = 0x00;
        resp->data[1] = 0x22;
        resp->length = 2;
        break;

    case UVC_GET_INFO:
        TRACE("UVC_GET_INFO");
        resp->data[0] = 0x03;
        resp->length = 1;
        break;
    }
}

static void
uvc_events_process_class(struct uvc_device *dev, struct usb_ctrlrequest *ctrl, struct uvc_request_data *resp)
{
    if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
    {
        TRACE("recipient not interface");
        return;
    }

    switch (ctrl->wIndex & 0xff) {
    case UVC_INTF_CONTROL:
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8, ctrl->wIndex >> 8, ctrl->wLength, resp);
        break;
    case UVC_INTF_STREAMING:
        uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8, resp);
        break;
    default:
        TRACE("unsupported wIndex %d", (int)ctrl->wIndex);
        break;
    }
}
static void
uvc_events_process_setup(struct uvc_device *dev, struct usb_ctrlrequest *ctrl, struct uvc_request_data *resp)
{
    dev->control = 0;

    const char *recipient_id[] =
    {"device", "interface", "endpoint", "other", "reserved"};
    const int recipient = ctrl->bRequestType & USB_RECIP_MASK;

#ifdef ENABLE_USB_REQUEST_DEBUG
    TRACE(
        "bRequestType %02x (%s, %d|%s) bRequest %02x wValue %04x wIndex %04x "
        "wLength %04x",
        ctrl->bRequestType,
        ctrl->bRequest & 0x80 ? "Device-to-Host" : "Host-to-device",
        recipient, recipient_id[min(4, recipient)],
        ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
#endif

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        uvc_events_process_standard(dev, ctrl, resp);
        break;
    case USB_TYPE_CLASS:
        uvc_events_process_class(dev, ctrl, resp);
        break;
    case USB_TYPE_VENDOR:
        TRACE("unsupported vendor request %02x", ctrl->bRequestType);
        break;
    case USB_TYPE_RESERVED:
        TRACE("unsupported reserved request %02x", ctrl->bRequestType);
        break;
    default:
        TRACE("unsupported request %02x", ctrl->bRequestType);
        break;
    }
}

static int
uvc_events_process_control_data(struct uvc_device *dev, uint8_t cs, uint8_t entity_id, struct uvc_request_data *data)
{
    TRACE("");
    switch (entity_id) {
    /* Processing unit 'UVC_VC_PROCESSING_UNIT'. */
    case 2:
        switch (cs) {
        /*
         * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing
         * Unit, as our bmControls[0] = 1 for PU.
         */
        case UVC_PU_BRIGHTNESS_CONTROL:
            memcpy(&dev->brightness_val, data->data, data->length);
            /* UVC - V4L2 integrated path. */
            if (!dev->run_standalone)
                /*
                 * Try to change the Brightness attribute on
                 * Video capture device. Note that this try may
                 * succeed or end up with some error on the
                 * video capture side. By default to keep tools
                 * like USBCV's UVC test suite happy, we are
                 * maintaining a local copy of the current
                 * brightness value in 'dev->brightness_val'
                 * variable and we return the same value to the
                 * Host on receiving a GET_CUR(BRIGHTNESS)
                 * control request.
                 *
                 * FIXME: Keeping in view the point discussed
                 * above, notice that we ignore the return value
                 * from the function call below. To be strictly
                 * compliant, we should return the same value
                 * accordingly.
                 */
                v4l2_set_ctrl(dev->vdev, dev->brightness_val, V4L2_CID_BRIGHTNESS);
            break;

        default:
            break;
        }

        break;

    default:
        break;
    }

    TRACE("Control Request data phase (cs %02x entity %02x)", cs, entity_id);

    return 0;
}

static int uvc_events_process_data(struct uvc_device *dev, struct uvc_request_data *data)
{
    struct uvc_streaming_control *target;
    struct uvc_streaming_control *ctrl;
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    const unsigned int *interval;
    unsigned int iformat, iframe;
    unsigned int nframes;
    unsigned int *val = (unsigned int *)data->data;
    int ret;

    switch (dev->control) {
    case UVC_VS_PROBE_CONTROL:
        TRACE("setting PROBE control, length = %d", data->length);
        target = &dev->probe;
        break;

    case UVC_VS_COMMIT_CONTROL:
        TRACE("setting COMMIT control, length = %d", data->length);
        target = &dev->commit;
        break;

    default:
        TRACE("setting unknown control, length = %d", data->length);

        /*
         * As we support only BRIGHTNESS control, this request is
         * for setting BRIGHTNESS control.
         * Check for any invalid SET_CUR(BRIGHTNESS) requests
         * from Host. Note that we support Brightness levels
         * from 0x0 to 0x10 in a step of 0x1. So, any request
         * with value greater than 0x10 is invalid.
         */
        if (*val > PU_BRIGHTNESS_MAX_VAL) {
            return -EINVAL;
        } else {
            ret = uvc_events_process_control_data(dev, UVC_PU_BRIGHTNESS_CONTROL, 2, data);
            if (ret < 0)
                goto err;

            return 0;
        }
    }

    ctrl = (struct uvc_streaming_control *)&data->data;
    iformat = clamp((unsigned int)ctrl->bFormatIndex, 1U, (unsigned int)ARRAY_SIZE(uvc_formats));
    format = &uvc_formats[iformat - 1];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    iframe = clamp((unsigned int)ctrl->bFrameIndex, 1U, nframes);
    frame = &format->frames[iframe - 1];
    interval = frame->intervals;

    while (interval[0] < ctrl->dwFrameInterval && interval[1])
        ++interval;

    target->bFormatIndex = iformat;
    target->bFrameIndex = iframe;
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        target->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_H264:
        {
			assert(dev->width);
			assert(dev->height);
            target->dwMaxVideoFrameSize = frame->width * frame->height;
            TRACE("dwMaxVideoFrameSize: %d", target->dwMaxVideoFrameSize);
            break;
        }
    }
    target->dwFrameInterval = *interval;

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        TRACE(
            "switching format from "
            V4L2_FOURCC_FMT
            " to "
            V4L2_FOURCC_FMT,
            V4L2_FOURCC_ARG(dev->fcc),
            V4L2_FOURCC_ARG(format->fcc));
        dev->fcc = format->fcc;
        dev->width = frame->width;
        dev->height = frame->height;

		ret = uvc_video_set_format(dev);
		if (ret < 0)
			goto err;

		if (dev->bulk) {
			ret = uvc_handle_streamon_event(dev);
			if (ret < 0)
				goto err;
		}
    }

    return 0;

err:
    return ret;
}

static void uvc_events_process(struct uvc_device *dev)
{
    struct v4l2_event v4l2_event;
    struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
    struct uvc_request_data resp;
    int ret;

    ret = ioctl(dev->uvc_fd, VIDIOC_DQEVENT, &v4l2_event);
    if (ret < 0) {
        TRACE("VIDIOC_DQEVENT failed: %s (%d)", strerror(errno), errno);
        return;
    }

    memset(&resp, 0, sizeof resp);
    resp.length = -EL2HLT;

    switch (v4l2_event.type) {
    case UVC_EVENT_CONNECT:
        TRACE("UVC_EVENT_CONNECT");
        return;

    case UVC_EVENT_DISCONNECT:
        TRACE("UVC_EVENT_DISCONNECT");
        dev->uvc_shutdown_requested = 1;
        TRACE(
            "UVC: Possible USB shutdown requested from "
            "Host, seen via UVC_EVENT_DISCONNECT");
        return;

    case UVC_EVENT_SETUP:
        uvc_events_process_setup(dev, &uvc_event->req, &resp);
        break;

    case UVC_EVENT_DATA:
        ret = uvc_events_process_data(dev, &uvc_event->data);
        if (ret < 0)
            break;
        return;

    case UVC_EVENT_STREAMON:
        dev->time_offset_us = monotonic_us();

        TRACE(
            "UVC_EVENT_STREAMON %" PRId64 "ms",
            dev->time_offset_us / 1000);
        if (!dev->bulk) uvc_handle_streamon_event(dev);
        return;

    case UVC_EVENT_STREAMOFF:
        TRACE("UVC_EVENT_STREAMOFF");
        /* Stop V4L2 streaming... */
        if (!dev->run_standalone && dev->vdev->is_streaming) {
            /* UVC - V4L2 integrated path. */
            v4l2_stop_capturing(dev->vdev);
            dev->vdev->is_streaming = 0;
        }

        /* ... and now UVC streaming.. */
        if (dev->is_streaming) {
            uvc_video_stream(dev, 0);
            uvc_uninit_device(dev);
            uvc_video_reqbufs(dev, 0);
            dev->is_streaming = 0;
            dev->first_buffer_queued = 0;
        }

        return;
    }

    ret = ioctl(dev->uvc_fd, UVCIOC_SEND_RESPONSE, &resp);
    if (ret < 0) {
        TRACE("UVCIOC_S_EVENT failed: %s (%d)", strerror(errno), errno);
        return;
    }
}

static void uvc_events_init(struct uvc_device *dev)
{
    TRACE("");
    struct v4l2_event_subscription sub;
    unsigned int payload_size = 0;

    switch (dev->fcc) {
    case V4L2_PIX_FMT_YUYV:
        payload_size = dev->width * dev->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_H264:
        {
            assert(dev->av_packet);
            payload_size = dev->av_packet->size;
        }
        break;
    }
#if 0
    uvc_fill_streaming_control(dev, &dev->probe, 0, 0);
    uvc_fill_streaming_control(dev, &dev->commit, 0, 0);
#endif

    if (dev->bulk) {
        assert(payload_size);
        /* FIXME Crude hack, must be negotiated with the driver. */
        dev->probe.dwMaxPayloadTransferSize = dev->commit.dwMaxPayloadTransferSize = payload_size;
    }

    memset(&sub, 0, sizeof sub);
    sub.type = UVC_EVENT_SETUP;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_DATA;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMON;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

/* ---------------------------------------------------------------------------
 * main
 */

static
int image_load(struct uvc_device *dev)
{
    assert(dev);
    assert(dev->video_filename);
    assert(!dev->av_ctx);
    int status = 0;

    status = avformat_open_input(&dev->av_ctx, dev->video_filename, NULL, NULL);

    if(status)
    {
        TRACE("error: %s", av_err2str(status));
        assert(NULL);
        return 1;
    }

    status = avformat_find_stream_info(dev->av_ctx, NULL);

    if(0 > status)
    {
        TRACE("error: %s", av_err2str(status));
        assert(NULL);
        return 1;
    }

    for(unsigned int i = 0; dev->av_ctx->nb_streams > i; ++i)
    {
        AVStream *stream = dev->av_ctx->streams[i];
        assert(stream);
        TRACE(
            "stream: %d, nb_frames: %" PRId64 ", time_base %d/%d",
            i,
            stream->nb_frames,
            stream->time_base.num,
            stream->time_base.den);
        av_dump_format(dev->av_ctx, i, dev->video_filename, 0 /* input */);
    }

    TRACE(
        "nb_streams: %u, packet_size: %u, start_time: %" PRId64,
        dev->av_ctx->nb_streams,
        dev->av_ctx->packet_size,
        dev->av_ctx->start_time);

    video_next_frame(dev);
    return 0;
}

static
int video_next_frame(struct uvc_device *dev)
{
    if(!dev->av_packet) dev->av_packet = av_packet_alloc();
    else av_packet_unref(dev->av_packet);

    assert(dev->av_packet);

    int status = av_read_frame(dev->av_ctx, dev->av_packet);

    if(AVERROR_EOF == status)
    {
        TRACE("EOF");
        dev->av_packet = NULL;
        return 1;
#if 0
        avformat_close_input(&dev->av_ctx);
        return image_load(dev);
#endif
    }

    if(0 != status)
    {
        TRACE("error: %s", av_err2str(status));
        assert(NULL);
        return 1;
    }

    TRACE(
        "PTS: %" PRId64 "ms"
        " POS: %" PRId64 " size: %d flags %08x",
        avpacket_pts(dev) / 1000,
        dev->av_packet->pos,
        dev->av_packet->size,
        dev->av_packet->flags);

    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -b		Use bulk mode\n");
    fprintf(stderr, " -d		Do not use any real V4L2 capture device\n");
    fprintf(stderr,
            " -f <format>    Select frame format\n\t"
            "0 = V4L2_PIX_FMT_YUYV\n\t"
            "1 = V4L2_PIX_FMT_H264\n\t"
            "2 = V4L2_PIX_FMT_MJPEG\n");
    fprintf(stderr, " -h		Print this help screen and exit\n");
    fprintf(stderr, " -i image	MJPEG image\n");
    fprintf(stderr, " -m		Streaming mult for ISOC (b/w 0 and 2)\n");
    fprintf(stderr, " -n		Number of Video buffers (b/w 2 and 32)\n");
    fprintf(stderr,
            " -o <IO method> Select UVC IO method:\n\t"
            "0 = MMAP\n\t"
            "1 = USER_PTR\n");
    fprintf(stderr,
            " -r <resolution> Select frame resolution:\n\t"
            "0 = 360p, VGA (640x360)\n\t"
            "1 = 720p, WXGA (1280x720)\n");
    fprintf(stderr,
            " -s <speed>	Select USB bus speed (b/w 0 and 2)\n\t"
            "0 = Full Speed (FS)\n\t"
            "1 = High Speed (HS)\n\t"
            "2 = Super Speed (SS)\n");
    fprintf(stderr, " -t		Streaming burst (b/w 0 and 15)\n");
    fprintf(stderr, " -u device	UVC Video Output device\n");
    fprintf(stderr, " -v device	V4L2 Video Capture device\n");
}

int main(int argc, char *argv[])
{
    struct uvc_device *udev = NULL;
    struct v4l2_device *vdev = NULL;
    struct timeval tv;
    struct v4l2_format fmt;
    char *uvc_devname = "/dev/video0";
    char *v4l2_devname = "/dev/video1";
    char *video_filename = NULL;

    fd_set fdsv, fdsu;
    int ret, opt, nfds;
    int bulk_mode = 0;
    int dummy_data_gen_mode = 0;
    /* Frame format/resolution related params. */
	uint32_t format =  V4L2_PIX_FMT_YUYV;
    int nbufs = 2;              /* Ping-Pong buffers */
    /* USB speed related params */
    int mult = 0;
    int burst = 0;
    enum usb_device_speed speed = USB_SPEED_SUPER; /* High-Speed */
    enum io_method uvc_io_method = IO_METHOD_USERPTR;
    int width = 0, height = 0;

    while ((opt = getopt(argc, argv, "bdf:h:i:m:n:o:r:s:t:u:v:w:")) != -1) {
        switch (opt) {
        case 'b':
            bulk_mode = 1;
            break;

        case 'd':
            dummy_data_gen_mode = 1;
            break;

        case 'f':
            {
                assert(optarg);
                int default_format = atoi(optarg);

                if(0 == default_format) format = V4L2_PIX_FMT_YUYV;
                else if(1 == default_format) format = V4L2_PIX_FMT_MJPEG;
                else if(2 == default_format) format = V4L2_PIX_FMT_H264;
                else {
                    usage(argv[0]);
                    return 1;
                }
            break;
            }

        case 'h':
            {
                assert(optarg);
                height = atoi(optarg);
                if(0 >= height || 1080 < height)
                {
                    usage(argv[0]);
                    return 1;
                }
                break;
            }
        case 'i':
            video_filename = optarg;
            break;

        case 'm':
            if (atoi(optarg) < 0 || atoi(optarg) > 2) {
                usage(argv[0]);
                return 1;
            }

            mult = atoi(optarg);
            TRACE("Requested Mult value = %d", mult);
            break;

        case 'n':
            if (atoi(optarg) < 2 || atoi(optarg) > 32) {
                usage(argv[0]);
                return 1;
            }

            nbufs = atoi(optarg);
            TRACE("Number of buffers requested = %d", nbufs);
            break;

        case 'o':
            if (atoi(optarg) < 0 || atoi(optarg) > 1) {
                usage(argv[0]);
                return 1;
            }

            uvc_io_method = atoi(optarg);
            TRACE("UVC: IO method requested is %s", (uvc_io_method == IO_METHOD_MMAP) ? "MMAP" : "USER_PTR");
            break;
        case 's':
            if (atoi(optarg) < 0 || atoi(optarg) > 2) {
                usage(argv[0]);
                return 1;
            }

            speed = atoi(optarg);
            break;

        case 't':
            if (atoi(optarg) < 0 || atoi(optarg) > 15) {
                usage(argv[0]);
                return 1;
            }

            burst = atoi(optarg);
            TRACE("Requested Burst value = %d", burst);
            break;

        case 'u':
            uvc_devname = optarg;
            break;

        case 'v':
            v4l2_devname = optarg;
            break;

        case 'w':
            {
                assert(optarg);
                width = atoi(optarg);
                if(0 >= width || 1920 < width)
                {
                    usage(argv[0]);
                    return 1;
                }
                break;
            }
        default:
            TRACE("Invalid option '-%c'", opt);
            usage(argv[0]);
            return 1;
        }
    }

    if (!dummy_data_gen_mode && !video_filename) {
        /*
         * Try to set the default format at the V4L2 video capture
         * device as requested by the user.
         */
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.sizeimage =
				V4L2_PIX_FMT_YUYV == format
				? (fmt.fmt.pix.width * fmt.fmt.pix.height * 2)
				: (fmt.fmt.pix.width * fmt.fmt.pix.height);

        fmt.fmt.pix.pixelformat = format;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        /* Open the V4L2 device. */
        ret = v4l2_open(&vdev, v4l2_devname, &fmt);
        if (vdev == NULL || ret < 0)
            return 1;
    }

    /* Open the UVC device. */
    ret = uvc_open(&udev, uvc_devname);
    if (udev == NULL || ret < 0)
        return 1;

    udev->uvc_devname = uvc_devname;

    if (!dummy_data_gen_mode && !video_filename) {
        vdev->v4l2_devname = v4l2_devname;
        /* Bind UVC and V4L2 devices. */
        udev->vdev = vdev;
        vdev->udev = udev;
    }

    /* Set parameters as passed by user. */
    udev->width = width;
    udev->height = height;
    udev->fcc = format;
    udev->io = uvc_io_method;
    udev->bulk = bulk_mode;
    udev->nbufs = nbufs;
    udev->mult = mult;
    udev->burst = burst;
    udev->speed = speed;

    if (dummy_data_gen_mode || video_filename)
        /* UVC standalone setup. */
        udev->run_standalone = 1;

    if (!dummy_data_gen_mode && !video_filename) {
        /* UVC - V4L2 integrated path */
        vdev->nbufs = nbufs;

        /*
         * IO methods used at UVC and V4L2 domains must be
         * complementary to avoid any memcpy from the CPU.
         */
        switch (uvc_io_method) {
        case IO_METHOD_MMAP:
            vdev->io = IO_METHOD_USERPTR;
            break;

        case IO_METHOD_USERPTR:
        default:
            vdev->io = IO_METHOD_MMAP;
            break;
        }
    }

    switch (speed) {
    case USB_SPEED_FULL:
        /* Full Speed. */
        if (bulk_mode)
            udev->maxpkt = 64;
        else
            udev->maxpkt = 1023;
        break;

    case USB_SPEED_HIGH:
        /* High Speed. */
        if (bulk_mode)
            udev->maxpkt = 512;
        else
            udev->maxpkt = 1024;
        break;

    case USB_SPEED_SUPER:
    default:
        /* Super Speed. */
        if (bulk_mode)
            udev->maxpkt = 1024;
        else
            udev->maxpkt = 1024;
        break;
    }

    if (!dummy_data_gen_mode && !video_filename && (IO_METHOD_MMAP == vdev->io)) {
        /*
         * Ensure that the V4L2 video capture device has already some
         * buffers queued.
         */
        v4l2_reqbufs(vdev, vdev->nbufs);
    }

    if (video_filename)
    {
        udev->video_filename = video_filename;
        if(image_load(udev)) return 1;
    }

    /* Init UVC events. */
    uvc_events_init(udev);

    while (1) {
        if (!dummy_data_gen_mode && !video_filename)
            FD_ZERO(&fdsv);

        FD_ZERO(&fdsu);

        /* We want both setup and data events on UVC interface.. */
        FD_SET(udev->uvc_fd, &fdsu);

        fd_set efds = fdsu;
        fd_set dfds = fdsu;

        /* ..but only data events on V4L2 interface */
        if (!dummy_data_gen_mode && !video_filename)
            FD_SET(vdev->v4l2_fd, &fdsv);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        if (!dummy_data_gen_mode && !video_filename) {
            nfds = max(vdev->v4l2_fd, udev->uvc_fd);
            ret = select(nfds + 1, &fdsv, &dfds, &efds, &tv);
        } else {
            ret = select(udev->uvc_fd + 1, NULL, &dfds, &efds, NULL);
        }

        if (-1 == ret) {
            TRACE("select error %d, %s", errno, strerror(errno));
            if (EINTR == errno)
                continue;

            break;
        }

        if (0 == ret) {
            TRACE("select timeout");
            break;
        }

        if (FD_ISSET(udev->uvc_fd, &efds))
            uvc_events_process(udev);
        if (FD_ISSET(udev->uvc_fd, &dfds))
        {
            if(0 != uvc_video_process(udev)) break;
        }
        if (!dummy_data_gen_mode && !video_filename)
            if (FD_ISSET(vdev->v4l2_fd, &fdsv))
                v4l2_process_data(vdev);
    }

    if (!dummy_data_gen_mode && !video_filename && vdev->is_streaming) {
        /* Stop V4L2 streaming... */
        v4l2_stop_capturing(vdev);
        v4l2_uninit_device(vdev);
        v4l2_reqbufs(vdev, 0);
        vdev->is_streaming = 0;
    }

    if (udev->is_streaming) {
        /* ... and now UVC streaming.. */
        uvc_video_stream(udev, 0);
        uvc_uninit_device(udev);
        uvc_video_reqbufs(udev, 0);
        udev->is_streaming = 0;
    }

    if (!dummy_data_gen_mode && !video_filename)
        v4l2_close(vdev);

    if(udev->av_ctx) avformat_close_input(&udev->av_ctx);
    uvc_close(udev);
    return 0;
}
