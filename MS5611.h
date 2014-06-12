/*
 * MS5611.h - My first kernel driver
 */

 /* Device specific macro definitions */
#define ADDR                    0xEE  /* Module address write mode 1110 1110 bit1=CSB bit0=0 write */
 
#define ADDR_RESET_CMD          0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1     0x48	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2     0x58	/* write to this address to start pressure conversion */
#define ADDR_DATA               0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP         0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1            0xA2	/* address of 6x 2 bytes calibration data */

/* interface ioctls */
#define IOCTL_RESET             2
#define IOCTL_MEASURE           3

/* Implementation specific macro definitions */
#define FIFO_SIZE               32
#define COEF_SIZE               8

#define CLASS_NAME              "MS5611"
#define DEVICE_NAME             "ms5611"

#define MAJOR_DEVID             0
#define SAMPLE_RATE             1 // in Hz