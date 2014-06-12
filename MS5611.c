/*
 * MS5611.c - My first Linux kernel driver
 * Based on: http://www.skybeeper.com/index.php/en/english-en/30-ms5611-pressure-sensor-i2c-driver
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include<linux/string.h>

#include "MS5611.h"


/////////////////////////////////////////////////////////////////////////////////////////
// PROTOTYPES
/////////////////////////////////////////////////////////////////////////////////////////
// misc:
uint8_t   check_spi_write  (int);
void      wait_ms          (uint16_t);

ssize_t   read_reg         (struct spi_device *, uint8_t);
ssize_t   read_mult_regs   (struct spi_device *, uint8_t, uint8_t *, size_t);
int       write_reg        (struct spi_device *, uint8_t, uint8_t);

// kernel driver:
int       MS5611_init      (void);
void      MS5611_exit      (void);
// file_operations:
ssize_t   fops_read        (struct file *, char __user *, size_t, loff_t *);
int       fops_open        (struct inode *, struct file *);
int       fops_close       (struct inode *, struct file *);
// spi_driver:
int       MS5611_probe     (struct spi_device *);
int       MS5611_remove    (struct spi_device *);

// runs as a thread
int       MS5611_get_sample(void *);

// fetching the sensor data
int       MS5611_reset     (struct spi_device *);
void      MS5611_read_coef (struct spi_device *, uint16_t *, uint8_t );
uint32_t  MS5611_read_prom (struct spi_device *, uint8_t);
uint32_t  MS5611_cnvrt_dx  (struct spi_device *, uint8_t);
void      MS5611_read_pt   (struct spi_device *, volatile int32_t *, volatile int32_t *);

/////////////////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
/////////////////////////////////////////////////////////////////////////////////////////
struct class*       _pMS5611_CLASS    = NULL;
struct device*      _pMS5611_DEVICE   = NULL;
struct task_struct* _pMS5611_SMPL_THR = NULL;
uint16_t            _pCOEF[COEF_SIZE]; //coefficients table for pressure sensor

typedef struct MS5611Report {
  struct timeval timestamp;
  uint32_t error_count;

  int32_t pressure_Pa;
  int32_t temperature_Deg;
} MS5611Report;

static struct of_device_id _MS5611_OF_MATCH[] = {
  { .compatible = "ti,ms5611", },
  { }
};

static struct file_operations _FOPS = {
  .read     = fops_read,
  .open     = fops_open,
  .release  = fops_close
};

static struct spi_driver _MS5611_DRIVER = {
  .driver = {
    .name  = DEVICE_NAME,
    .owner = THIS_MODULE,
    .of_match_table = _MS5611_OF_MATCH,
  },
  .probe  = MS5611_probe,
  .remove = MS5611_remove,
};

// macro calls
static DEFINE_MUTEX  (_MUT_RLOCK);
static DECLARE_KFIFO (_MS5611_REPORTS, MS5611Report, FIFO_SIZE);
MODULE_DEVICE_TABLE  (of, _MS5611_OF_MATCH);

/////////////////////////////////////////////////////////////////////////////////////////
// _FOPS
/////////////////////////////////////////////////////////////////////////////////////////
ssize_t fops_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
  int iBytesCopied;
  int32_t iFault;
  
  if(mutex_lock_interruptible(&_MUT_RLOCK) ) {
    return -ERESTARTSYS;
  }

  iFault = kfifo_to_user(&_MS5611_REPORTS, buf, count, &iBytesCopied);
  mutex_unlock(&_MUT_RLOCK);

  return iFault ? iFault : iBytesCopied;
}

int fops_open(struct inode * inode, struct file * file_ptr) {
  printk(KERN_ALERT "Open MS5611\n");
  return 0;
}

int fops_close(struct inode * inode, struct file * file_ptr) {
  printk(KERN_ALERT "Close MS5611\n");
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
// _MS5611_DRIVER
/////////////////////////////////////////////////////////////////////////////////////////
int MS5611_probe(struct spi_device *spi) {
  // Save spi pointer info
  spi->dev.platform_data = spi;
  
  // Init the device
  MS5611_reset(spi);
  MS5611_read_coef(spi, _pCOEF, COEF_SIZE);
  
  // Spawn thread for fetching samples
  _pMS5611_SMPL_THR = kthread_create(MS5611_get_sample, spi, "MS5611_sampling");
  if(_pMS5611_SMPL_THR) {
    printk(KERN_INFO "New thread created");
    wake_up_process(_pMS5611_SMPL_THR);
  }
  
  return 0;
}

int MS5611_remove(struct spi_device *spi) {
  printk(KERN_ALERT "Module removed\n");
  return 0;
}

// Note to kernel pro: I don't know whether "ADDR" is correct
int MS5611_reset(struct spi_device *spi) {
  int iRet;
  iRet = write_reg(spi, ADDR, ADDR_RESET_CMD);
  
  if(iRet < 0) {
    printk(KERN_ERR "MS5611_reset(): Reset device failed\n");
  } else {
    wait_ms(3);
  }
  
  return iRet;
}

int MS5611_get_sample(void *data) {
  struct spi_device *spi = (struct spi_device *)data;
  MS5611Report report;
  int iRet = 0;
  
  while(true) {
    schedule_timeout(SAMPLE_RATE / 100);

    // pull out old data to make room for new read
    // if needed and throw it out
    if(kfifo_is_full(&_MS5611_REPORTS) ) {
      iRet = kfifo_get(&_MS5611_REPORTS, &report);
    }
    
    do_gettimeofday(&report.timestamp);
    MS5611_read_pt(spi, &report.pressure_Pa, &report.temperature_Deg);
    iRet = kfifo_put(&_MS5611_REPORTS, &report);
    if(!iRet) {
      // TODO: fifo was full - error happened probably
    }
  }
  
  return iRet;
}

uint32_t MS5611_read_prom(struct spi_device *spi, uint8_t coef_num) {
  const uint8_t iRxBufSize = 2;
  uint8_t iRx[iRxBufSize];
  int iRet;
  uint8_t iRxIter;
  
  // Result variable after read of this function
  uint32_t iRes = 0;
  
  // send read coefficient command
  iRet = write_reg(spi, ADDR, ADDR_PROM_SETUP + coef_num * 2);
  if(!check_spi_write(iRet) ) {
    return iRet;
  }
  
  //read MSB and LSB
  iRet = read_mult_regs(spi, ADDR, iRx, sizeof(iRx) );
  if(!check_spi_write(iRet) ) {
    return iRet;
  }
  
  // Everything went well so far :D
  for(iRxIter = 0; iRxIter < iRxBufSize; iRxIter++) {
    iRes = (iRes << 8) + iRx[iRxIter];
  }
  return iRes;
}

void MS5611_read_coef(struct spi_device *spi, uint16_t *pC, uint8_t iLen) {
  uint8_t j;
    
  for(j = 0; j < iLen; j++) {
    pC[j] = MS5611_read_prom(spi, j);
  }
}

uint32_t MS5611_cnvrt_dx(struct spi_device *spi, uint8_t cmd) {
  const uint8_t iRxBufSize = 3;
  uint8_t iRx[iRxBufSize];
  int iRet = -1;
  uint32_t iDx = 0;
  
  uint8_t iRxIter;
  for(iRxIter = 0 ; iRxIter < 3 ; iRxIter++) {
    iRx[iRxIter] = 0;
  }
  
  // send conversion command
  iRet = write_reg(spi, ADDR, cmd);
  if(!check_spi_write(iRet) ) {
    return iRet;
  }
  wait_ms(10);
  
  //send read command
  iRet = write_reg(spi, ADDR, ADDR_DATA);
    if(!check_spi_write(iRet) ) {
    return iRet;
  }
  //wait_ms(3); // Maybe wait?
  
  //read 3 bytes conversion result
  read_mult_regs(spi, ADDR, iRx, iRxBufSize);
  
  for(iRxIter = 0; iRxIter < iRxBufSize; iRxIter++) {
    iDx = (iDx << 8) + iRx[iRxIter];
  }
  
  return iDx;
}

void MS5611_read_pt(struct spi_device *spi, volatile int32_t *pPress_Pa, volatile int32_t *pTemp_Deg) {
  uint32_t D1;  //ADC value of the pressure conversion
  uint32_t D2;  //ADC value of the temperature conversion
  int32_t dT;   //difference between actual and measured temp.
  int64_t OFF;  //offset at actual temperature
  int64_t SENS; //sensitivity at actual temperature
  int32_t T2;
  int64_t OFF2;
  int64_t SENS2;
  int64_t TEMP;
  
  D1 = MS5611_cnvrt_dx(spi, ADDR_CMD_CONVERT_D1);
  D2 = MS5611_cnvrt_dx(spi, ADDR_CMD_CONVERT_D2);
  
// Orig: dT    = D2 - _pCOEF[5] * pow(2, 8);
  dT    = D2 - _pCOEF[5] * (1 << 8);
// Orig: OFF   = _pCOEF[2] * pow(2,16) + (dT * _pCOEF[4]) / pow(2, 7);
  OFF   = _pCOEF[2] * (1 << 16) + (dT * _pCOEF[4]) / (1 << 7);
// Orig: SENS  = _pCOEF[1] * pow(2,15) + (dT * _pCOEF[3]) / pow(2, 8);
  SENS  = _pCOEF[1] * (1 << 15) + (dT * _pCOEF[3]) / (1 << 8);
//Orig: *pTemp_Deg = 2000 + dT * _pCOEF[6]/pow(2, 23);
  *pTemp_Deg = 2000 + dT * _pCOEF[6]/ (1 << 23);
  
  //SECOND ORDER TEMP COMPENSATION
  if(*pTemp_Deg < 2000) {
    T2    = (dT *dT) >> 31;
    TEMP  = *pTemp_Deg - 2000;
    OFF2  = (5 *  TEMP * TEMP) >> 1;
    SENS2 = OFF2 >> 1;
    if(*pTemp_Deg < -1500) {
      TEMP  = *pTemp_Deg + 1500;
      OFF2  = OFF2 + 7 * TEMP * TEMP;
      SENS2 = (SENS2 + (11 * TEMP * TEMP) ) >> 1;
    }
    *pTemp_Deg -= T2;
    OFF   -= OFF2;
    SENS  -= SENS2;
  }
// Orig: *pPress_Pa = (int32_t)((D1 * SENS / pow(2,21) - OFF)/pow(2,15) );
  *pPress_Pa = (int32_t)((D1 * SENS / (1 << 21) - OFF)/ (1 << 15) );
}

/////////////////////////////////////////////////////////////////////////////////////////
// MISC
/////////////////////////////////////////////////////////////////////////////////////////
int write_reg(struct spi_device *spi, uint8_t reg, uint8_t val) {
  uint8_t tx[2];

  tx[0] = reg;
  tx[1] = val;

  return spi_write(spi, tx, sizeof(tx));
}

ssize_t read_reg(struct spi_device *spi, uint8_t reg) {
  return spi_w8r8(spi, reg | 0x80); // 0x80 == 128
}

ssize_t read_mult_regs(struct spi_device *spi, uint8_t reg, uint8_t * rx_buf, size_t length) {
  reg = reg | 0x80;                 // 0x80 == 128
  return spi_write_then_read(spi, &reg, sizeof(uint8_t), rx_buf, length);
}

void wait_ms(uint16_t ms) {
  mdelay(ms * 1000);
}

uint8_t check_spi_write(int iRet) {
  if(iRet < 0) {
    printk(KERN_ERR "MS5611_Read_Prom(): Read from device failed\n");
    return 0;
  } 
  return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
// ENTRY AND EXIT POINTS
/////////////////////////////////////////////////////////////////////////////////////////
int MS5611_init(void) {
  int iRet;
  int iDevID;
  
  printk(KERN_ALERT "Init MS5611: Creating device file.\n");
  
  // Set _pCOEF to zero
  memset(_pCOEF, 0 , sizeof(_pCOEF) );

  INIT_KFIFO(_MS5611_REPORTS);
  iRet = spi_register_driver(&_MS5611_DRIVER);
  if(iRet < 0) {
    printk(KERN_ERR "Init MS5611: Failed to register ms5611 device driver - error %d\n", iRet);
    return iRet;
  }

  // Definition: int register_chrdev(unsigned int major, const char *name, struct file_operations *fops)
  iDevID = register_chrdev(MAJOR_DEVID, DEVICE_NAME, &_FOPS);
  if(iDevID < 0) {
    printk(KERN_ERR "Init MS5611: Failed to register ms5611 device file - error %d\n", iDevID);
    return iDevID;
  }
  
  _pMS5611_CLASS = class_create(THIS_MODULE, CLASS_NAME);
  if(IS_ERR(_pMS5611_CLASS) ) {
    printk(KERN_ERR "Init MS5611: Failed to register ms5611 device class '%s'\n", CLASS_NAME);
    return PTR_ERR(_pMS5611_CLASS);
  }
  
  _pMS5611_DEVICE = device_create(_pMS5611_CLASS, NULL, MKDEV(iDevID, 0), NULL, CLASS_NAME);
  if(IS_ERR(_pMS5611_DEVICE) ) {
    printk(KERN_ERR "Init MS5611: Failed to create device '%s'\n", CLASS_NAME);
  } else {  
    printk(KERN_ALERT "Init MS5611: Creating device file was successful.\n");
  }
  
  return PTR_ERR(_pMS5611_DEVICE);
}

void MS5611_exit(void) {
  printk(KERN_ALERT "Exit MS5611: Unregister device\n");
  unregister_chrdev(MAJOR_DEVID, DEVICE_NAME);
}

/////////////////////////////////////////////////////////////////////////////////////////
// GENERAL KERNEL DRIVER STUFF
/////////////////////////////////////////////////////////////////////////////////////////
module_init(MS5611_init);
module_exit(MS5611_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Frenzel <dgdanielf@gmail.com>");
MODULE_DESCRIPTION("Barometer driver"); 
