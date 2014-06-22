#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>


typedef struct MS5611Report {
  struct timeval timestamp;
  uint32_t error_count;

  int32_t pressure_Pa;
  int32_t temperature_Deg;
} MS5611Report;

double calc_height(MS5611Report report) {
  double fHeight_m;

  /* tropospheric properties (0-11km) for standard atmosphere */
  const double Tzero_K = 15.0 + 273.15;	  /* temperature at base height in Kelvin */
  const double a  = -6.5 / 1000;	        /* temperature gradient in degrees per metre */
  const double g  = 9.80665;	            /* gravity constant in m/s/s */
  const double R  = 287.05;	              /* ideal gas constant in J/kg/K */

  /* current pressure at MSL in kPa */
  double pMSL_kPa = _msl_pressure / 1000.0;

  /* measured pressure in kPa */
  double p_kPa = report.pressure_Pa / 1000.0; 

  /*
   * Solve:
   *
   *     /        -(aR / g)     \
   *    | (p_kPa / pMSL_kPa)          . Tzero_K | - Tzero_K
   *     \                      /
   * h = -------------------------------  + h1
   *                   a
   */
  fHeight_m = (((pow((p_kPa / pMSL_kPa), (-(a * R) / g) ) ) * Tzero_K) - Tzero_K) / a;
  return fHeight_m;
}

int main(int argc, char* argv[]) {
  int input_fd;
  MS5611Report buffer;

  input_fd = open ("/dev/ms5611", O_RDONLY);

  if (input_fd == -1) {
    perror ("open");
    return 2;
  }

  while(1) {
    read(input_fd, &buffer, sizeof(MS5611Report) );
    
		printf("%10d %10d %.4f | %ld.%ld\n",
            buffer.pressure_Pa,
            buffer.temperature_Deg,
            calc_height(buffer),
            buffer.timestamp.tv_sec,
            buffer.timestamp.tv_usec);
  }
}