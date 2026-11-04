#ifndef WIRING_CHECK_H
#define WIRING_CHECK_H

/* Tests every camera and DHT11 wire and prints a report on the serial port.
 * Leaves the camera powered down. Returns the number of problems found. */
int wiring_check_run(void);

#endif /* WIRING_CHECK_H */
