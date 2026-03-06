/*
 * Automatic per-flight state logger.
 *
 * Creates a new CSV file for each motors-on session and logs state + commands.
 * */

#ifndef FLIGHT_LOGGER__1H
#define FLIGHT_LOGGER__1H

extern void flight_logger_1_init(void);
extern void flight_logger_1_periodic(void);

#endif /* FLIGHT_LOGGER__1H */