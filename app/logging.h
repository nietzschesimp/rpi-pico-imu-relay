#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/**
 * @brief Generate and print a debug message from a supplied string.
 * @param msg: The base message to which `[DEBUG]` will be prefixed.
 */
inline void log_debug(const char* msg)
{
#ifdef DEBUG
  unsigned long msg_length = 9 + strlen(msg);
  char* sprintf_buffer = malloc(msg_length);
  sprintf(sprintf_buffer, "[DEBUG] %s\n", msg);
  printf("%s", sprintf_buffer);
  free(sprintf_buffer);
#endif
}

/**
 * @brief Generate and print an error message from a supplied string.
 * @param msg: The base message to which `[ERROR]` will be prefixed.
 */
inline void log_error(const char* msg)
{
  unsigned long msg_length = 9 + strlen(msg);
  char* sprintf_buffer = malloc(msg_length);
  sprintf(sprintf_buffer, "[ERROR] %s\n", msg);
  printf("%s", sprintf_buffer);
  free(sprintf_buffer);
}

#endif