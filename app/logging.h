#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <semphr.h>
#include <task.h>

#define MAX_MSG_LEN 128
#define MSG_QUEUE_SIZE 4096

enum log_level
{
  LL_DEBUG,
  TRACE,
  INFO,
  WARN,
  ERROR,
  MAX_LOG_LEVEL,
};

#define LOG_DEBUG(x) log_enqueue(DEBUG, x)
#define LOG_TRACE(x) log_enqueue(TRACE, x)
#define LOG_INFO(x) log_enqueue(INFO, x)
#define LOG_WARN(x) log_enqueue(WARN, x)
#define LOG_ERROR(x) log_enqueue(ERROR, x)

void log_init();

void log_enqueue(unsigned char fmt, const char* msg);

#endif