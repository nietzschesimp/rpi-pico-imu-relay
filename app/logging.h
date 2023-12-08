#ifndef LOGGING_H_
#define LOGGING_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <semphr.h>
#include <task.h>

enum log_level
{
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_TRACE,
  LOG_LEVEL_INFO,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_FATAL,
  LOG_LEVEL_MAX,
};

#define LOG_DEBUG(x) log_enqueue(LOG_LEVEL_DEBUG, x)
#define LOG_TRACE(x) log_enqueue(LOG_LEVEL_TRACE, x)
#define LOG_INFO(x) log_enqueue(LOG_LEVEL_INFO, x)
#define LOG_ERROR(x) log_enqueue(LOG_LEVEL_ERROR, x)
#define LOG_FATAL(x) log_enqueue(LOG_LEVEL_FATAL, x)

#define MAX_MSG_LEN 128
#define MSG_QUEUE_SIZE 4096
#define LOG_LEVEL LOG_LEVEL_DEBUG

void log_init();

void log_enqueue(unsigned char fmt, const char* msg);

#endif