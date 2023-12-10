#ifndef LOGGING_H_
#define LOGGING_H_

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

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

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_DEBUG(...) log_task_enqueue(LOG_LEVEL_DEBUG, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_TRACE(...) log_task_enqueue(LOG_LEVEL_TRACE, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) log_task_enqueue(LOG_LEVEL_INFO, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) log_task_enqueue(LOG_LEVEL_ERROR, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...) log_task_enqueue(LOG_LEVEL_FATAL, __FILENAME__, __LINE__, __VA_ARGS__)

#define MSG_QUEUE_SIZE 1024
#define LOG_LEVEL LOG_LEVEL_DEBUG
#define LOG_USE_COLOR 1

void log_task_init();

void log_task_enqueue(unsigned char level, const char* file, int line, const char* fmt, ...);

#endif