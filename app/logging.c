#include "logging.h"
#include <stdarg.h>

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YLW   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

typedef struct
{
  unsigned char level;
  const char* file;
  int line;
  const char* fmt;
  va_list args;
} log_event_t;


MessageBufferHandle_t msg_stream_handle = NULL;
SemaphoreHandle_t msg_lock = NULL;
unsigned char logging_level = LOG_LEVEL;


/**
 * \brief Helper function to grab a string depending on the event logging level
 * \param level   The logging level of the event
 * \return pointer to string pertaning to the formatting of the given log level
 */
const char* get_hdr_fmt_from_level(unsigned char level)
{
  switch(level)
  #if LOG_USE_COLOR
  {
    case LOG_LEVEL_DEBUG:
      return CYN "DEBUG" RESET " [%s:%d] | ";
    case LOG_LEVEL_TRACE:
      return MAG "TRACE" RESET " [%s:%d] | ";
    case LOG_LEVEL_INFO:
      return GRN "INFO" RESET "  [%s:%d] | ";
    case LOG_LEVEL_ERROR:
      return YLW "ERROR" RESET " [%s:%d] | ";
    case LOG_LEVEL_FATAL:
      return RED "FATAL" RESET " [%s:%d] | ";
    default:
      return "";
  }
  #else
  {
    case LOG_LEVEL_DEBUG:
      return "DEBUG [%s:%d] | ";
    case LOG_LEVEL_TRACE:
      return "TRACE [%s:%d] | ";
    case LOG_LEVEL_INFO:
      return "INFO  [%s:%d] | ";
    case LOG_LEVEL_ERROR:
      return "ERROR [%s:%d] | ";
    case LOG_LEVEL_FATAL:
      return "FATAL [%s:%d] | ";
    default:
      return "";
  }
  #endif
}


/**
 * \brief FreeRTOS task that will receive logging events, format them, and print them appropriately
 */
void log_task(void* unused)
{
  log_event_t log_event = {0};
  size_t msg_len = 0;
  const char* hdr;
  
  while(1) {
    msg_len = xMessageBufferReceive(msg_stream_handle,
                                    &log_event,
                                    sizeof(log_event_t),
                                    portMAX_DELAY);
    if (msg_len < 1) {
      // Didn't sucessfully receive a message from the queue, skip
      printf("[FATAL] Failed to receive logging info");
      continue;
    }
    
    hdr = get_hdr_fmt_from_level(log_event.level);
    printf(hdr, log_event.file, log_event.line);
    vprintf(log_event.fmt, log_event.args);
    printf("\n");
    va_end(log_event.args);
  }
}


/**
 * \brief Entry point to initialize the FreeRTOS task that will listen to
 *        logging events from other tasks to log to console.
 */
void log_task_init()
{
  if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    printf("Scheduler has already started!\n");
  }
  
  msg_stream_handle = xMessageBufferCreate(MSG_QUEUE_SIZE);
  if(msg_stream_handle == NULL) {
    printf("Could not initialize message stream!\n");
    return;
  }
  
  msg_lock = xSemaphoreCreateBinary();
  
  BaseType_t created_task = xTaskCreate(log_task,
                                      "LOG_TASK",
                                      1024,
                                      NULL,
                                      1,
                                      NULL);
  if(!created_task) {
    printf("Could not initialize message stream!\n");
    vMessageBufferDelete(msg_stream_handle);
    vSemaphoreDelete(msg_lock);
    return;
  }
  xSemaphoreGive(msg_lock);
}


/**
 * \brief Function to enqueue a message to be logged later.
 * \param level   Logging level of the message
 * \param msg     Pointer to the string to log
 */
void log_task_enqueue(unsigned char level, const char* file, int line, const char* fmt, ...)
{
  // Check if logging level is valid
  if ((level < logging_level) || (level >= LOG_LEVEL_MAX)) {
    return;
  }

  // Populate log event
  log_event_t log_event = {
      .level = level,
      .file = file,
      .line = line,
      .fmt = fmt
  };

  // Load arguments into log event
  va_list local_args;
  va_start(local_args, fmt);
  va_copy(log_event.args, local_args);
  va_end(local_args);

  // Enqueue the message to be logged
  xSemaphoreTake(msg_lock, portMAX_DELAY);
  xMessageBufferSend(msg_stream_handle, &log_event, sizeof(log_event_t), portMAX_DELAY);
  xSemaphoreGive(msg_lock);
}
