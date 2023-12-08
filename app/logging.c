#include "logging.h"


typedef struct
{
  unsigned char level;
  char msg[MAX_MSG_LEN];
} log_event_t;


MessageBufferHandle_t msg_stream_handle = NULL;
SemaphoreHandle_t msg_lock = NULL;
unsigned char logging_level = LOG_LEVEL;


const char* get_fmt_from_level(unsigned char level)
{
  switch(level)
  {
    case LOG_LEVEL_DEBUG:
      return "[DEBUG] %s\n";
    case LOG_LEVEL_TRACE:
      return "[TRACE] %s\n";
    case LOG_LEVEL_INFO:
      return "[INFO] %s\n";
    case LOG_LEVEL_ERROR:
      return "[ERROR] %s\n";
    case LOG_LEVEL_FATAL:
      return "[FATAL] %s\n";
    default:
      return "";
  }
}

void log_task(void* unused)
{
  log_event_t log_event = {0};
  size_t msg_len = 0;
  const char* fmt;
  
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
    
    // Get the prefix
    fmt = get_fmt_from_level(log_event.level);
    if (strlen(fmt) < 1) {
      continue;
    }
    
    // Print the info
    printf(fmt, log_event.msg);
  }
}

inline void log_init()
{
  if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    printf("[FATAL] Scheduler has already started!\n");
  }
  
  msg_stream_handle = xMessageBufferCreate(MSG_QUEUE_SIZE);
  if(msg_stream_handle == NULL) {
    printf("[FATAL] Could not initialize message stream!\n");
    return;
  }
  
  msg_lock = xSemaphoreCreateBinary();
  
  BaseType_t created_task = xTaskCreate(log_task,
                                      "LOG_TASK",
                                      1024,
                                      NULL,
                                      9,
                                      NULL);
  if(!created_task) {
    printf("[FATAL] Could not initialize message stream!\n");
    vMessageBufferDelete(msg_stream_handle);
    vSemaphoreDelete(msg_lock);
    return;
  }
  printf("[INFO] Initialized logging!\n");
  xSemaphoreGive(msg_lock);
}

void log_enqueue(unsigned char level, const char* msg)
{
  // Check if logging level is valid
  if ((level < logging_level) || (level >= LOG_LEVEL_MAX)) {
    return;
  }
  
  // Enqueue the message to be logged
  log_event_t log_event = {0};
  size_t msg_len = strlen(msg);
  msg_len = (msg_len > MAX_MSG_LEN) ? MAX_MSG_LEN : msg_len;
  memcpy(log_event.msg, msg, msg_len);
  log_event.level = level;
  xSemaphoreTake(msg_lock, portMAX_DELAY);
  xMessageBufferSend(msg_stream_handle, &log_event, sizeof(log_event_t), portMAX_DELAY);
  xSemaphoreGive(msg_lock);
}
