#include "logging.h"


typedef struct
{
  unsigned char level;
  char msg[MAX_MSG_LEN];
} log_event_t;


MessageBufferHandle_t msg_stream_handle = NULL;
SemaphoreHandle_t msg_lock = NULL;
unsigned char logging_level = LOG_LEVEL;


/**
 * \brief Helper function to grab a string depending on the event logging level
 * \param level   The logging level of the event
 * \return pointer to string pertaning to the formatting of the given log level
 */
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


/**
 * \brief FreeRTOS task that will receive logging events, format them, and print them appropriately
 */
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


/**
 * \brief Entry point to initialize the FreeRTOS task that will listen to
 *        logging events from other tasks to log to console.
 */
void log_task_init()
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
                                      1,
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


/**
 * \brief Function to enqueue a message to be logged later.
 * \param level   Logging level of the message
 * \param msg     Pointer to the string to log
 */
void log_task_enqueue(unsigned char level, const char* msg)
{
  // Check if logging level is valid
  if ((level < logging_level) || (level >= LOG_LEVEL_MAX)) {
    return;
  }
  
  // Check the message length
  size_t msg_len = strlen(msg);
  if (msg_len > MAX_MSG_LEN) {
    msg_len = MAX_MSG_LEN;
  }

  // Populate log event
  log_event_t log_event = {0};
  memcpy(log_event.msg, msg, msg_len);
  log_event.level = level;

  // Enqueue the message to be logged
  xSemaphoreTake(msg_lock, portMAX_DELAY);
  xMessageBufferSend(msg_stream_handle, &log_event, sizeof(log_event_t), portMAX_DELAY);
  xSemaphoreGive(msg_lock);
}
