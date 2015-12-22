#include "MarlinFirmware.h"
#include "thermal/preheat.h"

#if ENABLED(ULTIPANEL)
  PreheatStruct PreheatPLA;
  PreheatStruct PreheatABS;
#endif