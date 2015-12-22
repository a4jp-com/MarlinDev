#if ENABLED(ULTIPANEL)
  #define plaPreheatHotendTemp PreheatPLA.HotendTemp
  #define plaPreheatHPBTemp PreheatPLA.HPBTemp
  #define plaPreheatFanSpeed PreheatPLA.FanSpeed
  #define absPreheatHotendTemp PreheatABS.HotendTemp
  #define absPreheatHPBTemp PreheatABS.HPBTemp
  #define absPreheatFanSpeed PreheatABS.FanSpeed

  typedef struct {
    int HotendTemp;
    int HPBTemp;
    int FanSpeed;
    } PreheatStruct;

  extern PreheatStruct PreheatPLA;
  extern PreheatStruct PreheatABS;

#endif