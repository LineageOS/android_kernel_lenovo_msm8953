#ifndef _COULO_METER
#define _COULO_METER
int cm_get_ParameterVersion(void);
int cm_get_ID1(void);
int cm_get_ID2(void);
int cm_get_Temperature(void);
int cm_get_Voltage(void);
int cm_get_NominalAvailableCapacity(void);
int cm_get_RemainingCapacity(void);
int cm_get_FullChargeCapacity(void);
int cm_get_AverageTimeToEmpty(void);
int cm_get_AverageCurrent(void);
int cm_get_AverageTimeToFull(void);
int cm_get_InternalTemperature(void);
int cm_get_StateOfCharge(void);
int cm_get_StateOfHealth(void);
extern int g_power_is_couloMeter;/* Modify by lichuangchuang for battery debug (8909) SW00131408 20150531  */
#endif
