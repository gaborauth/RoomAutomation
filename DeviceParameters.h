/**
 * Device parameters.
 */
#define NUMBER_OF_DEVICES 1

typedef struct {
     String name;

     int pinLayout;

     int acEnabled;
     int deepSleepEnabled;
     int heatingEnabled;
     int pwmEnabled;
     int relayEnabled;

     int staticIpEnabled;
     int staticIp[4];
     int staticRoute[4];
     int staticMask[4];
} deviceParameters;

deviceParameters deviceParametersMap[NUMBER_OF_DEVICES] = {
    {"000000000000", 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, // Test-1
};
