/**
 * Device parameters.
 */
#define NUMBER_OF_DEVICES 11

typedef struct {
     String name;

     int pinLayout;

     int acEnabled;
     int deepSleepEnabled;
     int heatingEnabled;
     int pwmEnabled;
     int relayEnabled;
} deviceParameters;

deviceParameters deviceParametersMap[NUMBER_OF_DEVICES] = {
    {"000000000000", 0, 0, 0, 0, 0, 0}, // Test
};
