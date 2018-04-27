/**
 * Device parameters.
 */
#define NUMBER_OF_DEVICES 1
#define NUMBER_OF_NODES 1

typedef struct {
     String name;
     String deviceId;

     int pinLayout;

     int acEnabled;
     int deepSleepEnabled;
     int heatingEnabled;
     int pwmEnabled;
     int relayEnabled;
     int ventilationEnabled;

     int staticIpEnabled;
     int staticIp[4];
     int staticRoute[4];
     int staticMask[4];
} deviceParameters;

typedef struct {
     String name;
     String nodeId;
     String nodeKey;
     String deviceId;
} nodeParameters;

deviceParameters deviceParametersMap[NUMBER_OF_DEVICES] = {
    {"test", "61d315e0-46c7-11e8-a240-7bca10e5bd5b", 0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, // Test-1
};

nodeParameters nodeParametersMap[NUMBER_OF_NODES] = {
    {"test", "97ac1190-1094-11e7-bb99-dd93c17d8a4d", "mdXaRa6IbdJODwy3yolFKQ", "61d315e0-46c7-11e8-a240-7bca10e5bd5b"}, // Test-1
};

