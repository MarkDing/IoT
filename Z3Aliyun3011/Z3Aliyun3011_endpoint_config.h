// This file is generated by Simplicity Studio.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef SILABS_AF_ENDPOINT_CONFIG
#define SILABS_AF_ENDPOINT_CONFIG


// Fixed number of defined endpoints
#define FIXED_ENDPOINT_COUNT (1)




// Generated attributes
#define GENERATED_ATTRIBUTES { \
    { 0x0000, ZCL_INT8U_ATTRIBUTE_TYPE, 1, (ATTRIBUTE_MASK_SINGLETON), { (uint8_t*)0x03 } }, /* 0 / Basic / ZCL version*/\
    { 0x0007, ZCL_ENUM8_ATTRIBUTE_TYPE, 1, (ATTRIBUTE_MASK_SINGLETON), { (uint8_t*)0x00 } }, /* 1 / Basic / power source*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_SINGLETON), { (uint8_t*)0x0001 } }, /* 2 / Basic / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 3 / Identify / cluster revision*/\
    { 0x0000, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_WRITABLE), { (uint8_t*)0x0000 } }, /* 4 / Identify / identify time*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (0x00), { (uint8_t*)0x0001 } }, /* 5 / Identify / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 6 / Groups / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 7 / Scenes / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 8 / On/off / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 9 / Level Control / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 10 / Shade Configuration / cluster revision*/\
    { 0xFFFD, ZCL_INT16U_ATTRIBUTE_TYPE, 2, (ATTRIBUTE_MASK_CLIENT), { (uint8_t*)0x0001 } }, /* 11 / Color Control / cluster revision*/\
  }


// Cluster function static arrays
#define GENERATED_FUNCTION_ARRAYS \
const EmberAfGenericClusterFunction emberAfFuncArrayIdentifyClusterServer[] = { (EmberAfGenericClusterFunction)emberAfIdentifyClusterServerInitCallback,(EmberAfGenericClusterFunction)emberAfIdentifyClusterServerAttributeChangedCallback}; \


// Clusters definitions
#define GENERATED_CLUSTERS { \
    { 0x0000, (EmberAfAttributeMetadata*)&(generatedAttributes[0]), 3, 0, (CLUSTER_MASK_SERVER), NULL,  },    \
    { 0x0003, (EmberAfAttributeMetadata*)&(generatedAttributes[3]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0003, (EmberAfAttributeMetadata*)&(generatedAttributes[4]), 2, 4, (CLUSTER_MASK_SERVER| CLUSTER_MASK_INIT_FUNCTION| CLUSTER_MASK_ATTRIBUTE_CHANGED_FUNCTION), emberAfFuncArrayIdentifyClusterServer, },    \
    { 0x0004, (EmberAfAttributeMetadata*)&(generatedAttributes[6]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0005, (EmberAfAttributeMetadata*)&(generatedAttributes[7]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0006, (EmberAfAttributeMetadata*)&(generatedAttributes[8]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0008, (EmberAfAttributeMetadata*)&(generatedAttributes[9]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0100, (EmberAfAttributeMetadata*)&(generatedAttributes[10]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
    { 0x0300, (EmberAfAttributeMetadata*)&(generatedAttributes[11]), 1, 2, (CLUSTER_MASK_CLIENT), NULL,  },    \
  }


// Endpoint types
#define GENERATED_ENDPOINT_TYPES {        \
    { (EmberAfCluster*)&(generatedClusters[0]), 9, 18 }, \
  }


// Networks
#define EM_AF_GENERATED_NETWORK_TYPES { \
  EM_AF_NETWORK_TYPE_ZIGBEE_PRO, /* Primary */ \
}
#define EM_AF_GENERATED_ZIGBEE_PRO_NETWORKS { \
  { \
    /* Primary */ \
    ZA_COORDINATOR, \
    EMBER_AF_SECURITY_PROFILE_Z3, \
  }, \
}
#define EM_AF_GENERATED_NETWORK_STRINGS  \
  "Primary (pro)", \


// Cluster manufacturer codes
#define GENERATED_CLUSTER_MANUFACTURER_CODES {      \
{0x00, 0x00} \
  }
#define GENERATED_CLUSTER_MANUFACTURER_CODE_COUNT (0)

// Attribute manufacturer codes
#define GENERATED_ATTRIBUTE_MANUFACTURER_CODES {      \
{0x00, 0x00} \
  }
#define GENERATED_ATTRIBUTE_MANUFACTURER_CODE_COUNT (0)


// Largest attribute size is needed for various buffers
#define ATTRIBUTE_LARGEST (2)
// Total size of singleton attributes
#define ATTRIBUTE_SINGLETONS_SIZE (4)

// Total size of attribute storage
#define ATTRIBUTE_MAX_SIZE 18

// Array of endpoints that are supported
#define FIXED_ENDPOINT_ARRAY { 1 }

// Array of profile ids
#define FIXED_PROFILE_IDS { 260 }

// Array of device ids
#define FIXED_DEVICE_IDS { 0 }

// Array of device versions
#define FIXED_DEVICE_VERSIONS { 0 }

// Array of endpoint types supported on each endpoint
#define FIXED_ENDPOINT_TYPES { 0 }

// Array of networks supported on each endpoint
#define FIXED_NETWORKS { 0 }


// Code used to configure the cluster event mechanism
#define EMBER_AF_GENERATED_EVENT_CODE \
  EmberEventControl emberAfIdentifyClusterServerTickCallbackControl1; \
  extern EmberEventControl emberAfPluginConcentratorUpdateEventControl; \
  extern EmberEventControl emberAfPluginDeviceTableNewDeviceEventControl; \
  extern EmberEventControl emberAfPluginFormAndJoinCleanupEventControl; \
  extern EmberEventControl emberAfPluginNetworkCreatorSecurityOpenNetworkEventControl; \
  extern EmberEventControl emberAfPluginScanDispatchScanEventControl; \
  extern void emberAfPluginConcentratorUpdateEventHandler(void); \
  extern void emberAfPluginDeviceTableNewDeviceEventHandler(void); \
  extern void emberAfPluginFormAndJoinCleanupEventHandler(void); \
  extern void emberAfPluginNetworkCreatorSecurityOpenNetworkEventHandler(void); \
  extern void emberAfPluginScanDispatchScanEventHandler(void); \
  extern EmberEventControl addSubDevEventControl; \
  extern EmberEventControl clearWiFiEventControl; \
  extern EmberEventControl formNetworkRetryEventControl; \
  extern EmberEventControl pollAttrEventControl; \
  extern void addSubDevEventHandler(void); \
  extern void clearWiFiEventHandler(void); \
  extern void formNetworkRetryEventHandler(void); \
  extern void pollAttrEventHandler(void); \
  static void clusterTickWrapper(EmberEventControl *control, EmberAfTickFunction callback, uint8_t endpoint) \
  { \
    emberAfPushEndpointNetworkIndex(endpoint); \
    emberEventControlSetInactive(*control); \
    (*callback)(endpoint); \
    emberAfPopNetworkIndex(); \
  } \
  void emberAfIdentifyClusterServerTickCallbackWrapperFunction1(void) { clusterTickWrapper(&emberAfIdentifyClusterServerTickCallbackControl1, emberAfIdentifyClusterServerTickCallback, 1); } \


// EmberEventData structs used to populate the EmberEventData table
#define EMBER_AF_GENERATED_EVENTS   \
  { &emberAfIdentifyClusterServerTickCallbackControl1, emberAfIdentifyClusterServerTickCallbackWrapperFunction1 }, \
  { &emberAfPluginConcentratorUpdateEventControl, emberAfPluginConcentratorUpdateEventHandler }, \
  { &emberAfPluginDeviceTableNewDeviceEventControl, emberAfPluginDeviceTableNewDeviceEventHandler }, \
  { &emberAfPluginFormAndJoinCleanupEventControl, emberAfPluginFormAndJoinCleanupEventHandler }, \
  { &emberAfPluginNetworkCreatorSecurityOpenNetworkEventControl, emberAfPluginNetworkCreatorSecurityOpenNetworkEventHandler }, \
  { &emberAfPluginScanDispatchScanEventControl, emberAfPluginScanDispatchScanEventHandler }, \
  { &addSubDevEventControl, addSubDevEventHandler }, \
  { &clearWiFiEventControl, clearWiFiEventHandler }, \
  { &formNetworkRetryEventControl, formNetworkRetryEventHandler }, \
  { &pollAttrEventControl, pollAttrEventHandler }, \


#define EMBER_AF_GENERATED_EVENT_STRINGS   \
  "Identify Cluster Server EP 1",  \
  "Concentrator Support Plugin Update",  \
  "Device Table Plugin NewDevice",  \
  "Form and Join Library Plugin Cleanup",  \
  "Network Creator Security Plugin OpenNetwork",  \
  "Scan Dispatch Plugin Scan",  \
  "addSubDev Custom",  \
  "clearWiFi Custom",  \
  "formNetworkRetry Custom",  \
  "pollAttr Custom",  \


// The length of the event context table used to track and retrieve cluster events
#define EMBER_AF_EVENT_CONTEXT_LENGTH 1

// EmberAfEventContext structs used to populate the EmberAfEventContext table
#define EMBER_AF_GENERATED_EVENT_CONTEXT { 0x1, 0x3, false, EMBER_AF_LONG_POLL, EMBER_AF_OK_TO_SLEEP, &emberAfIdentifyClusterServerTickCallbackControl1}


#define EMBER_AF_GENERATED_PLUGIN_INIT_FUNCTION_DECLARATIONS \
  void emberAfPluginDeviceTableInitCallback(void); \
  void emberAfPluginConcentratorInitCallback(void); \
  void emberAfPluginIdleSleepInitCallback(void); \
  void emberAfPluginCountersInitCallback(void); \


#define EMBER_AF_GENERATED_PLUGIN_INIT_FUNCTION_CALLS \
  emberAfPluginDeviceTableInitCallback(); \
  emberAfPluginConcentratorInitCallback(); \
  emberAfPluginIdleSleepInitCallback(); \
  emberAfPluginCountersInitCallback(); \


#define EMBER_AF_GENERATED_PLUGIN_NCP_INIT_FUNCTION_DECLARATIONS \
  void emberAfPluginConcentratorNcpInitCallback(bool memoryAllocation); \
  void emberAfPluginAddressTableNcpInitCallback(bool memoryAllocation); \


#define EMBER_AF_GENERATED_PLUGIN_NCP_INIT_FUNCTION_CALLS \
  emberAfPluginConcentratorNcpInitCallback(memoryAllocation); \
  emberAfPluginAddressTableNcpInitCallback(memoryAllocation); \


#define EMBER_AF_GENERATED_PLUGIN_STACK_STATUS_FUNCTION_DECLARATIONS \
  void emberAfPluginDeviceTableStackStatusCallback(EmberStatus status); \
  void emberAfPluginConcentratorStackStatusCallback(EmberStatus status); \
  void emberAfPluginNetworkCreatorSecurityStackStatusCallback(EmberStatus status); \


#define EMBER_AF_GENERATED_PLUGIN_STACK_STATUS_FUNCTION_CALLS \
  emberAfPluginDeviceTableStackStatusCallback(status); \
  emberAfPluginConcentratorStackStatusCallback(status); \
  emberAfPluginNetworkCreatorSecurityStackStatusCallback(status); \


#define EMBER_AF_GENERATED_PLUGIN_MESSAGE_SENT_FUNCTION_DECLARATIONS \
  void emberAfPluginConcentratorMessageSentCallback(EmberOutgoingMessageType type, \
                    uint16_t indexOrDestination, \
                    EmberApsFrame *apsFrame, \
                    EmberStatus status, \
                    uint16_t messageLength, \
                    uint8_t *messageContents); \


#define EMBER_AF_GENERATED_PLUGIN_MESSAGE_SENT_FUNCTION_CALLS \
  emberAfPluginConcentratorMessageSentCallback(type, \
                    indexOrDestination, \
                    apsFrame, \
                    status, \
                    messageLength, \
                    messageContents); \

// Generated data for the command discovery
#define GENERATED_COMMANDS { \
    { 0x0003, 0x00, COMMAND_MASK_OUTGOING_CLIENT | COMMAND_MASK_INCOMING_SERVER }, /* Identify / Identify */ \
    { 0x0003, 0x00, COMMAND_MASK_OUTGOING_SERVER }, /* Identify / IdentifyQueryResponse */ \
    { 0x0003, 0x01, COMMAND_MASK_OUTGOING_CLIENT | COMMAND_MASK_INCOMING_SERVER }, /* Identify / IdentifyQuery */ \
    { 0x0004, 0x00, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / AddGroup */ \
    { 0x0004, 0x01, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / ViewGroup */ \
    { 0x0004, 0x02, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / GetGroupMembership */ \
    { 0x0004, 0x03, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / RemoveGroup */ \
    { 0x0004, 0x04, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / RemoveAllGroups */ \
    { 0x0004, 0x05, COMMAND_MASK_OUTGOING_CLIENT }, /* Groups / AddGroupIfIdentifying */ \
    { 0x0005, 0x00, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / AddScene */ \
    { 0x0005, 0x01, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / ViewScene */ \
    { 0x0005, 0x02, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / RemoveScene */ \
    { 0x0005, 0x03, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / RemoveAllScenes */ \
    { 0x0005, 0x04, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / StoreScene */ \
    { 0x0005, 0x05, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / RecallScene */ \
    { 0x0005, 0x06, COMMAND_MASK_OUTGOING_CLIENT }, /* Scenes / GetSceneMembership */ \
    { 0x0006, 0x00, COMMAND_MASK_OUTGOING_CLIENT }, /* On/off / Off */ \
    { 0x0006, 0x01, COMMAND_MASK_OUTGOING_CLIENT }, /* On/off / On */ \
    { 0x0006, 0x02, COMMAND_MASK_OUTGOING_CLIENT }, /* On/off / Toggle */ \
    { 0x0008, 0x00, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / MoveToLevel */ \
    { 0x0008, 0x01, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / Move */ \
    { 0x0008, 0x02, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / Step */ \
    { 0x0008, 0x03, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / Stop */ \
    { 0x0008, 0x04, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / MoveToLevelWithOnOff */ \
    { 0x0008, 0x05, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / MoveWithOnOff */ \
    { 0x0008, 0x06, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / StepWithOnOff */ \
    { 0x0008, 0x07, COMMAND_MASK_OUTGOING_CLIENT }, /* Level Control / StopWithOnOff */ \
    { 0x0300, 0x07, COMMAND_MASK_OUTGOING_CLIENT }, /* Color Control / MoveToColor */ \
    { 0x0300, 0x08, COMMAND_MASK_OUTGOING_CLIENT }, /* Color Control / MoveColor */ \
    { 0x0300, 0x09, COMMAND_MASK_OUTGOING_CLIENT }, /* Color Control / StepColor */ \
    { 0x0300, 0x0A, COMMAND_MASK_OUTGOING_CLIENT }, /* Color Control / MoveToColorTemperature */ \
  }
#define EMBER_AF_GENERATED_COMMAND_COUNT (31)

// Command manufacturer codes
#define GENERATED_COMMAND_MANUFACTURER_CODES {      \
{0x00, 0x00} \
  }
#define GENERATED_COMMAND_MANUFACTURER_CODE_COUNT (0)


// Generated reporting configuration defaults
#define EMBER_AF_GENERATED_REPORTING_CONFIG_DEFAULTS {\
}
#define EMBER_AF_GENERATED_REPORTING_CONFIG_DEFAULTS_TABLE_SIZE (0)
#endif // SILABS_AF_ENDPOINT_CONFIG