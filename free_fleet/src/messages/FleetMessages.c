/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FleetMessages.c
  Source: FleetMessages.idl
  Cyclone DDS: V0.7.0

*****************************************************************/
#include "FleetMessages.h"


static const uint32_t FreeFleetData_RobotMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_RobotMode_desc =
{
  sizeof (FreeFleetData_RobotMode),
  4u,
  0u,
  0u,
  "FreeFleetData::RobotMode",
  NULL,
  2,
  FreeFleetData_RobotMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"RobotMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_DockMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DockMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_DockMode_desc =
{
  sizeof (FreeFleetData_DockMode),
  4u,
  0u,
  0u,
  "FreeFleetData::DockMode",
  NULL,
  2,
  FreeFleetData_DockMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"DockMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_Location_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_Location, obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_Location_desc =
{
  sizeof (FreeFleetData_Location),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::Location",
  NULL,
  9,
  FreeFleetData_Location_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"obey_approach_speed_limit\"><Boolean/></Member><Member name=\"approach_speed_limit\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_RobotState_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, model),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, task_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, mode.mode),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_RobotState, battery_percent),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_RobotState, location.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_RobotState, location.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_RobotState, location.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_RobotState, location.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_RobotState, location.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_RobotState, location.obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_RobotState, location.approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_RobotState, location.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof (FreeFleetData_RobotState, path),
  sizeof (FreeFleetData_Location), (21u << 16u) + 4u,
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_Location, obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS,
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_RobotState_desc =
{
  sizeof (FreeFleetData_RobotState),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::RobotState",
  NULL,
  25,
  FreeFleetData_RobotState_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"RobotMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"obey_approach_speed_limit\"><Boolean/></Member><Member name=\"approach_speed_limit\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"RobotState\"><Member name=\"name\"><String/></Member><Member name=\"model\"><String/></Member><Member name=\"task_id\"><String/></Member><Member name=\"mode\"><Type name=\"RobotMode\"/></Member><Member name=\"battery_percent\"><Float/></Member><Member name=\"location\"><Type name=\"Location\"/></Member><Member name=\"path\"><Sequence><Type name=\"Location\"/></Sequence></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_ModeParameter_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeParameter, name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeParameter, value),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_ModeParameter_desc =
{
  sizeof (FreeFleetData_ModeParameter),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::ModeParameter",
  NULL,
  3,
  FreeFleetData_ModeParameter_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"ModeParameter\"><Member name=\"name\"><String/></Member><Member name=\"value\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_ModeRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_ModeRequest, mode.mode),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeRequest, task_id),
  DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof (FreeFleetData_ModeRequest, parameters),
  sizeof (FreeFleetData_ModeParameter), (9u << 16u) + 4u,
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeParameter, name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_ModeParameter, value),
  DDS_OP_RTS,
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_ModeRequest_desc =
{
  sizeof (FreeFleetData_ModeRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::ModeRequest",
  NULL,
  10,
  FreeFleetData_ModeRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"RobotMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"ModeParameter\"><Member name=\"name\"><String/></Member><Member name=\"value\"><String/></Member></Struct><Struct name=\"ModeRequest\"><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"mode\"><Type name=\"RobotMode\"/></Member><Member name=\"task_id\"><String/></Member><Member name=\"parameters\"><Sequence><Type name=\"ModeParameter\"/></Sequence></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_PathRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_PathRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_PathRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof (FreeFleetData_PathRequest, path),
  sizeof (FreeFleetData_Location), (21u << 16u) + 4u,
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_Location, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_Location, nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_Location, obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_Location, approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_Location, level_name),
  DDS_OP_RTS,
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_PathRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_PathRequest_desc =
{
  sizeof (FreeFleetData_PathRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::PathRequest",
  NULL,
  15,
  FreeFleetData_PathRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"obey_approach_speed_limit\"><Boolean/></Member><Member name=\"approach_speed_limit\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"PathRequest\"><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"path\"><Sequence><Type name=\"Location\"/></Sequence></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_DestinationRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_DestinationRequest, destination.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DestinationRequest, destination.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DestinationRequest, destination.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DestinationRequest, destination.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DestinationRequest, destination.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_DestinationRequest, destination.obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DestinationRequest, destination.approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, destination.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DestinationRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_DestinationRequest_desc =
{
  sizeof (FreeFleetData_DestinationRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::DestinationRequest",
  NULL,
  12,
  FreeFleetData_DestinationRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"obey_approach_speed_limit\"><Boolean/></Member><Member name=\"approach_speed_limit\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"DestinationRequest\"><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"destination\"><Type name=\"Location\"/></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_DockRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DockRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DockRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_DockRequest, destination.sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DockRequest, destination.nanosec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DockRequest, destination.x),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DockRequest, destination.y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DockRequest, destination.yaw),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_DockRequest, destination.obey_approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DockRequest, destination.approach_speed_limit),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DockRequest, destination.level_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_DockRequest, task_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (FreeFleetData_DockRequest, dock_mode.mode),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_DockRequest, machine),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (FreeFleetData_DockRequest, distance_go_out),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (FreeFleetData_DockRequest, custom_docking),
  DDS_OP_ADR | DDS_OP_TYPE_2BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_DockRequest, rotate_to_dock),
  DDS_OP_ADR | DDS_OP_TYPE_2BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_DockRequest, rotate_angle),
  DDS_OP_ADR | DDS_OP_TYPE_2BY | DDS_OP_FLAG_SGN, offsetof (FreeFleetData_DockRequest, rotate_orientation),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_DockRequest_desc =
{
  sizeof (FreeFleetData_DockRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::DockRequest",
  NULL,
  19,
  FreeFleetData_DockRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"Location\"><Member name=\"sec\"><Long/></Member><Member name=\"nanosec\"><ULong/></Member><Member name=\"x\"><Float/></Member><Member name=\"y\"><Float/></Member><Member name=\"yaw\"><Float/></Member><Member name=\"obey_approach_speed_limit\"><Boolean/></Member><Member name=\"approach_speed_limit\"><Float/></Member><Member name=\"level_name\"><String/></Member></Struct><Struct name=\"DockMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"DockRequest\"><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"destination\"><Type name=\"Location\"/></Member><Member name=\"task_id\"><String/></Member><Member name=\"dock_mode\"><Type name=\"DockMode\"/></Member><Member name=\"machine\"><Boolean/></Member><Member name=\"distance_go_out\"><Float/></Member><Member name=\"custom_docking\"><Boolean/></Member><Member name=\"rotate_to_dock\"><Short/></Member><Member name=\"rotate_angle\"><Short/></Member><Member name=\"rotate_orientation\"><Short/></Member></Struct></Module></MetaData>"
};


static const uint32_t FreeFleetData_CancelRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_CancelRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_CancelRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (FreeFleetData_CancelRequest, task_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t FreeFleetData_CancelRequest_desc =
{
  sizeof (FreeFleetData_CancelRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "FreeFleetData::CancelRequest",
  NULL,
  4,
  FreeFleetData_CancelRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"FreeFleetData\"><Struct name=\"CancelRequest\"><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"task_id\"><String/></Member></Struct></Module></MetaData>"
};
