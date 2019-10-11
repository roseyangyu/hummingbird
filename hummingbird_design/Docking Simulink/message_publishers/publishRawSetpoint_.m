function published = publishRawSetpoint_(pos, vel, yaw, pub, msg)
    msg.Position.X = pos(1);
    msg.Position.Y = pos(2);
    msg.Position.Z = pos(3);
    msg.Velocity.X = vel(1);
    msg.Velocity.Y = vel(2);
    msg.Velocity.Z = vel(3);
    msg.Yaw = yaw;
    send(pub, msg);
    published = 1;
end
%{
mavros_msgs/PositionTarget definition

uint8 FRAME_LOCAL_NED=1
uint8 FRAME_LOCAL_OFFSET_NED=7
uint8 FRAME_BODY_NED=8
uint8 FRAME_BODY_OFFSET_NED=9
uint16 IGNORE_PX=1
uint16 IGNORE_PY=2
uint16 IGNORE_PZ=4
uint16 IGNORE_VX=8
uint16 IGNORE_VY=16
uint16 IGNORE_VZ=32
uint16 IGNORE_AFX=64
uint16 IGNORE_AFY=128
uint16 IGNORE_AFZ=256
uint16 FORCE=512
uint16 IGNORE_YAW=1024
uint16 IGNORE_YAW_RATE=2048
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 coordinate_frame
uint16 type_mask
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 velocity
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 acceleration_or_force
  float64 x
  float64 y
  float64 z
float32 yaw
float32 yaw_rate
%}