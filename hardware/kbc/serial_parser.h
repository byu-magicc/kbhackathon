//////////////////////////////////
/// SERIAL PARSER
//////////////////////////////////
#define START_BYTE 0x2A
#define MAX_LEN 12

enum
{
  TYPE_COMMAND,
  TYPE_SENSOR,
};

typedef struct __attribute__((packed))
{
  float thr; // commanded throttle
  float steer; // commanded steering
} cmd_t;

typedef struct __attribute__((packed))
{
  float rc_thr; // current RC throttle value (0-1)
  float rc_steer; // current steering signal (0-1)
  float distance; // distance traveled in (m)
  float velocity; // current estimate of velocity (m/s)
  float sonar; // distance read by sonar (m)
} sens_t;

typedef struct __attribute__((packed))
{
  uint8_t type;
  uint8_t len; // length of the payload i.e. (sizeof(cmd_t))
  union payload 
  {
    cmd_t cmd;
    sens_t sens;
    uint8_t buf[MAX_LEN];
  } dat;
} message_t;

enum Com_State{ 
  S_START,
  S_TYPE,
  S_LEN,
  S_BUFFER,
  S_END
};

message_t in_message;

// this creates a byte-wise xor checksum to be sent with the
// data packet for error checking
uint8_t calcChecksum(message_t* msg)
{
  uint8_t crc = 0x0;
  uint8_t* buf = (uint8_t*)msg;

  for (int i = 0; i < 1 + msg->len; i++) // 3 bytes for the header, plus the length of the message
  {
    crc ^= buf[i];  // XOR each byte in the message until we get to the crc
  }
  return crc;
}

void parseMsg(uint8_t data) {
  static Com_State state = S_START;
  static String val_str;
  static uint8_t i = 0;
  switch(state) 
  {
    case S_START:
      if (data == START_BYTE)
      {
        memset(&in_message, 0, sizeof(message_t));
        state = S_TYPE;
      }
      break;

    case S_TYPE:
      in_message.type = data;
      state = S_LEN;
      break;

    case S_LEN:
      if (data <= sizeof(message_t::dat))
      {
        // make sure that this is a valid message, so we aren't clocking in bytes forever
        in_message.len = data;
        state = S_BUFFER;
      }
      else
      {
        state = S_START;
      }
      break;

    case S_BUFFER:
      if (i < in_message.len)
      {
        in_message.dat.buf[i] = data;
        i++;
        if (i == in_message.len)
        {
          state = S_END;
        }
      }
      break;

    case S_END:
      if (calcChecksum(&in_message) == data)
      {
        switch(in_message.type)
        {
          case TYPE_COMMAND:
            command_callback();
            break;
          case TYPE_SENSOR:
            sensor_callback();
            break;
          default:
            break;
        }
      }
      state = S_START;
      break;

    default:
      state = S_START;
  }
}