#define MAX_NUM_NEIGHBORS 10
#define SHARING_TIME 20

//PAYLOAD
#define MSG 0
#define ID 1
#define RIGHT_ID  2
#define LEFT_ID  3
#define STATE  4

#define RECEIVER 5
#define SENDER 6
#define COLOR 7


#define ACTIVE 0




#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif

typedef enum { NULL_MSG,
    SHARE,
    JOIN,
    LEAVE,
    MOVE
} message_type;  // MESSAGES

typedef enum {
    AUTONOMOUS,
    COOPERATIVE
} robot_state;  // STATE


// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

typedef struct{
    uint8_t id;
    uint8_t right_id;
    uint8_t left_id;
    robot_state state;
    uint8_t distance;
    uint8_t num;
    uint8_t num_cooperative;

} nearest_neighbor_t;

typedef struct  {
    uint8_t motion;
    uint8_t time;
} motion_time_t;


typedef struct
{
    uint8_t my_id;
    uint8_t my_right;
    uint8_t my_left;
    message_t msg;
    robot_state state;
    
    // Tejas added these
    uint8_t send_election;
    uint8_t my_leader;
    uint8_t is_leader;
    //end
    uint8_t num_neighbors;
    uint8_t message_sent;
    uint16_t now;
    uint16_t nextShareSending;
    uint8_t cur_motion;
    uint8_t motion_state;
    uint8_t time_active;
    uint8_t move_state;
    nearest_neighbor_t nearest_neighbors[MAX_NUM_NEIGHBORS];
    motion_time_t move_motion[3];
    char send_moving;
    uint8_t green;
    uint8_t red;
    uint8_t blue;
    
} USERDATA;
