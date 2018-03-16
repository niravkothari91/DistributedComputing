#define MAX_NUM_NEIGHBORS 10
#define SHARING_TIME 20
#define WAKEUPTIME 50
#define MISTIME 40
#define STABILIZATIONTIME 30
//PAYLOAD
#define MSG 0
#define ID 1
//#define RIGHT_ID  2
//#define LEFT_ID  3
#define MOTION_STATE 3 
#define STATE  4

#define RECEIVER 5
#define SENDER 6
#define LEADER 7
#define COLOR 8 // new addition
#define RANDOM_VALUE 2

#define ACTIVE 0
#define DISABLED 1



#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif

typedef enum { NULL_MSG,
    SHARE,
    JOIN,
    LEAVE,
    ELECTION,
    ELECTED,
    SETCOLOR
} message_type;  // MESSAGES

typedef enum {
    AUTONOMOUS,
    COOPERATIVE,
    DEACTIVATED,
    MIS,
    PASSIVE,
    NOMIS
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
    uint8_t random_value; //random value
    robot_state state;
    uint8_t distance;
    uint8_t num;
    uint8_t num_cooperative;
   // uint8_t color;
} nearest_neighbor_t;

typedef struct  {
    uint8_t motion;
    uint16_t time;
} motion_time_t;


typedef struct
{
    uint8_t my_id;
    uint8_t my_color;
    uint8_t my_right;
    uint8_t my_left;
    uint8_t my_right_color;
    uint8_t my_left_color;
    uint8_t my_random_value; //added random value
    message_t msg;
    robot_state state;
    
    uint8_t num_neighbors;
    uint8_t message_sent;
    uint16_t now;
    uint16_t next_share_sending;
    uint16_t next_wakeup;//wakeup
    uint16_t next_mis;
    uint16_t next_stabilization;
    uint8_t cur_motion;
    uint8_t motion_state;
    uint16_t time_active;
    uint8_t active;
    uint8_t move_state; 
    uint8_t distance; //distance
    uint8_t counter;// counter for flag
    
    nearest_neighbor_t nearest_neighbors[MAX_NUM_NEIGHBORS];
    motion_time_t move_motion[2];
    uint8_t send_election;
    uint8_t send_elected;
    uint8_t send_color; //send message to cet color
    int8_t is_leader;
    uint8_t my_leader;
    uint8_t green;
    uint8_t red;
    uint8_t blue;
    
} USERDATA;

uint8_t getRoundcolor(uint8_t parent_color,uint8_t my_color){
    uint8_t xor_result,index=0,c;
    xor_result=parent_color^my_color;
    //printf("xor_result:%d\n",xor_result);
    while((xor_result&1)!=1)
    {
        xor_result=xor_result>>1;
        index++;
    }
    //printf("index:%d\n",index);
    my_color=my_color>>index;
    c=my_color&1;
    index=(index<<1)|c;
    return index;
}
