#define SIMULATOR

#ifndef SIMULATOR
#include <kilolib.h>
#include <avr/io.h>  // for microcontroller register defs
#else
#include <math.h>
#include <kilombo.h>
#include <stdio.h> // for printf
#endif


#define MAX_NUM_NEIGHBORS 10
#define SHARING_TIME 20
#define TOKEN_TIME 20
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
    
    uint8_t num_neighbors;
    uint8_t message_sent;
    int8_t token;
    uint16_t now;
    uint16_t nextShareSending;
    uint16_t nextToken;
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
    
}  USERDATA;
    USERDATA myData;
    USERDATA *mydata = &myData;


#ifdef SIMULATOR
REGISTER_USERDATA(USERDATA)
#endif








/* Helper function for setting motor speed smoothly
 */
void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
    // OCR2A = ccw;  OCR2B = cw;
/*#ifdef KILOBOT
    uint8_t l = 0, r = 0;
    if (ccw && !OCR2A) // we want left motor on, and it's off
        l = 0xff;
    if (cw && !OCR2B)  // we want right motor on, and it's off
        r = 0xff;
    if (l || r)        // at least one motor needs spin-up
    {
        set_motors(l, r);
        delay(15);
    }
#endif*/
    // spin-up is done, now we set the real value
    set_motors(ccw, cw);
}


void set_motion(motion_t new_motion)
{
    switch(new_motion) {
        case STOP:
            smooth_set_motors(0,0);
            break;
        case FORWARD:
            smooth_set_motors(kilo_straight_left, kilo_straight_right);
            break;
        case LEFT:
            smooth_set_motors(kilo_turn_left, 0);
            break;
        case RIGHT:
            smooth_set_motors(0, kilo_turn_right); 
            break;
    }
}


char in_interval(uint8_t distance)
{
    //if (distance >= 40 && distance <= 60)
    if (distance <= 90)
        return 1;
    return 0;
}

//
char is_stabilized()
{
    uint8_t i=0,j=0;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        
        if ((mydata->nearest_neighbors[i].state == AUTONOMOUS && mydata->nearest_neighbors[i].num > 2) ||
            (mydata->nearest_neighbors[i].state == COOPERATIVE && mydata->nearest_neighbors[i].num_cooperative > 2))
            j++;
    }

    return j == mydata->num_neighbors;
}

// Search for id in the neighboring nodes
uint8_t exists_nearest_neighbor(uint8_t id)
{
    uint8_t i;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        if (mydata->nearest_neighbors[i].id == id)
            return i;
    }
    return i;
}


// Search for id in the neighboring nodes
uint8_t are_all_cooperative()
{
    uint8_t i;
    for (i=0; i<mydata->num_neighbors; i++)
    {
        if (mydata->nearest_neighbors[i].state == COOPERATIVE)
            return 0;
    }
    return 1;
}

uint8_t get_nearest_two_neighbors()
{
    uint8_t i, l, k;
    uint16_t min_sum = 0xFFFF;
    
    k = i = mydata->num_neighbors;
    if (are_all_cooperative())
    {
        for (i=0; i<mydata->num_neighbors; i++)
        {
            // shortest
            if (mydata->nearest_neighbors[i].distance < min_sum)
            {
                k = i;
            }
        }
        if (k < mydata->num_neighbors)
        {
            i = k;
        }
    }
    else
    {
        for (i=0; i<mydata->num_neighbors; i++)
        {
            // Is it cooperative and at distance in [4cm,6cm]?
            if (mydata->nearest_neighbors[i].state == COOPERATIVE)
            {
                l = exists_nearest_neighbor(mydata->nearest_neighbors[i].right_id);
                // Does the right exits in my table?
                if (l < mydata->num_neighbors)
                {
                    if (mydata->nearest_neighbors[i].distance +
                        mydata->nearest_neighbors[l].distance < min_sum)
                    {
                        min_sum = mydata->nearest_neighbors[i].distance + mydata->nearest_neighbors[l].distance;
                        k = i;
                    }
                }
            }
        }
        if (k < mydata->num_neighbors)
        {
            i = k;
        }
    }
    return i;
}

void recv_sharing(uint8_t *payload, uint8_t distance)
{
    if (payload[ID] == mydata->my_id  || payload[ID] == 0 || !in_interval(distance) ) return;
    


    uint8_t i = exists_nearest_neighbor(payload[ID]);
    if (i >= mydata->num_neighbors) // The id has never received
    {
        if (mydata->num_neighbors < MAX_NUM_NEIGHBORS)
        {
            i = mydata->num_neighbors;
            mydata->num_neighbors++;
            mydata->nearest_neighbors[i].num = 0;
            
        }
    }

    
    mydata->nearest_neighbors[i].id = payload[ID];
    mydata->nearest_neighbors[i].right_id = payload[RIGHT_ID];
    mydata->nearest_neighbors[i].left_id = payload[LEFT_ID];
    mydata->nearest_neighbors[i].state = payload[STATE];
    mydata->nearest_neighbors[i].distance = distance;
    if (payload[STATE] == AUTONOMOUS)
    {
        mydata->nearest_neighbors[i].num++;
        mydata->nearest_neighbors[i].num_cooperative = 0;
    }
    else
    {
        mydata->nearest_neighbors[i].num_cooperative++;
        mydata->nearest_neighbors[i].num = 0;
    }

}

void recv_joining(uint8_t *payload)
{
    if (payload[ID] == mydata->my_id) return;
    if (payload[RIGHT] == mydata->my_id) // && payload[LEFT] == my_left)
    {
        mydata->my_right = payload[ID];
        mydata->state   = COOPERATIVE;
    }
    if (payload[LEFT] == mydata->my_id) // && payload[RIGHT] == my_right)
    {
        mydata->my_left = payload[ID];
       mydata-> state   = COOPERATIVE;
    }
    if (mydata->num_neighbors == 1 && payload[LEFT] == mydata->my_id)
    {
        mydata->my_left = mydata->my_right = payload[ID];
        mydata->state   = COOPERATIVE;
    }
    if (mydata->state == COOPERATIVE) {
        mydata->blue=1;
        mydata->nextToken = mydata->now + TOKEN_TIME;
    }
#ifdef SIMULATOR
    printf("Joining %d right=%d left=%d\n", mydata->my_id, mydata->my_right, mydata->my_left);
#endif
    
}

void recv_move(uint8_t *payload)
{
    if (mydata->my_id == payload[RECEIVER]) {
        mydata->token = 1;
    	mydata->green=1;
    	mydata->nextToken = mydata->now + TOKEN_TIME;
#ifdef SIMULATOR
        printf("%d has the token until %d now=%d\n", mydata->my_id, mydata->nextToken, mydata->now);
#endif
    }
}


void message_rx(message_t *m, distance_measurement_t *d)
{
    uint8_t dist = estimate_distance(d);
    
    if (m->type == NORMAL)
    {
#ifdef SIMULATOR
        printf("%d Receiving %d \n", mydata->my_id, m->data[MSG]);
#endif
        recv_sharing(m->data, dist);
        switch (m->data[MSG])
        {
            case JOIN:
                recv_joining(m->data);
                break;
            case MOVE:
                recv_move(m->data);
                break;
        
        }
    }
}


void prepare_message(uint8_t m, uint8_t receiver)
{
#ifdef SIMULATOR
    printf("%d is preparing Message %d \n", mydata->my_id, m);
#endif
    mydata->msg.data[MSG] = m;
    mydata->msg.data[ID] = mydata->my_id;
    mydata->msg.data[RIGHT_ID] = mydata->my_right;
    mydata->msg.data[LEFT_ID] = mydata->my_left;
    mydata->msg.data[RECEIVER] = receiver;
    mydata->msg.data[SENDER] = mydata->my_id;
    mydata->msg.data[STATE] = mydata->state;
    
    mydata->msg.type = NORMAL;
    mydata->msg.crc = message_crc(&mydata->msg);
    mydata->message_sent = 0;
}

/**********************************/
/**********************************/
void send_joining()
{
    uint8_t i;
    /* precondition  */
    
    if (mydata->state == AUTONOMOUS && is_stabilized())
    {

        i = get_nearest_two_neighbors();
        if (i < mydata->num_neighbors && mydata->message_sent == 1)
        {
            // effect:

            mydata->state = COOPERATIVE;
            mydata->nextToken = mydata->now + TOKEN_TIME;
            mydata->my_right = mydata->nearest_neighbors[i].right_id;
            mydata->my_left = mydata->nearest_neighbors[i].id;
			mydata->blue = 1;
            prepare_message(JOIN, 0);
#ifdef SIMULATOR
            printf("Sending Joining %d right=%d left=%d\n", mydata->my_id, mydata->my_right, mydata->my_left);
#endif
        }
    }
}

void send_sharing()
{
    // Precondition
    if (mydata->now >= mydata->nextShareSending &&  mydata->message_sent)
    {
        // Sending
        if(mydata->nextToken <= mydata->now && mydata->state == COOPERATIVE && mydata->message_sent && mydata->token == 1)
            prepare_message(MOVE,mydata->my_right);
        else
            prepare_message(SHARE, 0);
        // effect:
        mydata->nextShareSending = mydata->now + SHARING_TIME;
    }
}





void send_move()
{
    if(mydata->nextToken <= mydata->now && mydata->state == COOPERATIVE && mydata->message_sent && mydata->token == 1){
		prepare_message(MOVE,mydata->my_right);
		mydata->green=0;
		mydata->token = 0;
		mydata->message_sent = 0;
#ifdef SIMULATOR
        printf("%d is releasing the token\n", mydata->my_id);
#endif
	}
}

void move(uint8_t tick)
{
   
}


void loop()
{
    delay(30);
    
    send_joining();
    send_move();
    send_sharing();
    move(mydata->now);
    
    set_color(RGB(mydata->red, mydata->green, mydata->blue));

    mydata->now++;
}


message_t *message_tx()
{
    return &mydata->msg;
}
 
void message_tx_success() {
    mydata->message_sent = 1;
    mydata->msg.data[MSG] = NULL_MSG;
    mydata->msg.crc = message_crc(&mydata->msg);
}

void setup() {
    rand_seed(rand_hard());

    mydata->my_id = rand_soft();
    
    mydata->state = AUTONOMOUS;
    mydata->my_left = mydata->my_right = mydata->my_id;
    mydata->num_neighbors = 0;
    mydata->message_sent = 0,
    mydata->now = 0,
    mydata->nextShareSending = SHARING_TIME,
    mydata->cur_motion = STOP;
    mydata->motion_state = STOP;
    mydata->time_active = 0;
    mydata->move_state = 0;
    mydata->move_motion[0].motion = LEFT;
    mydata->move_motion[0].motion = 3;
    mydata->move_motion[1].motion = RIGHT;
    mydata->move_motion[1].motion = 5;
    mydata->move_motion[0].motion = LEFT;
    mydata->move_motion[0].motion = 2;
    mydata->red = 0,
    mydata->blue = 0,
    
    mydata->msg.data[MSG] = NULL_MSG;
    mydata->msg.crc = message_crc(&mydata->msg);
    
    mydata->token = mydata->my_id < 100;
    mydata->green = mydata->token;
   
#ifdef SIMULATOR
    printf("Initializing %d\n", mydata->my_id);
    printf("Token %d\n", mydata->token );
#endif

    mydata->message_sent = 1;
	
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
char *cb_botinfo(void)
{
    char *p = botinfo_buffer;
    p += sprintf (p, "ID: %d \n", kilo_uid);
    if (mydata->state == COOPERATIVE)
        p += sprintf (p, "State: COOPERATIVE\n");
    if (mydata->state == AUTONOMOUS)
        p += sprintf (p, "State: AUTONOMOUS\n");
    
    return botinfo_buffer;
}
#endif

int main() {
    kilo_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);
    
    return 0;
}
