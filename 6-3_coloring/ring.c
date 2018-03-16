#define SIMULATOR
#ifdef REAL
#include <kilolib.h>
#include <avr/io.h>  // for microcontroller register defs
#include "ring.h"
USERDATA myData;
USERDATA *mydata = &myData;
#else
#include <math.h>
#include <kilombo.h>
#include <stdio.h> // for printf
#include "ring.h"
REGISTER_USERDATA(USERDATA)
#endif

void send_election();
void prepare_message(uint8_t m, uint8_t receiver);
static uint8_t final=0;//,color_done=0;//count=1;; //once found minimum will become 1 so rounds wont happen but if finds a smaller node again then becomes 0 and we will execute all rounds and then at the end of the round will again go back to 1 so as to stop further processing till finds a smaller one
//end

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
    if (mydata->state == COOPERATIVE)
        mydata->red = 1;
    else
        mydata->red = 0;
    
#ifdef SIMULATOR
    printf("Joining %d right=%d left=%d\n", mydata->my_id, mydata->my_right, mydata->my_left);
#endif
    mydata->send_election=1;
}


//new
void recv_color(uint8_t *payload)
{
    
        if(payload[ID]==mydata->my_left)
        {
            if(mydata->my_color > 5 || payload[COLOR] > 5){
                mydata->my_color = getRoundcolor(payload[COLOR],mydata->my_color);
                printf("(Not in range) MyId:%d MyColor:%d\n",mydata->my_id,mydata->my_color);
            }
        }
        if(payload[COLOR]==mydata->my_color)
         {
            mydata->my_color=mydata->my_id;
            printf("(Invalid color) MyId:%d MyColor:%d\n",mydata->my_id,mydata->my_color);
         }
    
}


void recv_elected(uint8_t *payload)
{
    //In any round if find smaller id member make him leader and set token type election
    // or new node has been added and he doesn't know leader
    
		if (payload[LEADER] != mydata->my_id) {
        		mydata->green=0;
            mydata->my_leader=payload[LEADER];
            #ifdef SIMULATOR
                	//printf("Round 2 token at %d and leader is %d \n",mydata->my_id,mydata->my_leader);
                    printf("My_id :%d My_left:%d My_right:%d\n",mydata->my_id,mydata->my_left,mydata->my_right);
            #endif
                	mydata->send_elected = 1;
		} else {
			mydata->green=1;
            
            #ifdef SIMULATOR
            printf("My_id :%d My_left:%d My_right:%d\n",mydata->my_id,mydata->my_left,mydata->my_right);
            printf("End of round 2 final leader is %d \n",mydata->my_id);
            #endif
                        final=1; //found a smaller one final the leader, finalize till finds a smaller one
		}
    mydata->active =0;
}

void recv_election(uint8_t *payload)
{
    //In any round if find smaller id member make him leader and set token type election
    // or new node has been added and he doesn't know leader
    
	if (payload[MSG] == ELECTION) {
        	//for making leader change his color and start round two ROUND 1 END
        	//if in round 1 I am leader and I get message from neighbour saying that I will change to green and start round2
        	// final =0 and not 1 so that means leader is not finalised so execute the round
	        //mydata->blue=1-mydata->blue;
		if (payload[LEADER] == mydata->my_id) {
                	mydata->green = 1;
                    #ifdef SIMULATOR
                	printf("%d sets itself as leader\n", mydata->my_id);
                	printf("Round 2 Start\n");
                    #endif
                	mydata->send_elected = 1;
        } else if (payload[LEADER] < mydata->my_leader) {      //new node and is smaller
                    mydata->my_leader=payload[LEADER];
                	mydata->green=0;
                	mydata->send_election=1;
        } else {
                    if (mydata->active == 0)
                    { //if payload leader greater sets leader to itself in paylod (by default while prepare message our myleader is myid by default)
                        #ifdef SIMULATOR
                	    printf("%d sets %d as leader\n", mydata->my_id, mydata->my_leader);
                        #endif
                        mydata->send_election=1;
                        mydata->active =1;
                    }
		}
	}
}

void message_rx(message_t *m, distance_measurement_t *d)
{
    uint8_t dist = estimate_distance(d);
    
    if (m->type == NORMAL)
    {
        recv_sharing(m->data, dist);
        switch (m->data[MSG])
        {
            case JOIN:
                recv_joining(m->data);
                break;
            case ELECTION:
		if (mydata->my_left == m->data[ID])
               		 recv_election(m->data);
                break;
            case ELECTED:
		if (mydata->my_left == m->data[ID])
                	recv_elected(m->data);
                break;
            case SHARE: // new addition
                if (mydata->my_left == m->data[ID] || mydata->my_right == m->data[ID])
                recv_color(m->data);
                break;
         //end
        }
    }
}


void prepare_message(uint8_t m, uint8_t receiver)
{
    mydata->msg.data[MSG] = m;
    mydata->msg.data[ID] = mydata->my_id;
    mydata->msg.data[RIGHT_ID] = mydata->my_right;
    mydata->msg.data[LEFT_ID] = mydata->my_left;
    mydata->msg.data[RECEIVER] = mydata->my_right;
    mydata->msg.data[SENDER] = mydata->my_id;
    mydata->msg.data[STATE] = mydata->state;
    mydata->msg.data[LEADER]=mydata->my_leader;
    mydata->msg.data[COLOR]=mydata->my_color; // new addition for setting color in message
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
            mydata->my_right = mydata->nearest_neighbors[i].right_id;
            mydata->my_left = mydata->nearest_neighbors[i].id;
            prepare_message(JOIN, 0);
            mydata->red = 1;
	    mydata->send_election =1; 
#ifdef SIMULATOR
            printf("Sending Joining %d right=%d left=%d\n", mydata->my_id, mydata->my_right, mydata->my_left);
#endif
            

 final=0; //new node so redo all rounds
//end
            mydata->send_election=1;
        }
    }
}

void send_sharing()
{
    // Precondition
    if (mydata->now >= mydata->next_share_sending &&  mydata->message_sent == 1)
    {
        // Sending
        prepare_message(SHARE, 0);
        // effect:
        mydata->next_share_sending = mydata->now + SHARING_TIME;
    }
}

void send_elected()
{
    // TODO: it will send only one election message  if it has just join the ring
    // Precondition:
    if (mydata->state == COOPERATIVE && mydata->send_elected  && mydata->message_sent == 1)
    {
        // Sending
        prepare_message(ELECTED, mydata->my_left);
        mydata->send_elected = 0;
        // effect:
    }
}


void send_election()
{
    // TODO: it will send only one election message  if it has just join the ring
    // Precondition:
    if (mydata->state == COOPERATIVE && mydata->send_election  && mydata->message_sent == 1)
    {
        // Sending
        prepare_message(ELECTION, mydata->my_left);
        mydata->send_election = 0;
	mydata->active =1;
        // effect:
    }
}

void move(uint8_t tick)
{
    // TODO: Optional you can make the leader robot move
    // Precondition:
    if (mydata->motion_state == ACTIVE && mydata->state == COOPERATIVE && mydata->is_leader)
    {
        
        if (mydata->time_active == mydata->move_motion[mydata->move_state].time)
        {
            // Effect:
            mydata->move_state++;
            if (mydata->move_state == 3)
            {
                
#ifdef SIMULATOR
                printf("Sending Move %d\n", mydata->my_id);
#endif
                mydata->motion_state = STOP;
                return;
            }
            mydata->time_active = 0;
            
        }
        set_motion(mydata->move_motion[mydata->move_state].motion);
        mydata->time_active++;
    }
    else
    {
        set_motion(STOP);
    }
    
}


void loop()
{
    delay(30);
    
    send_joining();
    send_sharing();
    //send_election();
    //send_elected();
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
    mydata->my_color = mydata->my_id; //new addition
    mydata->state = AUTONOMOUS;
    mydata->my_left = mydata->my_right = mydata->my_id;
    mydata->num_neighbors = 0;
    mydata->message_sent = 0,
    mydata->now = 0,
    mydata->next_share_sending = SHARING_TIME,
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
    mydata->green = 0,
    mydata->blue = 0,
    mydata->send_election = 1;
    mydata->is_leader = 0;
    mydata->my_leader = mydata->my_id;
    mydata->active = 0;
    
    mydata->msg.data[MSG] = NULL_MSG;
    mydata->msg.crc = message_crc(&mydata->msg);
    
    
    
#ifdef SIMULATOR
    printf("Initializing %d\n", mydata->my_id);
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
