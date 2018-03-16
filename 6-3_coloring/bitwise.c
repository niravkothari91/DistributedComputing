//
//  bitwise.c
//  
//
//  Created by Tejas  Tundulwar on 10/4/17.
//

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int getch (void)
{
    int ch;
    struct termios oldt, newt;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON|ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    return ch;
}

//int getRound_color(int parent_color,int my_color){
int getRound_color(int parent_color,int my_color){
    int xor_result,index=0,c;
    xor_result=parent_color^my_color;
    printf("xor_result:%d\n",xor_result);
    while((xor_result&1)!=1)
    {
        xor_result=xor_result>>1;
        index++;
    }
    printf("index:%d\n",index);
    my_color=my_color>>index;
    c=my_color&1;
    index=(index<<1)|c;
    return index;
}

int main(){
    printf("new color:%d\n",getRound_color(0,44));
    return 0;
}

