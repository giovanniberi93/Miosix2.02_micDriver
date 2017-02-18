#include <cstdio>
#include "miosix.h"
#include "Microphone.h"
#include "player.h"
#include <math.h>
#include <tr1/functional>

using namespace std;
using namespace std::tr1;
using namespace miosix;

/* 
 * This example program streams on the serial every 15ms of audio as couples time:val
 * each representing 1 PCM sample. Every 15 second the word "new" is sent.
 * With this you can use the plot.py program to see the waveform.
 */

void visualize(unsigned short* PCM, unsigned short  size){
    iprintf("new\n");
    for(int i = 0; i < size; i++){
        iprintf("%d:%d\n", i, (short) PCM[i] ); // prints key:val
    }
}

int main()
{

    bool lit = false;
    Microphone& mic = Microphone::instance(); 
    
    static const unsigned short size = 661; // sets 15ms (almost)
    
    // init the Microphone driver
    mic.init(&visualize,size);
    // recording.
    mic.start();
    
    
    while (1){
        // shows that you can do something else in the meanwhile
        if(lit){
            lit=false;
            ledOff();
        } else {
            lit = true;
            ledOn();
        }
            delayMs(500);
    };
    
}
