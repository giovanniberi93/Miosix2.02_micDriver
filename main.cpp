
#include <cstdio>
#include "miosix.h"
#include "Microphone.h"
#include "player.h"
#include <math.h>
#include <functional>

using namespace std;
using namespace miosix;

/*
 * This example program records half second (22050 samples) from the microphone and
 * plays such sound through the board's DAC.
 */
int main()
{

    Player& player = Player::instance();
    Microphone& mic = Microphone::instance(); 
    
    /* Best results obtained with a size in the form of (N + 0.5) * 256 with N integer */
    static const unsigned short size = 896;
    mic.init(bind(&Player::play,&player,placeholders::_1,placeholders::_2),size);
    player.init();
    mic.start();
    
    while (1){
    };
    
}
