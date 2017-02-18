/***************************************************************************
 *   Copyright (C) 2011 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include "miosix.h"

#ifndef PLAYER_H
#define PLAYER_H

/**
 * Class to play an audio file on the STM32's DAC
 */
class Player
{
public:
	/**
	 * \return an instance of the player (singleton)
	 */
	static Player& instance();

	/**
	 * Play an audio file, returning after the file has coompleted playing
	 * \param sound sound file to play
	 */
	void play(unsigned short* buf, unsigned short size);

	/**
	 * \return true if the resource is busy
	 */
	bool isPlaying() const;
        
        void init();
private:
	Player();
        ~Player();
	Player(const Player&);
	Player& operator= (const Player&);
        bool fillStereoBuffer(unsigned short *buffer, int size);
	mutable miosix::Mutex mutex;
        unsigned short* soundBuffer;
        unsigned int soundSize;
        unsigned int soundIndex;
};

#endif //PLAYER_H
