/*! \file delay.h \brief Delay routine
 * \author Mikael Larsmark, SM2WMV
 * \date 2008-04-06
 */
/***************************************************************************
 *   Copyright (C) 2009 by Mikael Larsmark, Luleå University of Technology *
 *   larsmark@ltu.se                                                       *
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
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef _DELAY_H_
#define _DELAY_H_

#define F_CPU        16000000               		// 16MHz processor
//#define  F_CPU  14745600   /* The AVR clock frequency in Hertz */

void delay_ms(unsigned char ms);	//MAX 255ms
void delay_s(unsigned char s);

#endif
