/*! \file usart.h \brief USART routines. 
 * \author Mikael Larsmark
 * \date 2009-10-28
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


#ifndef _USART_H_
#define _USART_H_

unsigned char poll_usart0_receive (void);
void usart0_init (unsigned int baudrate);
unsigned char usart0_transmit (unsigned char data);
unsigned char usart0_receive (void);
unsigned char usart0_receive_loopback (void);
unsigned char poll_usart0_receive (void);
unsigned char usart0_sendstring (char *data,unsigned char length);

unsigned char poll_usart1_receive (void);
void usart1_init (unsigned int baudrate);
unsigned char usart1_transmit (unsigned char data);
unsigned char usart1_receive (void);
unsigned char usart1_receive_loopback (void);
unsigned char poll_usart1_receive (void);
unsigned char usart1_sendstring (char *data,unsigned char length);

#endif

