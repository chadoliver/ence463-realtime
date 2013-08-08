/*
 * lcd_lld.h
 *
 *  Created on: Aug 7, 2013
 *      Author: chad_oliver
 */

#ifndef LCD_LLD_H_
#define LCD_LLD_H_

#include "FreeRTOS.h"
#include "rit128x96x4.h"

void ( *lcd_init )( unsigned portLONG ) = RIT128x96x4Init;
void ( *lcd_draw_string )( const portCHAR *, unsigned portLONG, unsigned portLONG, unsigned portCHAR ) = RIT128x96x4StringDraw;
void ( *lcd_clear )( void ) = RIT128x96x4Clear;


#endif /* LCD_LLD_H_ */
