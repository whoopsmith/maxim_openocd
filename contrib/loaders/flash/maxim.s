/***************************************************************************
 *   Copyright (C) 2012 by Maxim Integrated                                *
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

	.text
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func

/*
 * Params :
 * r0 = workarea start
 * r1 = workarea end
 * r2 = target address
 * r3 = count (32bit words)
 *
 * Clobbered:
 * r4 = pFLASH_CTRL_BASE
 * r5 = FLASHWRITECMD
 * r7 - rp
 * r8 - wp, tmp
 */

write:
	ldr 	r4, pFLASH_CTRL_BASE

wait_fifo:
	ldr 	r8, [r0, #0]	/* read wp */
	cmp 	r8, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r7, [r0, #4]	/* read rp */
	cmp 	r7, r8			/* wait until rp != wp */
	beq 	wait_fifo

mainloop:
	str		r2, [r4, #0x00]	/* FLSH_ADDR - write address */
	add		r2, r2, #4		/* increment target address */
	ldr		r8, [r7], #4
	str		r8, [r4, #0x30]	/* FLSH_DATA0 - write data */
	ldr		r5, [r4, #0x08]	/* FLSH_CN */
	orr		r5, r5, #1
	str		r5, [r4, #0x08]	/* FLSH_CN - enable write */
busy:
	ldr		r8, [r4, #0x08]	/* FLSH_CN */
	tst		r8, #1
	bne		busy

	cmp 	r7, r1			/* wrap rp at end of buffer */
	it  	cs
	addcs	r7, r0, #8		/* skip loader args */
	str 	r7, [r0, #4]	/* store rp */
	subs	r3, r3, #1		/* decrement word count */
	cbz 	r3, exit		/* loop if not done */
	b		wait_fifo
exit:
	bkpt

	nop
pFLASH_CTRL_BASE: .word 0x40000000  /* this will be overwritten by the driver */

