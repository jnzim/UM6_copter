#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#ifdef HAVE_LIMITS_H
#include <limits.h>
#endif
#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h>
#endif
#include <errno.h>
#ifdef NEED_DECLARATION_ERRNO
extern int errno;
#endif
#if 0
#include <stdlib.h>
#endif

#include "ctype.h"
//#include "ansidecl.h"
//#include "safe-ctype.h"

#ifndef Uint16_MAX
#define	Uint16_MAX	((uint16_t)(0xFFFF))		
#endif

uint16_t strtoUint16(const char *, char **, register int );
uint16_t my_map_int( uint16_t , uint16_t, uint16_t, uint16_t, uint16_t );

 //* Convert a string to an unsigned long integer.
 //*
 //* Ignores `locale' stuff.  Assumes that the upper and lower case
 //* alphabets and digits are each contiguous.

uint16_t strtoUint16(const char *nptr, char **endptr, register int base)
{
	register const char *s = nptr;
	register unsigned long acc;
	register int c;
	register uint16_t cutoff;
	register int neg = 0, any, cutlim;

	
	//See strtol for comments as to the logic used.

	do {
		c = *s++;
	} while (isspace(c));
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+')
	c = *s++;
	if ((base == 0 || base == 16) &&
	c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0)
	base = c == '0' ? 8 : 10;
	cutoff = (uint16_t)Uint16_MAX / (uint16_t)base;		//  the type cast is so we make sure we to integer math
	cutlim = (uint16_t)Uint16_MAX % (uint16_t)base;		//  then we use the mod operator to get the remainder
	
	for (acc = 0, any = 0;; c = *s++)								// endless for loop, only way out is a break, increment happens at the end of the loop
	{
		if (isdigit(c))
		c -= '0';
		else if (isalpha(c))
		c -= isupper(c) ? 'A' - 10 : 'a' - 10;					// c -= 'A' - 10   or  'a' -10
		else
		break;
		if (c >= base)
		break;
		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
		any = -1;
		else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}
	if (any < 0) {
		acc = Uint16_MAX;
		errno = ERANGE;
	} else if (neg)
	acc = -acc;
	if (endptr != 0)
	*endptr = (char *) (any ? s - 1 : nptr);
	return (acc);
}

//my_map_int(yaw,0,999,0,16000);

uint16_t my_map_int(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return  (x * out_max / in_max);
	
}



//
//double int_trapezium(double from, double to, double n, double (*func)())
//{   
	//double h = (to - from) / n;   
	//double sum = func(from) + func(to);   
	//int i;   
	//for(i = 1;i < n;i++)    
	//{
		//sum += 2.0*func(from + i * h); 	
	//}   
	  //
	//return  h * sum / 2.0;
//}
//
//
//double int_simpson(double from, double to, double n, double (*func)())
//{   
	//double h = (to - from) / n;   
	//double sum1 = 0.0;   double sum2 = 0.0;   
	//int i;    double x;    
	//for(i = 0;i < n;i++) 
	//{
		     //sum1 += func(from + h * i + h / 2.0); 
	//}
	//for(i = 1;i < n;i++)      
	//{
		//sum2 += func(from + h * i);    
	//}	
		//return h / 6.0 * (func(from) + func(to) + 4.0 * sum1 + 2.0 * sum2);
//}



//#endif /* MYUTILITIES_H_ */



/*
ORIGIANL ORIGINAL:

 * Copyright (c) 1990 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. [rescinded 22 July 1999]
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.



#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#ifdef HAVE_LIMITS_H
#include <limits.h>
#endif
#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h>
#endif
#include <errno.h>
#ifdef NEED_DECLARATION_ERRNO
extern int errno;
#endif
#if 0
#include <stdlib.h>
#endif
//#include "ansidecl.h"
//#include "safe-ctype.h"

#ifndef ULONG_MAX
#define	ULONG_MAX	((unsigned long)(~0L))		
#endif


 * Convert a string to an unsigned long integer.
 *
 * Ignores `locale' stuff.  Assumes that the upper and lower case
 * alphabets and digits are each contiguous.

unsigned long strtoul(const char *nptr, char **endptr, register int base)
{
	register const char *s = nptr;
	register unsigned long acc;
	register int c;
	register unsigned long cutoff;
	register int neg = 0, any, cutlim;

	
//See strtol for comments as to the logic used.

	do {
		c = *s++;
	} while (isspace(c));
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+')
		c = *s++;
	if ((base == 0 || base == 16) &&
	    c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0)
		base = c == '0' ? 8 : 10;
	cutoff = (unsigned long)ULONG_MAX / (unsigned long)base;		//  the type cast is so we make sure we to integer math
	cutlim = (unsigned long)ULONG_MAX % (unsigned long)base;		//  then we use the mod operator to get the remainder
	
	for (acc = 0, any = 0;; c = *s++)								// endless for loop, only way out is a break, increment happens at the end of the loop
	{
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c))
			c -= isupper(c) ? 'A' - 10 : 'a' - 10;					// c -= 'A' - 10   or  'a' -10
		else
			break;
		if (c >= base)
			break;
		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
			any = -1;
		else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}
	if (any < 0) {
		acc = ULONG_MAX;
		errno = ERANGE;
	} else if (neg)
		acc = -acc;
	if (endptr != 0)
		*endptr = (char *) (any ? s - 1 : nptr);
	return (acc);
}




*/