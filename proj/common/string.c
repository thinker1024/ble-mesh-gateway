#include "types.h"
#include "string.h"
#include "../common/assert.h"

char* strcpy(char * dst0, const char * src0) {
	char *s = dst0;
	while ((*dst0++ = *src0++))
		;
	return s;
}

char * strchr(const char *s, int c) {
	do {
		if (*s == c) {
			return (char*) s;
		}
	} while (*s++);
	return (0);
}

int memcmp(const void * m1, const void *m2, unsigned int n) {

	unsigned char *s1 = (unsigned char *) m1;
	unsigned char *s2 = (unsigned char *) m2;

	while (n--) {
		if (*s1 != *s2) {
			return *s1 - *s2;
		}
		s1++;
		s2++;
	}
	return 0;
}

void *
memchr(register const void * src_void, int c, unsigned int length) {
	const unsigned char *src = (const unsigned char *) src_void;

	while (length-- > 0) {
		if (*src == c)
			return (void *) src;
		src++;
	}
	return NULL;
}

void * memmove(void * dest, const void * src, unsigned int n) {
	char * d = (char *)dest;
	char * s = (char *)src;

	while (n--)
		*d++ = *s++;

	return dest;
}

void bcopy(register char * src, register char * dest, int len) {
	if (dest < src)
		while (len--)
			*dest++ = *src++;
	else {
		char *lasts = src + (len - 1);
		char *lastd = dest + (len - 1);
		while (len--)
			*(char *) lastd-- = *(char *) lasts--;
	}
}

void * memset(void * dest, int val, unsigned int len) {
	register unsigned char *ptr = (unsigned char*) dest;
	while (len-- > 0)
		*ptr++ = (unsigned char)val;
	return dest;
}

#if DEBUG_MEMCPY
void * memcpyb(void * out, const void * in, unsigned int length) {
	bcopy((char *) in, (char *) out, (int) length);
	return out;
}
#else
void * memcpy(void * out, const void * in, unsigned int length) {
	bcopy((char *) in, (char *) out, (int) length);
	return out;
}
#endif

// for performance, assume lenght % 4 == 0,  and no memory overlapped
void memcpy4(void * d, const void * s, unsigned int length){
	int* dst = (int*)d;
	int* src = (int*)s;
	assert((((int)dst) >> 2) << 2 == ((int)dst));			// address must alighn to 4
	assert((((int)src) >> 2) << 2 == ((int)src));			// address must alighn to 4
	assert((length >> 2) << 2 == length);					// lenght % 4 == 0
	assert(( ((char*)dst) + length <= (const char*)src) || (((const char*)src) + length <= (char*)dst));	//  no overlapped
	unsigned int len = length >> 2;
	while(len --){
		*dst++ = *src++;
	}
}

unsigned int strlen(const char *str) {

	unsigned int len = 0;

	if (str != NULL) {
		while (*str++) {

			len++;

		}
	}

	return len;
}

int strcmp(const char* firstString, const char* secondString) {
	while (*firstString == *secondString) {
		if (*firstString == '\0') {
			return 0;
		}
		++firstString;
		++secondString;
	}
	if (((unsigned char) *firstString - (unsigned char) *secondString) < 0) {
		return -1;
	}
	return 1;
}

char * strncpy(char *s, const char *t, unsigned int n) {
	char *p = s;
	unsigned int i = 0;

	if (!s)
		return s;

	while (t && i < n) {
		*s++ = *t++;
		i++;
	}

	if (!t) {
		do
			*s++ = '\0';
		while (i++ < n);
	}
	return p;
}

int ismemzero4(void *data, unsigned int len){
	int *p = (int*)data;
	len = len >> 2;
	for(int i = 0; i < len; ++i){
		if(*p){
			return 0;
		}
		++p;
	}
	return 1;
}

int ismemf4(void *data, unsigned int len){
	int *p = (int*)data;
	len = len >> 2;
	for(int i = 0; i < len; ++i){
		if(*p != 0xffffffff){
			return 0;
		}
		++p;
	}
	return 1;
}

void * memset4(void * dest, int val, unsigned int len) {
	int *p = (int*)dest;
	len = len >> 2;
	for(int i = 0; i < len; ++i){
		*p++ = val;
	}
	return dest;
}

void zeromem4(void *data, unsigned int len){
	memset4(data, 0, len);
}

char *strcat(char *dest, const char *src)
{

    while (*dest)
      dest++;
    while (*dest++ = *src++)
      ;
    return dest;
}

unsigned int myStrCmp(const char *s1, const char *s2, unsigned int n)
{
	unsigned int i = 0;
	while (i < n){
		if(s1[i] != s2[i])
			return 1;
		i++;
	}
	return 0;
}

char *strstr( char *str, char * subStr )
{
	while (*str != '\0' ) {

		char *p = str;
		char *q = subStr;
		char *res = NULL ;
		if (*p == *q) {
			res = p;
			while (*q != '\0' && *p == *q){
				p++;
				q++;
			}
			if (*q == '\0' )
				return res;
		}
		str++;
	}
}

int atoi(char* pointer)
{
	int result = 0;
	char* pointer1;
	int multiplier = 1;
	char sign = 1;

	if(*pointer == '-')
		sign =- 1;

	pointer1 = pointer;

	while(*pointer != '\0')
	{
		if(*pointer >= '0' && *pointer <= '9')
			multiplier = multiplier * 10;

		pointer = pointer + 1;
	}

	pointer = pointer1;

	while(*pointer != '\0')
	{
		if(*pointer >= '0' && *pointer <= '9')
		{
			result = result + ( (*pointer%48)  * multiplier);
			multiplier = multiplier / 10;
		}

		pointer = pointer+1;
	}

	return (result * sign) / 10;
}

unsigned char *itoa(u8 num, unsigned char *str, u8 radix)
{
  unsigned char index[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  u8 i = 0, j, unum = 0;
  unum = num;
  do {
    str[i++] = index[num % radix];
    num /= radix;
  } while (num);

  if (unum < 16) {
    str[i++] = '0';
  }
  str[i] = '\0';
  unsigned char temp;
  for (j = 0; j <= (i - 1) / 2; j++) {
    temp = str[j];
    str[j] = str[i - 1 - j];
    str[i - 1 - j] = temp;
  }
  return str;
}


