#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

uint8_t atohex8(const char *s)
{
	uint8_t v = 0;
	if(s == 0 || strlen(s) == 0)
		return 0xff;

	size_t sl = strlen(s);
	if(sl >= 2 && s[0] == '0' && (s[1] == 'X' || s[1] == 'x')) {
		if(sl == 2) {/* error !!! 0x only */
			printf("%s = 0xff\n", s);
			return 0xff;
		}
		s += 2;
	}

	if(*s >= '0' && *s <= '9')
		v = (*s - '0') << 4;
	else if(*s >= 'A' && *s <= 'F')
		v = ((*s - 'A') + 10) << 4;
	else if(*s >= 'a' && *s <= 'f')
		v = ((*s - 'A') + 10) << 4;

	s++;

	if(*s >= '0' && *s <= '9')
		v |= (*s - '0');
	else if(*s >= 'A' && *s <= 'F')
		v |= ((*s - 'A') + 10);
	else if(*s >= 'a' && *s <= 'f')
		v |= ((*s - 'A') + 10) << 4;

	//printf("%s = 0x%02x\n", s, v);

	return v;
}

uint32_t atodec32(const char *s)
{
	uint32_t v = 0;
	size_t sl = 0;
	uint8_t l = 0;
	const char *p = s;

	if(s == 0 || strlen(s) == 0) {
		v = 0xffffffff;
		goto err_a2d;
	}

	p = s;
	while(*p != '\0') {
		if((*p >= '0' && *p <= '9')) {
			l++;
			p++;
		} else {/* Error !!! illegal character */
			printf("illegal cahcacter 0x%02x\n", *p);
			v = 0xffffffff;
			goto err_a2d;
		}

		if(l >= 12) /* Max characters count */
			break;
	}

	p = s;
	for(int i=0;i<l;i++) {
		v *= 10;
		if(*p >= '0' && *p <= '9')
			v += (*p - '0');
		p++;
	}

err_a2d:
	//printf("%s = %u\n", s, v);

	return v;
}

uint32_t atohex32(const char *s) {
	uint32_t v = 0;
	size_t sl = 0;
	uint8_t l = 0;
	const char *p = s;

	if(s == 0 || strlen(s) == 0) {
		v = 0xffffffff;
		goto err_a2h;
	}

	sl = strlen(s);
	if(sl >= 2 && s[0] == '0' && (s[1] == 'X' || s[1] == 'x')) {
		s += 2;
	}

	p = s;
	while(*p != '\0') {
		if((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f')) {
			l++;
			p++;
		} else {/* Error !!! illegal character */
			printf("illegal cahcacter 0x%02x\n", *p);
			v = 0xffffffff;
			goto err_a2h;
		}

		if(l >= 8) /* Max characters count */
			break;
	}

	p = s;
	for(int i=0;i<l;i++) {
		v <<= 4;
		if(*p >= '0' && *p <= '9')
			v |= (*p - '0');
		else if(*p >= 'A' && *p <= 'F')
			v |= ((*p - 'A') + 10);
		else if(*p >= 'a' && *p <= 'f')
			v |= ((*p - 'a') + 10);
		p++;
	}

err_a2h:
	//printf("%s = 0x%08x\n", s, v);

	return v;
}

uint16_t atohex16(const char *s) {
	return atohex32(s) & 0xffff;
}

uint16_t atou8(const char *s) {
	if(s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		return atohex8(s);
	else
		return atodec32(s) & 0xff;
}

uint32_t atou32(const char *s) {
	if(s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		return atohex32(s);
	else
		return atodec32(s);
}

uint16_t atou16(const char *s) {
	if(s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		return atohex16(s);
	else
		return atodec32(s) & 0xffff;
}
