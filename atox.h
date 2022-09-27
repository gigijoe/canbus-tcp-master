#ifndef _ATOX_H
#define _ATOX_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t atohex8(const char *s);
uint32_t atodec32(const char *s);
uint32_t atohex32(const char *s);
uint16_t atohex16(const char *s);
uint16_t atou8(const char *s);
uint32_t atou32(const char *s);
uint16_t atou16(const char *s);

#ifdef __cplusplus
}
#endif

#endif
