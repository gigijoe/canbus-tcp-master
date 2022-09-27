#ifndef _CSV_PARSER_H
#define _CSV_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

char** parse_csv_line(char* line, int *argc, const char token[]);

#ifdef __cplusplus
}
#endif

#endif
