#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define CSV_COLUMN_MAX 32

char** parse_csv_line(char* line, int *argc, const char token[] )
{
    static char *argv[CSV_COLUMN_MAX];

    int c = 0;
    char* tok;
    for (tok = strtok(line, token);
        tok && *tok;
        tok = strtok(NULL, token))
    {
        char *p = tok;
        
        while(*p <= 0x20) /* Strip space ahead */
            ++p;

        size_t sl = strlen(p);

        while(sl > 0 && p[sl- 1] <= 0x20) /* Strip space on tail */
            sl--;

        if(sl == 0) { /* space only ... */
            //p[0] = '\0';
            continue; /* Illegal item ... */
        } else {
            p[sl] = '\0';
        }

        argv[c] = p;
        c++;

        if(c >= CSV_COLUMN_MAX)
            break;
    }

    *argc = c;

    return argv;
}