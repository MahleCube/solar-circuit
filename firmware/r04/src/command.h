#ifndef C07_COMMAND_H
#define C07_COMMAND_H
#include <stdbool.h>
#include <string.h>

/* ASCII command frames end at LF; CR is allowed only immediately before LF.
 * Any malformed byte invalidates the whole frame, never a valid prefix. */
typedef enum { C07_COMMAND_NONE, C07_COMMAND_ARM, C07_COMMAND_DISARM,
               C07_COMMAND_RESET, C07_COMMAND_STATUS } c07_command_event;
typedef struct {
    char line[16];
    unsigned length;
    bool invalid, after_cr;
} c07_command_parser;

static inline c07_command_event c07_command_feed(c07_command_parser *p,int byte) {
    if(byte=='\n') {
        c07_command_event event=C07_COMMAND_NONE;
        p->line[p->length]='\0';
        if(!p->invalid) {
            if(!strcmp(p->line,"ARM")) event=C07_COMMAND_ARM;
            else if(!strcmp(p->line,"DISARM")) event=C07_COMMAND_DISARM;
            else if(!strcmp(p->line,"RESET")) event=C07_COMMAND_RESET;
            else if(!strcmp(p->line,"STATUS")) event=C07_COMMAND_STATUS;
        }
        p->length=0; p->invalid=false; p->after_cr=false;
        return event;
    }
    if(p->invalid) return C07_COMMAND_NONE;
    if(p->after_cr) { p->invalid=true; return C07_COMMAND_NONE; }
    if(byte=='\r') { p->after_cr=true; return C07_COMMAND_NONE; }
    if(byte<0x20 || byte>0x7e || p->length>=sizeof(p->line)-1) {
        p->invalid=true; return C07_COMMAND_NONE;
    }
    p->line[p->length++]=(char)byte;
    return C07_COMMAND_NONE;
}
#endif
