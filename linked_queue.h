/* LinkedQueue.h */

#include "defs.h"

/* Type definitions */
typedef struct {
    material_t item_type; 	/* stores a number describing the element */
    uint16_t item_min;
    // add more item variables here
} element;

typedef struct link{
    element		e;
    struct link *next;
} link;

void	initLink	(link **newLink);
void 	setup		(link **h, link **t);
void 	clearQueue	(link **h, link **t);
void 	enqueue		(link **h, link **t, link **nL);
void 	dequeue		(link **h, link **t, link **deQueuedLink);
element firstValue	(link *h);
char 	isEmpty		(link *h);
int 	size		(link *h);

