#ifndef OBE_QUEUE_H
#define OBE_QUEUE_H

#include <stdio.h>
#include <pthread.h>

typedef struct
{
    char name[128];
    void **queue;
    int  size;

    pthread_mutex_t mutex;
    pthread_cond_t  in_cv;
    pthread_cond_t  out_cv;
} obe_queue_t;

void obe_init_queue(obe_queue_t *queue, char *name);
void obe_destroy_queue(obe_queue_t *queue);
int  add_to_queue(obe_queue_t *queue, void *item);
int  remove_from_queue_without_lock(obe_queue_t *queue);
int  remove_from_queue(obe_queue_t *queue);
int  remove_item_from_queue(obe_queue_t *queue, void *item);

#endif /* OBE_QUEUE_H */
