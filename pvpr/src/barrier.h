#ifndef BARRIER_H
#define BARRIER_H

#include <pthread.h>

typedef struct { 
	pthread_mutex_t mutex;
	pthread_cond_t cv;
	int nthreads;
	int arrived;
} barrier_t;

void barrier_init(barrier_t *, int);
void barrier_wait_run(barrier_t *, void (*run_func)(void *), void *arg);
void barrier_wait(barrier_t *);

#endif
