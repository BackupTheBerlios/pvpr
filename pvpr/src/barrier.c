#include <pthread.h>
#include "barrier.h"

void barrier_init(barrier_t *barrier, int nthreads) {
	pthread_mutex_init(&barrier->mutex, NULL);
	pthread_cond_init(&barrier->cv, NULL);
	barrier->nthreads = nthreads;
	barrier->arrived = 0;
}

void barrier_wait_run(barrier_t *, void (*run_func)(void *), void *arg) {
	
	pthread_mutex_lock(&barrier->mutex);
	
	(barrier->arrived)++;
	
	if (barrier->arrived < barrier->nthreads)
		pthread_cond_wait(&barrier->cv, &barrier->mutex);
	else {
		barrier->arrived = 0;
		(*run_func)(arg);
		pthread_cond_broadcast(&barrier->cv);
	}
	pthread_mutex_unlock(&barrier->mutex);
}
		

void barrier_wait(barrier_t *barrier) {

	pthread_mutex_lock(&barrier->mutex);

	(barrier->arrived)++;

	if (barrier->arrived < barrier->nthreads)
		pthread_cond_wait(&barrier->cv, &barrier->mutex);
	else {
		pthread_cond_broadcast(&barrier->cv);
		barrier->arrived = 0; /* be prepared for next barrier */
	}
	
	pthread_mutex_unlock(&barrier->mutex);
}
