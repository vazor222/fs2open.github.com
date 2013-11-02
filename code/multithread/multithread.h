#ifndef _MULTITHREAD_H
#define _MULTITHREAD_H

#include "object/object.h"
#include <pthread.h>

#define MULTITHREADING_ENABLED
//#define MULTITHREADING_ENABLED2
//#define MULTITHREADING_ENABLED3
#define MAX_THREADS											256

#define SAFETY_TIME											2

#define THREAD_WAIT											-1
#define THREAD_EXIT											-2

#ifdef MULTITHREADING_ENABLED
#define OPENGL_LOCK											{pthread_mutex_lock(&render_mutex);}
#define OPENGL_UNLOCK										{pthread_mutex_unlock(&render_mutex);}
#define G3_COUNT_LOCK										{pthread_mutex_lock(&g3_count_mutex);}
#define G3_COUNT_UNLOCK										{pthread_mutex_unlock(&g3_count_mutex);}
#define HOOK_LOCK											{pthread_mutex_lock(&hook_mutex);}
#define HOOK_UNLOCK											{pthread_mutex_unlock(&hook_mutex);}
#else
#define OPENGL_LOCK
#define OPENGL_UNLOCK
#define G3_COUNT_LOCK
#define G3_COUNT_UNLOCK
#define HOOK_LOCK
#define HOOK_UNLOCK
#endif

typedef struct {
	pthread_t handle;
	pthread_mutex_t mutex;
	pthread_cond_t condition;
} thread_condition;

typedef struct {
	object *object_1;
	object *object_2;
//    unsigned char flags;
	bool processed;
	void *operation_func;
} collision_pair;

typedef enum {
//  THREAD_TYPE_OBJECT_MOVE,
//  THREAD_TYPE_DOCKING,
	THREAD_TYPE_COLLISION,
	THREAD_TYPE_INVALID
} thread_type;

typedef enum {
	COLLISION_PAIR_FLAG_COLLIDED,
	COLLISION_PAIR_FLAG_RESULT,
	COLLISION_PAIR_FLAG_EXECUTED,
	COLLISION_PAIR_FLAG_INVALID
} collision_pair_flag;

extern pthread_mutex_t render_mutex;
extern pthread_mutex_t g3_count_mutex;
extern pthread_mutex_t hook_mutex;
extern bool threads_alive;

void create_threads();
void destroy_threads();

void execute_collisions();

/**
 * We do not expect this function to run inside a thread
 * @param object_1
 * @param object_2
 */
void collision_pair_add(object *object_1, object *object_2);

void collision_pair_clear();

#endif
