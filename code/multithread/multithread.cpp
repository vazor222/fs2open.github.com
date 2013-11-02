#include <pthread.h>

#include "asteroid/asteroid.h"
#include "cmeasure/cmeasure.h"
#include "debris/debris.h"
#include "fireball/fireballs.h"
#include "freespace2/freespace.h"
#include "globalincs/linklist.h"
#include "iff_defs/iff_defs.h"
#include "io/timer.h"
#include "jumpnode/jumpnode.h"
#include "lighting/lighting.h"
#include "mission/missionparse.h"
#include "network/multi.h"
#include "network/multiutil.h"
#include "object/objcollide.h"
#include "object/object.h"
#include "object/objectdock.h"
#include "object/objectshield.h"
#include "object/objectsnd.h"
#include "observer/observer.h"
#include "parse/scripting.h"
#include "playerman/player.h"
#include "radar/radar.h"
#include "radar/radarsetup.h"
#include "render/3d.h"
#include "ship/afterburner.h"
#include "ship/ship.h"
#include "weapon/beam.h"
#include "weapon/shockwave.h"
#include "weapon/swarm.h"
#include "weapon/weapon.h"

#include "multithread/multithread.h"

#include <time.h>

extern int Cmdline_num_threads;

extern void game_shutdown(void);

extern int G3_count;

pthread_attr_t attr_thread;
pthread_mutexattr_t attr_mutex;
pthread_mutexattr_t attr_mutex_recursive;
pthread_mutex_t collision_master_mutex;
pthread_cond_t collision_master_condition;
pthread_mutex_t render_mutex;
pthread_mutex_t g3_count_mutex;
pthread_mutex_t hook_mutex;

SCP_vector<collision_pair> collision_list;
SCP_vector<unsigned int> thread_number;
SCP_vector<collision_pair> thread_collision_vars;
SCP_vector<thread_condition> conditions;

bool threads_alive = false;

unsigned int executions = 0;
int threads_used_record = 0;

timespec wait_time = { 2, 0 };

void *supercollider_thread(void *obj_collision_vars_ptr);

void create_threads()
{
	int i;
	collision_pair setup_pair;

	threads_alive = true;
	setup_pair.object_1 = NULL;
	setup_pair.object_2 = NULL;
	setup_pair.processed = true;

	pthread_attr_init(&attr_thread);
	pthread_mutexattr_init(&attr_mutex);
	pthread_mutexattr_init(&attr_mutex_recursive);

	pthread_attr_setdetachstate(&attr_thread, PTHREAD_CREATE_JOINABLE);

	pthread_mutexattr_settype(&attr_mutex_recursive, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&collision_master_mutex, NULL);
	pthread_cond_init(&collision_master_condition, NULL);
	pthread_mutex_init(&render_mutex, &attr_mutex_recursive);
	pthread_mutex_init(&g3_count_mutex, &attr_mutex_recursive);
	pthread_mutex_init(&hook_mutex, &attr_mutex_recursive);

	if (Cmdline_num_threads < 1) {
		Cmdline_num_threads = 1;
	}
	conditions.resize(Cmdline_num_threads);

	//ensure these numbers are filled before creating our threads
	for (i = 0; i < Cmdline_num_threads; i++) {
		thread_number.push_back(i);
	}

	for (i = 0; i < Cmdline_num_threads; i++) {
		nprintf(("Multithread", "multithread: Creating thread %d\n", i));
		thread_collision_vars.push_back(setup_pair);
		pthread_mutex_init(&(conditions[i].mutex), NULL);
		pthread_cond_init(&(conditions[i].condition), NULL);
		pthread_create(&(conditions[i].handle), &attr_thread, supercollider_thread, &(thread_number[i]));
	}
}

void destroy_threads()
{
	int i;

	threads_alive = false;
	//kill our threads, they shouldn't be doing anything anyway
	for (i = 0; i < Cmdline_num_threads; i++) {
		nprintf(("Multithread", "multithread: destroying thread %d\n", i));
		pthread_cancel(conditions[i].handle);
		pthread_mutex_destroy(&(conditions[i].mutex));
		pthread_cond_destroy(&(conditions[i].condition));
	}
	pthread_mutex_destroy(&render_mutex);
	pthread_mutex_destroy(&g3_count_mutex);
	pthread_mutex_destroy(&collision_master_mutex);
	pthread_cond_destroy(&collision_master_condition);

	pthread_attr_destroy(&attr_thread);
	pthread_mutexattr_destroy(&attr_mutex);
	pthread_mutexattr_destroy(&attr_mutex_recursive);
}

void collision_pair_clear()
{
	collision_list.clear();
}

void collision_pair_add(object *object_1, object *object_2)
{
	collision_pair pair;

	pair.object_1 = object_1;
	pair.object_2 = object_2;
	pair.processed = false;
	pair.operation_func = NULL;

	collision_list.push_back(pair);
}

void execute_collisions()
{
	int i;
	unsigned int object_counter = 0;
	unsigned int loop_counter = 0;
	SCP_vector<collision_pair>::iterator it;
	bool assigned_any, assigned_once = false;
	bool done = false;
	bool skip = false;
	int threads_used = 1;

	time_t start_time = time(NULL);

	OPENGL_LOCK
	if(!G3_count){
		g3_start_frame(1);
	}
	nprintf(("Multithread", "multithread: execution %d start\n", executions));

	while (done == false) {
		//go through our collision list
		nprintf(("Multithread", "multithread: execution %d start main loop %d\n", executions, loop_counter));
		if ((time(NULL) - start_time) > SAFETY_TIME) {
			nprintf(("Multithread", "multithread: execution %d - STUCK\n", executions));
			Int3();
			for (i = 0; i < Cmdline_num_threads; i++) {
				mprintf(("multithread: destroying thread %d\n", i));
				pthread_cancel(conditions[i].handle);
				pthread_mutex_destroy(&(conditions[i].mutex));
				pthread_cond_destroy(&(conditions[i].condition));
			}
			Error(LOCATION, "Encountered fatal error in multithreading\n");
			exit(-1);
		}
//		mprintf(("multithread: execution %d - passed time check\n", executions));
		assigned_any = false;
		object_counter = 0;
//		mprintf(("multithread: execution %d - start list loop\n", executions));
		for (it = collision_list.begin(); it != collision_list.end(); it++, object_counter++) {
			assigned_once = false;
			skip = false;
			if (it->processed == true) {
//				mprintf(("multithread: execution %d object pair %d - skip (already processed)\n", executions, object_counter));
				continue;
			}
			//skip if the pairs are obviously wrong
			if ((it->object_1 == NULL) || (it->object_2 == NULL) || (it->object_1 == it->object_2)) {
				nprintf(("Multithread", "multithread: execution %d object pair %d - invalid objects\n", executions, object_counter));
				it->processed = true;
				continue;
			}

			//ensure the objects being checked aren't already being used
			for (i = 0; i < Cmdline_num_threads; i++) {
				if ((it->object_1 == thread_collision_vars[i].object_1) || (it->object_1 == thread_collision_vars[i].object_2) || (it->object_2 == thread_collision_vars[i].object_1) || (it->object_2 == thread_collision_vars[i].object_2)) {
					skip = true;
					break;
				}
			}

			if (skip == true) {
//				mprintf(("multithread: execution %d object pair %d - skip (busy)\n", executions, object_counter));
				continue;
			}
			//check for a free thread to handle the collision
			for (i = 0; i < Cmdline_num_threads; i++) {
				if (pthread_mutex_trylock(&(conditions[i].mutex)) == 0) {
					if (thread_collision_vars[i].processed == false) {
						nprintf(("Multithread", "multithread: execution %d object pair %d - thread %d busy\n", executions, object_counter, i));
						pthread_mutex_unlock(&(conditions[i].mutex));
						continue;
					}
//					mprintf(("multithread: execution %d object pair %d - passed to thread %d\n", executions, object_counter, i));
					thread_collision_vars[i].object_1 = it->object_1;
					thread_collision_vars[i].object_2 = it->object_2;
					thread_collision_vars[i].processed = false;
					it->processed = true;

					pthread_cond_signal(&(conditions[i].condition));
					pthread_mutex_unlock(&(conditions[i].mutex));
					assigned_once = true;
					if((i + 1) > threads_used)
					{
						threads_used = (i + 1);
					}
					break;
				}
			}

//			mprintf(("multithread: execution %d object pair %d - middle\n", executions, object_counter));
//			if (assigned_once == false) {
//				mprintf(("multithread: execution %d object pair %d - NONE_FREE\n", executions, object_counter));
////				Int3();
//				//if there weren't any free threads, wait until one is free - if we wait too long, skip
//				if (pthread_mutex_timedlock(&collision_master_mutex, &wait_time) == 0) {
//					pthread_cond_wait(&collision_master_condition, &collision_master_mutex);
//
//					//then check again
//					for (i = 0; i < Cmdline_num_threads; i++) {
//						if (pthread_mutex_trylock(&(conditions[i].mutex)) == 0) {
//							if (thread_collision_vars[i].processed == false) {
//								mprintf(("multithread: execution %d object pair %d - thread %d busy after wait\n", executions, object_counter, i));
//								pthread_mutex_unlock(&(conditions[i].mutex));
//								continue;
//							}
//							mprintf(("multithread: execution %d object pair %d - processed by thread %d after wait\n", executions, object_counter, i));
//							thread_collision_vars[i].object_1 = it->object_1;
//							thread_collision_vars[i].object_2 = it->object_2;
//							thread_collision_vars[i].processed = false;
//							it->processed = true;
//
//							pthread_cond_signal(&(conditions[i].condition));
//							pthread_mutex_unlock(&(conditions[i].mutex));
//							assigned_once = true;
//							if((i + 1) > threads_used)
//							{
//								threads_used = (i + 1);
//							}
//							break;
//						}
//					}
//					pthread_mutex_unlock(&collision_master_mutex);
//				} else {
//					//we deadlocked - quit the game
//					mprintf(("multithread: execution %d object pair %d - DEADLOCK\n", executions, object_counter));
//					Int3();
//					for (i = 0; i < Cmdline_num_threads; i++) {
//						mprintf(("multithread: destroying thread %d\n", i));
//						pthread_cancel(conditions[i].handle);
//						pthread_mutex_destroy(&(conditions[i].mutex));
//						pthread_cond_destroy(&(conditions[i].condition));
//					}
//					game_shutdown();
//					exit(-1);
//				}
//			}
//			if (assigned_once == true) {
//				assigned_any = true;
//			}
		}
//		mprintf(("multithread: execution %d - end list loop\n", executions));
//		if (assigned_any == false) {
//			mprintf(("multithread: execution %d - INVALID\n", executions));
//			Int3();
//			for (i = 0; i < Cmdline_num_threads; i++) {
//				mprintf(("multithread: destroying thread %d\n", i));
//				pthread_cancel(conditions[i].handle);
//				pthread_mutex_destroy(&(conditions[i].mutex));
//				pthread_cond_destroy(&(conditions[i].condition));
//			}
//		}
//		mprintf(("multithread: execution %d - final check\n", executions));

		//make sure we processs everything on the list
		done = true;
		for (it = collision_list.begin(); it != collision_list.end(); it++) {
			if (it->processed == false) {
				nprintf(("Multithread", "multithread: execution %d - looping back\n", executions));
				done = false;
				loop_counter++;
				break;
			}
		}
	}
	if(threads_used > threads_used_record)
	{
		threads_used_record = threads_used;
	}
	nprintf(("Multithread", "multithread: execution %d - %d collision pairs, %d threads used - record is %d threads used\n", executions, collision_list.size(), threads_used, threads_used_record));

	//make sure all the threads are done executing
	for (i = 0; i < Cmdline_num_threads; i++) {
		pthread_mutex_lock(&(conditions[i].mutex));
		pthread_mutex_unlock(&(conditions[i].mutex));
	}
	executions++;
	g3_end_frame();
	OPENGL_UNLOCK
}

void *supercollider_thread(void *num)
{
	int thread_num = *(int *) num;

	nprintf(("Multithread", "multithread: supercollider_thread %d started\n", thread_num));

	pthread_mutex_lock(&(conditions[thread_num].mutex));
	while (threads_alive) {
//		mprintf(("multithread: supercollider_thread %d ready\n", thread_num));
		pthread_cond_wait(&(conditions[thread_num].condition), &(conditions[thread_num].mutex));

//		mprintf(("multithread: supercollider_thread %d processing\n", thread_num));
		if ((thread_collision_vars[thread_num].object_1 != NULL) && (thread_collision_vars[thread_num].object_2 != NULL)) {
			obj_collide_pair(thread_collision_vars[thread_num].object_1, thread_collision_vars[thread_num].object_2);
			thread_collision_vars[thread_num].object_1 = NULL;
			thread_collision_vars[thread_num].object_2 = NULL;
		}
		thread_collision_vars[thread_num].processed = true;
//		mprintf(("multithread: supercollider_thread %d done\n", thread_num));

		pthread_mutex_lock(&collision_master_mutex);
		pthread_cond_signal(&collision_master_condition);
		pthread_mutex_unlock(&collision_master_mutex);

//		pthread_yield();
	}
	pthread_mutex_unlock(&(conditions[thread_num].mutex));
	pthread_exit(NULL);

	return 0;
}
