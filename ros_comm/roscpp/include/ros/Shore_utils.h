#ifndef ROS_SHORE_UTILS_H
#define ROS_SHORE_UTILS_H


#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"
#include "ros/timer.h"
#include "ros/rate.h"
#include "ros/wall_timer.h"
#include "ros/steady_timer.h"
#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/subscribe_options.h"
#include "ros/service_client_options.h"
#include "ros/timer_options.h"
#include "ros/wall_timer_options.h"
#include "ros/spinner.h"
#include "ros/init.h"
#include "common.h"

#include "ros/param.h"

/*

#define MAX_NUM_TDG_VERTICES 10

struct Shore_info
{
    int Shoreline_id;
    void* callback_funcion_ptr;
    int priority;
    int num_related_timing_graph_vertices;
    int* related_timing_graph_vertices[MAX_NUM_TDG_VERTICES];
    pid_t kernel_task_id;
};

*/





#endif // ROS_SHORE_UTILS_H