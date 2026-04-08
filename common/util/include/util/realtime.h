// common/util/include/util/realtime.h
// Thread configuration: naming, CPU affinity, RT scheduling.
#pragma once
#include "util/ilogger.h"

#include <cstdio>

#include <pthread.h>
#include <sched.h>

inline void set_thread_params(const char* name, int core, int policy, int priority) {
    // Set thread name (max 15 chars + null)
    char tname[16];
    snprintf(tname, sizeof(tname), "%s", name);
    pthread_setname_np(pthread_self(), tname);

    // Pin to CPU core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (rc != 0) DRONE_LOG_WARN("setaffinity({}) failed: {}", core, rc);

    // Set scheduling policy and priority (requires CAP_SYS_NICE or root)
    if (policy == SCHED_FIFO || policy == SCHED_RR) {
        sched_param param{};
        param.sched_priority = priority;
        rc                   = pthread_setschedparam(pthread_self(), policy, &param);
        if (rc != 0) DRONE_LOG_WARN("setschedparam({},{}) failed: {}", policy, priority, rc);
    }

    DRONE_LOG_DEBUG("Thread '{}': core={}, policy={}, priority={}", name, core, policy, priority);
}
