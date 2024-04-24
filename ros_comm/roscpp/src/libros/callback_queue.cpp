/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/callback_queue.h"
#include "ros/assert.h"
#include <boost/scope_exit.hpp>

#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/Shore_utils.h"


namespace ros
{

CallbackQueue::CallbackQueue(bool enabled)
: calling_(0)
, enabled_(enabled)
{
}

CallbackQueue::~CallbackQueue()
{
  disable();
}

void CallbackQueue::enable()
{
  boost::mutex::scoped_lock lock(mutex_);
  enabled_ = true;

  condition_.notify_all();
}

void CallbackQueue::disable()
{
  boost::mutex::scoped_lock lock(mutex_);
  enabled_ = false;

  condition_.notify_all();
}

void CallbackQueue::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  callbacks_.clear();
}

bool CallbackQueue::isEmpty()
{
  boost::mutex::scoped_lock lock(mutex_);

  return callbacks_.empty() && calling_ == 0;
}

bool CallbackQueue::isEnabled()
{
  boost::mutex::scoped_lock lock(mutex_);

  return enabled_;
}

void CallbackQueue::setupTLS()
{
  if (!tls_.get())
  {
    tls_.reset(new TLS);
  }
}

void CallbackQueue::addCallback(const CallbackInterfacePtr& callback, uint64_t removal_id)
{
  CallbackInfo info;
  info.callback = callback;
  info.removal_id = removal_id;

  {
    boost::mutex::scoped_lock lock(id_info_mutex_);

    M_IDInfo::iterator it = id_info_.find(removal_id);
    if (it == id_info_.end())
    {
      IDInfoPtr id_info(boost::make_shared<IDInfo>());
      id_info->id = removal_id;
      id_info_.insert(std::make_pair(removal_id, id_info));
    }
  }

  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!enabled_)
    {
      return;
    }

// Ao added NOTE
    callbacks_.push_back(info);
  }

  condition_.notify_one();
}

#ifdef Shore_Sched
/* FOR EXPERIMENTS */
// std::vector<double> durations_creation_update;

void CallbackQueue::Shore_addCallback(const CallbackInterfacePtr& callback,
                                      uint64_t removal_id,
                                      struct ShoreInfo* shore_info) {
  
  /* FOR EXPERIMENTS 
  auto start = std::chrono::high_resolution_clock::now();   
  */

  CallbackInfo info;
  info.callback = callback;
  info.removal_id = removal_id;

#ifdef Shore_Debug
  printf("[Shore-Debug] In Shore_addCallback(), the callback is prioritized, with function pointer %p \n",
         shore_info->callback_func_ptr);
#endif

  // TMP
//   CallbackInfo* tmp = (CallbackInfo*)malloc(sizeof(CallbackInfo));

  info.setShoreCallbackInfoFuncPtr(shore_info->callback_func_ptr);
//   info.setShoreCallbackInfoFuncPtr((void*)tmp);
  info.setPriority(shore_info->priority);
  //FIXME AO
  if (shore_info->priority != 0) {
    info.isShoreActive = 1;
  } else {
    info.isShoreActive = 0;
  }
    /* FOR EXPERIMENTS 
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    durations_creation_update.push_back(duration.count());

    if (durations_creation_update.size() == 100 || durations_creation_update.size() == 200 ||
        durations_creation_update.size() == 500 || durations_creation_update.size() == 1000 ||
        durations_creation_update.size() == 2000 || durations_creation_update.size() == 5000) {
        double sum = 0;
        for (int i = 0; i < durations_creation_update.size(); i++) {
            sum += durations_creation_update[i];
        }
        
        std::cout << "[Shore-Experiments] Creation runtime overhead : " << sum << std::endl;
    }
    */

    // printf("the tmp pointer is %p \n", tmp);
    // delete tmp;
// End of Added code logic

  {
    boost::mutex::scoped_lock lock(id_info_mutex_);

    M_IDInfo::iterator it = id_info_.find(removal_id);
    if (it == id_info_.end())
    {
      IDInfoPtr id_info(boost::make_shared<IDInfo>());
      id_info->id = removal_id;
      id_info_.insert(std::make_pair(removal_id, id_info));
    }
  }

  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!enabled_)
    {
      return;
    }

// Ao added NOTE
    callbacks_.push_back(info);
  }

  condition_.notify_one();
}
#endif // End - Shore_Sched

CallbackQueue::IDInfoPtr CallbackQueue::getIDInfo(uint64_t id) {
    boost::mutex::scoped_lock lock(id_info_mutex_);
    M_IDInfo::iterator it = id_info_.find(id);
    if (it != id_info_.end()) {
        return it->second;
    }

    return IDInfoPtr();
}

void CallbackQueue::removeByID(uint64_t removal_id) {
    setupTLS();

    {
        IDInfoPtr id_info;
        {
            boost::mutex::scoped_lock lock(id_info_mutex_);
            M_IDInfo::iterator it = id_info_.find(removal_id);
            if (it != id_info_.end()) {
                id_info = it->second;
            } else {
                return;
            }
        }

        // If we're being called from within a callback from our queue, we
        // must unlock the shared lock we already own here so that we can
        // take a unique lock.  We'll re-lock it later.
        if (tls_->calling_in_this_thread == id_info->id) {
            id_info->calling_rw_mutex.unlock_shared();
        }

        {
            boost::unique_lock<boost::shared_mutex> rw_lock(
                id_info->calling_rw_mutex, boost::defer_lock);
            if (rw_lock.try_lock()) {
                boost::mutex::scoped_lock lock(mutex_);
                D_CallbackInfo::iterator it = callbacks_.begin();
                for (; it != callbacks_.end();) {
                    CallbackInfo& info = *it;
                    if (info.removal_id == removal_id) {
                        it = callbacks_.erase(it);
                    } else {
                        ++it;
                    }
                }
            } else {
                // We failed to acquire the lock, it can be that we are
                // trying to remove something from the callback queue while
                // it is being executed. Mark it for removal and let it be
                // cleaned up later.
                boost::mutex::scoped_lock lock(mutex_);
                for (D_CallbackInfo::iterator it = callbacks_.begin();
                     it != callbacks_.end(); it++) {
                    CallbackInfo& info = *it;
                    if (info.removal_id == removal_id) {
                        info.marked_for_removal = true;
                    }
                }
            }
        }

        if (tls_->calling_in_this_thread == id_info->id) {
            id_info->calling_rw_mutex.lock_shared();
        }
    }

    // If we're being called from within a callback, we need to remove the
    // callbacks that match the id that have already been popped off the
    // queue
    {
        D_CallbackInfo::iterator it = tls_->callbacks.begin();
        D_CallbackInfo::iterator end = tls_->callbacks.end();
        for (; it != end; ++it) {
            CallbackInfo& info = *it;
            if (info.removal_id == removal_id) {
                info.marked_for_removal = true;
            }
        }
    }

    {
        boost::mutex::scoped_lock lock(id_info_mutex_);
        id_info_.erase(removal_id);
    }
}

CallbackQueue::CallOneResult CallbackQueue::callOne(ros::WallDuration timeout) {
    setupTLS();
    TLS* tls = tls_.get();

    CallbackInfo cb_info;

    {
        boost::mutex::scoped_lock lock(mutex_);

        if (!enabled_) {
            return Disabled;
        }

        if (callbacks_.empty()) {
            if (!timeout.isZero()) {
                condition_.wait_for(
                    lock, boost::chrono::nanoseconds(timeout.toNSec()));
            }

            if (callbacks_.empty()) {
                return Empty;
            }

            if (!enabled_) {
                return Disabled;
            }
        }

        D_CallbackInfo::iterator it = callbacks_.begin();
        for (; it != callbacks_.end();) {
            CallbackInfo& info = *it;

            if (info.marked_for_removal) {
                it = callbacks_.erase(it);
                continue;
            }

            if (info.callback->ready()) {
                cb_info = info;
                it = callbacks_.erase(it);
                break;
            }

            ++it;
        }

        if (!cb_info.callback) {
            return TryAgain;
        }

        ++calling_;
    }

    bool was_empty = tls->callbacks.empty();
    tls->callbacks.push_back(cb_info);
    if (was_empty) {
        tls->cb_it = tls->callbacks.begin();
    }

    CallOneResult res = callOneCB(tls);
    if (res != Empty) {
        boost::mutex::scoped_lock lock(mutex_);
        --calling_;
    }
    return res;
}

void CallbackQueue::callAvailable(ros::WallDuration timeout) {
    setupTLS();
    TLS* tls = tls_.get();

    {
        boost::mutex::scoped_lock lock(mutex_);

        if (!enabled_) {
            return;
        }

        if (callbacks_.empty()) {
            if (!timeout.isZero()) {
                condition_.wait_for(
                    lock, boost::chrono::nanoseconds(timeout.toNSec()));
            }

            if (callbacks_.empty() || !enabled_) {
                return;
            }
        }

        bool was_empty = tls->callbacks.empty();

        tls->callbacks.insert(tls->callbacks.end(), callbacks_.begin(),
                              callbacks_.end());
        callbacks_.clear();

        calling_ += tls->callbacks.size();

        if (was_empty) {
            tls->cb_it = tls->callbacks.begin();
        }
    }

    size_t called = 0;

    while (!tls->callbacks.empty()) {
        if (callOneCB(tls) != Empty) {
            ++called;
        }
    }

    {
        boost::mutex::scoped_lock lock(mutex_);
        calling_ -= called;
    }
}

#ifdef Shore_Sched

// Scheduling related functions
void CallbackQueue::sortQueueByShorePriority(D_CallbackInfo& callbacks) {
  std::sort(callbacks.begin(), callbacks.end(),
            [](const CallbackInfo& a, const CallbackInfo& b) {
              return a.priority > b.priority;
            });
}

#endif

/* FOR EXPERIMENTS */
// std::vector<double> durations_sched;

CallbackQueue::CallOneResult CallbackQueue::callOneCB(TLS* tls) {
#ifdef Shore_Debug
    // printf("[Shore-Debug] in CallbackQueue::callOneCB\n");
#endif

#ifdef Shore_Sched


    /* FOR EXPERIMENTS 
    auto start = std::chrono::high_resolution_clock::now();
    */
    // Sort the callback queue to prioritize
    if (isShoreSchedActive()) {
        sortQueueByShorePriority(tls->callbacks);
// #ifdef Shore_Debug
        printf("[Shore-Debug] ROS callback is prioritizing. Total queue size %d \n", tls->callbacks.size());
// #endif
    }

    /* FOR EXPERIMENTS 
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    durations_sched.push_back(duration.count());

    if (durations_sched.size() == 100 || durations_sched.size() == 200 ||
        durations_sched.size() == 500 || durations_sched.size() == 1000 ||
        durations_sched.size() == 2000 || durations_sched.size() == 5000) {
        double sum = 0;
        for (int i = 0; i < durations_sched.size(); i++) {
            sum += durations_sched[i];
        }
      
        std::cout << "[Shore-Experiments] Scheduler runtime overhead : " << sum << std::endl;
    }
    */

#ifdef Shore_Debug
    for (auto it = tls->callbacks.begin(); it != tls->callbacks.end(); ++it) {
        printf("[Shore-Debug] In the callOneCB function, the callback priority is %d "
               "\n",
               it->priority);
    }
#endif // Debug    
#endif // Sched


    // Check for a recursive call.  If recursive, increment the current
    // iterator.  Otherwise set the iterator it the beginning of the
    // thread-local callbacks
    if (tls->calling_in_this_thread == 0xffffffffffffffffULL) {
        tls->cb_it = tls->callbacks.begin();
    }

    if (tls->cb_it == tls->callbacks.end()) {
        return Empty;
    }

    ROS_ASSERT(!tls->callbacks.empty());
    ROS_ASSERT(tls->cb_it != tls->callbacks.end());

    CallbackInfo info = *tls->cb_it;
    CallbackInterfacePtr& cb = info.callback;

#ifdef Shore_Debug
    // For debugging purposes
    // printf(
    //     "[Shore-Debug] In the callOneCB function, the callback function pointer is %p "
    //     "\n",
    //     info.Shore_callbackinfo_func_ptr);

#endif

    IDInfoPtr id_info = getIDInfo(info.removal_id);
    if (id_info) {
        boost::shared_lock<boost::shared_mutex> rw_lock(
            id_info->calling_rw_mutex);

        uint64_t last_calling = tls->calling_in_this_thread;
        tls->calling_in_this_thread = id_info->id;

        CallbackInterface::CallResult result = CallbackInterface::Invalid;

        {
            // Ensure that thread id gets restored, even if callback throws.
            // This is done with RAII rather than try-catch so that the source
            // of the original exception is not masked in a crash report.
            BOOST_SCOPE_EXIT(&tls, &last_calling) {
                tls->calling_in_this_thread = last_calling;
            }
            BOOST_SCOPE_EXIT_END

            if (info.marked_for_removal) {

        tls->cb_it = tls->callbacks.erase(tls->cb_it);
      }
      else
      {
        tls->cb_it = tls->callbacks.erase(tls->cb_it);
        result = cb->call();

#ifdef Shore_Sched
        // Reset the priority to default
        if(tls->cb_it->isShoreActive == 1) {
            resetShorePriorityToDefaultbyCallbackFuncPtr(tls->cb_it->Shore_callbackinfo_func_ptr);
        }
#endif

      }
    }

    // Push TryAgain callbacks to the back of the shared queue
    if (result == CallbackInterface::TryAgain && !info.marked_for_removal)
    {
      boost::mutex::scoped_lock lock(mutex_);
      callbacks_.push_back(info);

      return TryAgain;
    }

    return Called;
  }
  else
  {
    tls->cb_it = tls->callbacks.erase(tls->cb_it);
  }

  return Called;
}
}
