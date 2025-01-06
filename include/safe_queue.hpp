#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

// A threadsafe-queue.
template <class T>
class SafeQueue
{
    std::queue<T> q;
    std::mutex m;
    std::condition_variable c;
    bool _stopped;

  public:
    SafeQueue()
        : q(), m(), c(), _stopped(false)
    {
    }

    void stop()
    {
        _stopped = true;
        c.notify_one();
    }

    // Add an element to the queue.
    void enqueue(T t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    // Get the "front"-element.
    // If the queue is empty, wait till a element is avaiable.
    T dequeue()
    {
        std::unique_lock<std::mutex> lock(m);
        c.wait(lock, [&] { return !q.empty() || _stopped; });
        if (_stopped)
            return T();

        T val = q.front();
        q.pop();
        return val;
    }

    int size() const
    {
        return q.size();
    }
};