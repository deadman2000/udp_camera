#pragma once

#include <array>
#include <memory>
#include <queue>

static constexpr int READ_BUFF_SIZE = 2048;
static constexpr int CHUNKS_COUNT = 50;

struct buff_t
{
    std::array<char, READ_BUFF_SIZE> data;
    int size;
};

using buff_ptr = std::shared_ptr<buff_t>;

class BufferManager
{
    std::queue<buff_ptr> _idle;
    std::vector<buff_ptr> _completed;

  public:
    BufferManager()
    {
        for (int i = 0; i < CHUNKS_COUNT; ++i)
            _idle.emplace(std::make_shared<buff_t>());
    }

    buff_ptr get_buffer()
    {
        if (_idle.empty())
            return nullptr;

        auto buff = _idle.front();
        _idle.pop();
        return buff;
    }

    void complete(buff_ptr &buff)
    {
        _completed.push_back(buff);
    }

    std::vector<buff_ptr> pop_completed()
    {
        std::vector<buff_ptr> copy = _completed;
        _completed.clear();
        return copy;
    }

    void return_back(std::vector<buff_ptr> &buffers)
    {
        for (auto &buff : buffers)
            _idle.push(buff);
    }

    void reset()
    {
        for (auto &buff : _completed)
            _idle.push(buff);
        _completed.clear();
    }

    int completed_count() const
    {
        return _completed.size();
    }
};