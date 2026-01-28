#ifndef DATACACHE_HPP
#define DATACACHE_HPP

#include <vector>
template <typename T>

class dataCache
{
public:
    datacache(int size) : size(size), buffer(size), head(0), tail(0), count(0) {}
    dataCache() : size(0), head(0), tail(0), count(0) {} //default constructor

    bool isFull() const
    {
        return this->count == this->size;
    }
    bool isEmpty() const
    {
        return this->count == 0;
    }
    int getCount()
    {
        return this->count;
    }
    void enqueue(const T &data)
    {
        buffer[head] = data;
        head = (head + 1) % size;

        if (isFull())
        {
            tail = (tail + 1) % size;
        }
        else
        {
            count += 1;
        }
    }
    T newestValue() const
    {
        if (!isEmpty())
        {
            int newestIndex = (head - 1 + size) % size;
            return buffer[newestIndex];
        }
        else
        {
            return T{};
        }
    }
private:
    int size;
    int head;
    int tail;
    int count;
    std::vector<T> buffer = {};
};

#endif