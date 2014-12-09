//
//  SharedQueue.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/3/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef SHARED_QUEUE_H
#define SHARED_QUEUE_H

#include <iostream>
#include <mutex>
#include <deque>

template <typename T>
class SharedQueue {
public:
	SharedQueue();
	~SharedQueue();
	bool IsEmpty() const;
	void Add(T value);
	bool Remove(T &item);
	void Print();
	size_t size();
private:
	std::deque<T> queue;
	mutable std::mutex lock;
};

template <typename T>
SharedQueue<T>::SharedQueue()
{
}

template <typename T>
SharedQueue<T>::~SharedQueue()
{
}

template <typename T>
bool SharedQueue<T>::IsEmpty() const
{
	lock.lock();
	bool result = (queue.empty());
	lock.unlock();
	return result;
}

template <typename T>
size_t SharedQueue<T>::size()
{
	lock.lock();
	size_t result = queue.size();
	lock.unlock();
	return result;
}

template <typename T>
void SharedQueue<T>::Add(T value)
{
	lock.lock();
	queue.push_back(value);
	lock.unlock();
}

template <typename T>
bool SharedQueue<T>::Remove(T &item)
{
	lock.lock();
	if (queue.empty())
	{
		lock.unlock();
		return false;
	}
	
	item = queue.front();
	queue.pop_front();
	lock.unlock();
	return true;
}

template <typename T>
void SharedQueue<T>::Print()
{
	lock.lock();
	for (auto c : queue)
	{
		std::cout << c << " ";
	}
	std::cout << std::endl;
	lock.unlock();
}


#endif
