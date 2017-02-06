#pragma once
#include <mutex>

class SafeMutex
{
public:
	SafeMutex(std::mutex& mutex) : mMutex(mutex) { mMutex.lock(); }
	~SafeMutex() { mMutex.unlock(); }
	
private:
	std::mutex& mMutex;
};

#define SAFE_MUTEX(mutex) SafeMutex safeMutex(mutex) 