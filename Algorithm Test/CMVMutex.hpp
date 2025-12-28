#ifndef CMVMUTEX_HPP
#define CMVMUTEX_HPP
#include <Windows.h>

class CMVMutex
{
public:
    CMVMutex()
    {
        InitializeCriticalSection(&m_Mutex);
    }
    ~CMVMutex()
    {
        DeleteCriticalSection(&m_Mutex);
    }
    void _Lock()
    {
        EnterCriticalSection(&m_Mutex);
    }
    void _Unlock()
    {
        LeaveCriticalSection(&m_Mutex);
    }
private:
    CRITICAL_SECTION m_Mutex;

    // 禁止拷贝和赋值
    CMVMutex(const CMVMutex&) = delete;
    CMVMutex& operator=(const CMVMutex&) = delete;
};

// RAII 自动锁类
class CMVAutoLock
{
public:
    explicit CMVAutoLock(CMVMutex& mutex) : m_Mutex(mutex)
    {
        m_Mutex._Lock();
    }
    ~CMVAutoLock()
    {
        m_Mutex._Unlock();
    }
private:
    CMVMutex& m_Mutex;

    // 禁止拷贝和赋值
    CMVAutoLock(const CMVAutoLock&) = delete;
    CMVAutoLock& operator=(const CMVAutoLock&) = delete;
};

#endif // !CMVMUTEX_HPP