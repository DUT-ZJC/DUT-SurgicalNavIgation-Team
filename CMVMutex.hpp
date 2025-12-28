#pragma once
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
    CRITICAL_SECTION    m_Mutex;
};

#endif // !CMVMUTEX_HPP
