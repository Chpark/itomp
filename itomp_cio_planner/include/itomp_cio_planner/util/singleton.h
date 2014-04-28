/*
 * Singleton.h
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{

template<class T>
class Singleton
{
public:
        virtual ~Singleton(void) {}
        static T* getInstance();

protected:
        Singleton(void) {}
        static T* instance_;
};

template<class T>
T* Singleton<T>::instance_ = NULL;

template<class T>
T* Singleton<T>::getInstance()
{
        if (instance_ == NULL)
                instance_ = new T;
        return instance_;
}
}

#endif /* SINGLETON_H_ */
