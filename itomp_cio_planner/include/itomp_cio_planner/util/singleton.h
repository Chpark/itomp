/*
 * Singleton.h
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

namespace itomp_cio_planner
{

template<class T>
class Singleton
{
public:
	virtual ~Singleton(void) {}
	static T* getInstance();
    static void destroy();

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

template<class T>
void Singleton<T>::destroy()
{
    delete instance_;
    instance_ = NULL;
}

}

#endif /* SINGLETON_H_ */
