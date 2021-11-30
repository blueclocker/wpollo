/*
 * @Author: your name
 * @Date: 2021-11-16 13:53:37
 * @LastEditTime: 2021-11-18 16:56:47
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/include/test/subdeque.h
 */
#ifndef SUBDEQUEUE_H_
#define SUBDEQUEUE_H_  

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp> 
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

template <typename T>
struct MessageQueue
{
    T data;
    MessageQueue<T> *next;
    MessageQueue<T> *prev;
};


template <typename T>
class TopicStack
{
private:
    std::string TopicName;
    int QueueLength;
    int QueueSize;
    MessageQueue<T> *head;
    MessageQueue<T> *end;
    ros::Subscriber sub;
public:
    bool init(std::string TopicName_, int QueueLength_ = 5);
    bool empty();
    void push_back(const T pushin);
    void pop_front();
    void pop(const int index);
    T* find(const int index);
    void subscribing(ros::NodeHandle nh_);
    int size();
    void show(const int index);
    void callback(const boost::shared_ptr<const T> &msg);
    TopicStack();
    virtual ~TopicStack();
};

template<typename T>
TopicStack<T>::TopicStack()
{
    head = new MessageQueue<T>;
    end = new MessageQueue<T>;
    head->next = end;
    head->prev = nullptr;
    end->prev = head;
    end->next = nullptr;
    QueueSize = 0;
}

template<typename T>
TopicStack<T>::~TopicStack()
{
    //清空消息队列
    if(QueueSize == 0)
    {
        delete head;
        delete end;
        head = nullptr;
        end = nullptr;
        return;
    }else{
        while (head->next != nullptr)
        {
            MessageQueue<T> *temp = head;
            head = head->next;
            delete temp;
        }
        delete head;
        head = nullptr;
    }
    //std::cout << "~topicstack" << std::endl;
}

template<typename T>
bool TopicStack<T>::init(std::string TopicName_, int QueueLength_)
{
    if(TopicName_.empty()) return false;
    TopicName = TopicName_;
    QueueLength = QueueLength_;
    std::cout << TopicName << "\t" << QueueLength << std::endl;
    return true;
}

template<typename T>
bool TopicStack<T>::empty()
{
    if(QueueSize == 0)
    {
        return true;
    }else{
        return false;
    }
}

template<typename T>
void TopicStack<T>::push_back(const T pushin)
{
    MessageQueue<T> *newnode = new MessageQueue<T>;
    end->data = pushin;
    end->next = newnode;
    newnode->prev = end;
    newnode->next = nullptr;
    end = end->next;
    QueueSize++;
}

template<typename T>
void TopicStack<T>::pop_front()
{
    if(QueueSize == 0)
    {
        std::cout << "queue is empty" << std::endl;
        return;
    }
    MessageQueue<T> *node = head->next;
    head->next = node->next;
    node->next->prev = head;
    node = nullptr;
    delete node;
    QueueSize--;
}

template<typename T>
void TopicStack<T>::pop(const int index)
{
    if(index >= QueueSize || index < 0)
    {
        std::cout << "out of index error" << std::endl;
        return;
    }
    MessageQueue<T> *node = head;
    for(int i = 0; i <= index; ++i)
    {
        node = node->next;
    }
    node->prev->next = node->next;
    node->next->prev = node->prev;
    node = nullptr;
    delete node;
    QueueSize--;
}

template<typename T>
int TopicStack<T>::size()
{
    return QueueSize;
}

template<typename T>
T* TopicStack<T>::find(const int index)
{
    if(index >= QueueSize || index < 0)
    {
        std::cout << "out of index error" << std::endl;
        return nullptr;
    }
    MessageQueue<T> *node = head;
    for(int i = 0; i <= index; ++i)
    {
        node = node->next;
    }
    return &(node->data);
}

template<typename T>
void TopicStack<T>::show(const int index)
{
    if(index >= QueueSize || index < 0)
    {
        std::cout << "out of index error" << std::endl;
        return;
    }
    MessageQueue<T> *node = head;
    for(int i = 0; i <= index; ++i)
    {
        node = node->next;
    }
    std::cout << node->data;
}

template<typename T>
void TopicStack<T>::callback(const boost::shared_ptr<const T> &msg)
{
    push_back(*msg);
    ROS_INFO("callback = %d", size());
    if(QueueSize > QueueLength) pop_front();
}

template<typename T>
void TopicStack<T>::subscribing(ros::NodeHandle nh_)
{
    sub = nh_.subscribe(TopicName, QueueLength, &TopicStack<T>::callback, this);
    ros::spin();
}




#endif