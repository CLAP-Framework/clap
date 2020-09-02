/**
 * @file shm_object.h
 * @brief Semaphore in processes. The first INFOLEN number of  integers are for message information.
 * Reading process should be open after writing process!
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-13
 */
#ifndef RSLIDAR_SHMOBJECT_H
#define RSLIDAR_SHMOBJECT_H

#include<stdio.h>
#include<iostream>
#include<unistd.h>
#include<stdlib.h>
#include<sys/types.h>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<string.h>
#include <ros/ros.h>

#define INFO_LEN 10           //information length: int number before data block
#define MAX_CLIENT_NUM 20    //max number of receive node can be set
namespace shared_memory{
class ShmObject
{
public:
    int shmid_;                   //共享内存ID
    int client_num_;        //SHM接收端的个数
    void *p_map_;           //共享内存首地址

    unsigned int *pp_;   //数据区首地址

    unsigned int *blksize_;//块大小
    unsigned int *blocks_;//总块数
    unsigned int *rd_index_;//读索引
    unsigned int *wr_index_;//写索引
    unsigned int *quit_;//退出标志位
    unsigned int *exist_;//内存存在标志位

    unsigned char *databegin_;//数据区开始位置
    unsigned char **shmdata_;//共享内存数据块

    ShmObject(){}
    ShmObject(int datalen,int datanum,int shmkey, int client_num = 1);//写进程构造函数          
    ShmObject(int shmkey);//读进程构造函数
    ShmObject& operator = (const ShmObject& a){
        this->shmid_ = a.shmid_;
        this->client_num_ = a.client_num_;        
        this->p_map_ = a.p_map_;
        this->pp_ = a.pp_;
        this->blksize_ = a.blksize_;
        this->blocks_ = a.blocks_;
        this->rd_index_ = a.rd_index_;
        this->wr_index_ = a.wr_index_;
        this->quit_ = a.quit_;
        this->exist_ = a.exist_;
        this->databegin_ = a.databegin_;
        this->shmdata_ = a.shmdata_;
    }

    void write_release();//写数据对象释放
    void read_release();//读数据对象释放a

    unsigned char* require_data();//内存要求函数，成功则返回可写内存块指针
    void update_shm(unsigned char *getdata);//数据更新，等待被读程序读入

    unsigned char* get_data_for_read();//内存要求，成功则返回可读内存指针
    unsigned char* get_data_for_read_with_sem(const int shm_index);
}; 
}   // end of shared_memory
#endif
