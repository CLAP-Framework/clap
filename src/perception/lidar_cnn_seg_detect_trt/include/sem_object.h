/**
 * @file sems_object.h
 * @brief Semaphore for communication in processes. 
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-18
 */
#ifndef RSLIDAR_SEM_OBJECT_H
#define RSLIDAR_SEM_OBJECT_H

#include <stdio.h>                        //also in SHM
#include <sys/types.h>              //also in SHM
#include <sys/ipc.h>                   //also in SHM
#include <sys/sem.h>
#include <sys/wait.h>
#include <error.h>
#include<unistd.h>

#include <ros/ros.h>


#define  PATHNAME "./"
//#define  PROJ_ID  100                 //0       //信号集的ID key：相同的可以让不同进程看到同一个信号集

namespace shared_memory{
class SemObject
{
public:
    SemObject(){}
    ~SemObject(){}
    
    //信号量是创建还是获取在于semget函数参数flag的设置
    static int CommSemid(int proj_id, int nums, int flags);
    //创建信号量
    int CreatSemid(int proj_id, int nums);
    //获取已经创建的信号量
    int GetSemid(int proj_id, int nums);
    //检查信号量是否存在
    bool IfSemExist(int proj_id);
    //初始化信号量
    int InitSem(int semid, int which, int _val);
    //PV操作在于它_op的值
    static int SemPV(int semid, int which, int _op);
    //P操作
    int P(int semid, int which, int _op);
    //V操作
    int V(int semid, int which, int _op);
    //由于（System V通信方式）信号量生命周期随内核,所以要销毁信号量
    int Destory(int semid);   
//private:
    union semun
    {
        int val;        //使用的值
        struct semid_ds *buf;       //IPC_STAT、IPC_SET 使用的缓存区
        unsigned short *arry;       //GETALL,、SETALL 使用的数组
        struct seminfo *__buf;     // IPC_INFO(Linux特有) 使用的缓存区
    };
};          //end of class SemObject
}           //end of namespace shared_memory
#endif      //end of RSLIDAR_SEM_OBJECT_H
