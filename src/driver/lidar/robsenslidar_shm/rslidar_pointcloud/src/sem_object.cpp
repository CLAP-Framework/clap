/**
 * @file sems_object.cpp
 * @brief Semaphore for communication in processes. 
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-18
 */

#include"sem_object.h"

namespace shared_memory{

//int semget(key_t key, int nsems, int semflg);

int SemObject::CommSemid(int proj_id, int nums, int flags)
{
    key_t _key = ftok("./", proj_id);
    ROS_INFO_STREAM("[SEM] Key_t value is " << _key);
    if (_key>0)
    {   
        // ROS_INFO_STREAM("[SEM_TEST] _key is " << _key << ", and nums is "
        //     << nums << ", and flags is " << flags);
        int ret = semget(_key, nums, flags);
        ROS_INFO_STREAM("[SEM] semget return value(SEM_ID) is " << ret);
        return ret;       //根据_key值创建/获得信号集
    }
    else
    {
        perror("CommSemid");
        return -1;
    }
}

int SemObject::CreatSemid(int proj_id, int nums)
{
    // return CommSemid(proj_id, nums, IPC_CREAT | IPC_EXCL | 0666);
    return CommSemid(proj_id, nums, IPC_CREAT | 0666);
    //IPC_CREAT | IPC_EXCL则可以创建一个新的，唯一的信号量，
    //如果信号量已存在，返回一个错误。一般我们会还或上一个文件权限

}

// int semctl(int semid, int semnum, int cmd, ...);
int SemObject::GetSemid(int proj_id, int nums)       
{
        int ret;
        for(int i = 0; i < 3; i++){
            ret = CommSemid(proj_id, nums, IPC_CREAT);
            if(ret == -1) {
                usleep(300000);
                continue;
            }            
            else
                break;
        }
        return ret;          
    //设置了IPC_CREAT标志后，即使给出
    //的key是一个已有信号量的key，也不会产生错误  
}

bool SemObject::IfSemExist(int proj_id)
{
    key_t _key = ftok("./", proj_id);
    if (_key>0)
    {   
        int ret = semget(_key, 0, 0);
        if(ret >= 0)
            return true;
        else
            return false;       
    }
    else
    {
        perror("CommSemid");
        return false;
    }
}


int SemObject::Destory(int semid)
{
    printf("[SEM]sem destroyed entered already!");
    if (semctl(semid, 0, IPC_RMID)>0)       //IPC_RMID:删除信号量标识符
    {
        printf("[SEM]sem destroyed and ID is : d%" , semid);
        return 0;
    }
    else
    {
        perror("Destory");
        return -1;
    }
}

int SemObject::InitSem(int semid, int which, int _val)
{

    union semun _semun;
    _semun.val = _val;
    if (semctl(semid, which, SETVAL, _semun)<0)     //SETVAL：用来把信号量初始化为一个已知的值。
                                                                                                     // p 这个值通过union semun中的val成员设置，其作用是在信号量第一次使用前对它进行设置。
    {
        perror("InitSem");
        return -1;
    }
    return 0;
}

int SemObject::SemPV(int semid, int which, int _op)
{
    struct sembuf _sf;
    _sf.sem_num = which;
    _sf.sem_op = _op;               //-1 || 1               信号值-1或+1
    _sf.sem_flg = 0;    
    return semop(semid, &_sf, 1);
    // ROS_WARN_STREAM("[SEM_TEST] _sf-which is " << which
    //     << ", and _op is " << _op << ", and semid is "<<semid);
}

int SemObject::P(int semid, int which, int _op)     //P(semid, 0, -1);
{
    //ROS_INFO("[SEM] P waiting process.");
    if (SemPV(semid, which, _op)<0)
    {
        ROS_ERROR_STREAM("P error " << SemPV(semid, which, _op));
        return -1;
    }
    int signal_num = semctl(semid, 0, GETVAL);
    return 0;
}

int SemObject::V(int semid, int which, int _op)     //V(semid, 0, 1);
{
    //ROS_INFO("[SEM] V sending process.");
    if (SemPV(semid, which, _op)<0)
    {
        ROS_ERROR_STREAM("V error " << SemPV(semid, which, _op));
        return -1;
    }
    int signal_num = semctl(semid, 0, GETVAL);
    return 0;
}
}   //end of namespace shared_memory
