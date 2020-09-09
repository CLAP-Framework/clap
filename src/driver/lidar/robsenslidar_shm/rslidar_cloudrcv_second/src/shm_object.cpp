/**
 * @file shm_object.cpp
 * @brief SHM in processes. The first INFOLEN int are for message information.
 * Reading process should be open after writing process!
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-13
 */

#include"shm_object.h"

unsigned char* ShmObject::get_data_for_read()
{
    unsigned char *readaddr;
    if ((*rd_index_!=*blocks_ - 1) && (shmdata_[*rd_index_][0] == 1))     //未到最后并且当前数据块可读
    {
        readaddr=shmdata_[*rd_index_];
        *rd_index_=*rd_index_+1;
    }   
    else if((*rd_index_==*blocks_ -1) && shmdata_[0][0] == 1)          //到最后一帧，且下一帧（开始帧）可读
    {
        *rd_index_=0;
        readaddr=shmdata_[*rd_index_];      //读取开始帧
        *rd_index_=*rd_index_+1;
    }
    else
    {
        return NULL;
    }
    return readaddr;
}
 
unsigned char* ShmObject::get_data_for_read_with_sem(const int clent_index)
{
    unsigned char *readaddr;
    if ((*wr_index_ > 0) && (shmdata_[*wr_index_ - 1][clent_index-1] == 1))     //未到最后并且当前数据块可读
    {
        readaddr=shmdata_[*wr_index_ - 1];
    }
    else if ((*wr_index_== 0) && (shmdata_[*blocks_ - 1][clent_index-1] == 1))     //已到最后并且当前数据块可读
    {
        readaddr=shmdata_[*blocks_ - 1];
    }
    else
    {
        return NULL;
    }
    return readaddr;
}

// 读成员构造函数
ShmObject::ShmObject(int shmkey)
{
    // 生成一个key
    key_t key = ftok("/temp", shmkey);
    // 获取共享内存，返回一个id
    for(int i = 0; i < 3; i++){
        shmid_ = shmget(key, 0, 0);
        printf("[SHM]READ key, shmkey, shmid_ is %u, %d, %d.\n", (uint32_t)key, shmkey, shmid_);
        if(shmid_ == -1) {
            usleep(300000);
            continue;
        }            
        else
            break;
    }       //end of for
    if(shmid_ == -1){
        perror("shmget failed");
        exit(1);
    }
    // 映射共享内存，得到虚拟地址
    p_map_ = shmat(shmid_, 0, 0);
    if((void*)-1 == p_map_)
    {
        perror("shmat failed");
        exit(2);
    }
    // 读共享内存
    pp_ = (unsigned int *)p_map_;
    blksize_=pp_;//块大小
    blocks_=pp_+1;//总块数
    rd_index_=pp_+2;//读索引
    wr_index_=pp_+3;//写索引
    quit_=pp_+4;//退出标志位
    exist_=pp_+5;//内存存在标志位
    printf("[SHM] block size %d\n", *blksize_);
    printf("[SHM] block number %d\n", *blocks_);
    printf("[SHM] read index %d\n", *rd_index_);
    printf("[SHM] write index %d\n", *wr_index_);
    printf("[SHM] quit flag %d\n", *quit_);
    printf("[SHM] SHM exist flag %d\n", *exist_);
    databegin_=(unsigned char *)(pp_+INFO_LEN);//数据区开始位置
    shmdata_=new unsigned char*[*blocks_];
    for(int i=0;i<*blocks_;i++)
    {
        shmdata_[i]=databegin_+i*(*blksize_);
    }
}
 
// 更新写索引
void ShmObject::update_shm(unsigned char *getdata)
{
    for(int i=0; i < client_num_; i++){
        getdata[i] = 1;
    }
    if(*wr_index_ < *blocks_ - 1)
        *wr_index_=*wr_index_+1;        //更新写入索引
    else    
        *wr_index_=0;    
}

// 内存要求函数，成功则返回可写内存块指针
unsigned char* ShmObject::require_data()
{
    if(*quit_==1)//退出标志位
    {
        printf("[SHM] get SHM remove request, no longer write and will remove.\n");
        return NULL;
    }
    return shmdata_[*wr_index_];
}
 
// 写成员构造函数
ShmObject::ShmObject(int datalen,int datanum,int shmkey, int client_num) : client_num_(client_num)
{
    if(client_num_ > MAX_CLIENT_NUM)                        //client number is out of range
    {
        perror("[SHM]client number is out of range!");
        exit(1);
    }
    // 生成一个key
    key_t key = ftok("/temp", shmkey);
    // 创建共享内存，返回一个id
    // 第二个参数为共享内存大小，前面四个值分别记录共享内存循环队列的块大小，总块数，写指针索引,读指针索引与退出标志
    while(true)
    {
        shmid_ = shmget(key, sizeof(int)*INFO_LEN+(datalen + MAX_CLIENT_NUM)*datanum, IPC_CREAT|0666); //0666是文件权限，不写只有超级用户才能打开
        printf("[SHM]WRITE key, shmkey, shmid_ is %u, %d, %d.\n", (uint32_t)key, shmkey, shmid_);
        if(-1 == shmid_)                        //4 * 10 + length * num          字节为单位  datalen字节数，datanum数据个数
        {
            perror("[SHM]shmget failed");
            exit(1);
        }
        // 映射共享内存，得到虚拟地址
        p_map_ = shmat(shmid_, 0, 0);                 
        //void *p_map_ = (people*)shmat(shm_id,NULL,0);    TODO:add struct.!!!!!!!!!!!!!!!!!!!!!!
        if((void*)-1 == p_map_)
        {
            perror("[SHM]shmat failed");
            exit(2);
        }
        shmdata_=new unsigned char*[datanum];       //指针数据 8
        // 共享内存对象映射
        pp_ = (unsigned int *)p_map_;           //共享内存区首地址

        blksize_=pp_;                //块大小
        blocks_=pp_+1;            //总块数
        rd_index_=pp_+2;       //读索引
        wr_index_=pp_+3;      //写索引
        quit_=pp_+4;                //退出标志位
        exist_=pp_+5;               //内存存在标志位
 
        if (*exist_==99)        //原先内存没有销毁，先销毁掉再创建
        {
            printf("[SHM] old SHM not removed, will remove and rebuild.\n");
            if(-1 == shmctl(shmid_, IPC_RMID, NULL))
            {
                perror("shmctl failed");
                exit(4);
            }
        }
        else
        {
            break;              //不存在就跳出
        }
    }
 
    databegin_=(unsigned char *)(pp_+INFO_LEN);//数据区开始位置   uint_32 * pp_        (p+40)更好？
    for(int i=0;i<datanum;i++)
    {
        shmdata_[i]=databegin_+i*(datalen + MAX_CLIENT_NUM);               //一帧一帧拷贝首地址  
    }
    // 共享内存对象初始化
    *blksize_=(datalen + MAX_CLIENT_NUM);          //字节数 block size equal to: datalen + max_client_num(for flags)
    *blocks_=datanum;       //数量：帧数
    *rd_index_=0;
    *wr_index_=0;
    *quit_=0;
    *exist_=99;//内存存在标志位
    printf("[SHM] initialization finished, SHM built !\n");
    printf("[SHM] block size %d\n", *blksize_);
    printf("[SHM] block number %d\n", *blocks_);
    printf("[SHM] read index %d\n", *rd_index_);
    printf("[SHM] write index %d\n", *wr_index_);
    printf("[SHM] quit flag %d\n", *quit_);
}
 
//释放共享内存对象
void ShmObject::write_release()
{
    // 解除映射(断开共享内存连接)
    if(-1 == shmdt(p_map_))          
    {
        perror("shmdt failed");
        exit(3);
    }
    // 销毁共享内存
    printf("[SHM] release mapping %d\n",shmid_);
    if(-1 == shmctl(shmid_, IPC_RMID, NULL))                //销毁共享内存
    {
        perror("shmctl failed");
        exit(4);
    }               
    // 销毁开辟内存
    delete []shmdata_;
}
 
//解除共享内存映射
void ShmObject::read_release()
{
    *quit_=1;//设置退出标志位
    // 解除映射
    if(-1 == shmdt(p_map_))
    {
        perror("shmdt failed");
        exit(3);
    }
    // 销毁开辟内存
    delete []shmdata_;
}
