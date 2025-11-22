#include "app_storage.h"
#include "app_config.h"


#include "FreeRTOS.h"
#include "task.h"
#include "bsp_spi.h"
#include "stdlib.h"

#include "lfs.h"
#include "lfs_util.h"
#include "bsp_tx25f64b.h"

// -------------------------------
// memory hardware
// -------------------------------
#define MEM_BYTES_PER_PAGE          (256)
#define MEM_PAGES_PER_BLOCK         (16)
#define MEM_BLOCK_SIZE              (MEM_BYTES_PER_PAGE * MEM_PAGES_PER_BLOCK)
#define MEM_BLOCK_COUNT             (2048)  // 24 blocks reserve to bad block manage  
#define MEM_FS_BUFFER_LENGTH        256


// variables used by the filesystem
static lfs_t lfs;
static lfs_file_t file;
static lfs_dir_t dir;

fs_info_t fs_info;
file_info_t file_info;
struct lfs_info file_info_temp;


extern char file_name_update[40];

static unsigned char lfs_read_buff[256];
static unsigned char lfs_write_buff[256];
static unsigned int lfs_head_buff[32];

int user_provided_block_device_read(const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size)
{
    LFS_ASSERT(block < c->block_count);
    unsigned int addr= (block * MEM_PAGES_PER_BLOCK)*MEM_BYTES_PER_PAGE + off;
    bsp_W25QXX_Read((uint8_t *)buffer,addr,size);
    return 0;
}

int user_provided_block_device_prog(const struct lfs_config *c, lfs_block_t block,lfs_off_t off, const void *buffer, lfs_size_t size)
{
    LFS_ASSERT(block < c->block_count);
    unsigned int addr= (block * MEM_PAGES_PER_BLOCK)*MEM_BYTES_PER_PAGE + off;
    bsp_W25QXX_Write_Page((uint8_t *)buffer,addr,size);
    return 0;
}

int user_provided_block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
    LFS_ASSERT(block < c->block_count);
    bsp_W25QXX_Erase_Sector(block);
    return 0;
}

int user_provided_block_device_sync(const struct lfs_config *c)
{
    return 0;
}
// configuration of the filesystem is provided by this struct
const struct lfs_config cfg =
{
    // block device operations
    .read = user_provided_block_device_read,
    .prog = user_provided_block_device_prog,
    .erase = user_provided_block_device_erase,
    .sync = user_provided_block_device_sync,

    // block device configuration
    .read_size = MEM_FS_BUFFER_LENGTH,
    .prog_size = MEM_FS_BUFFER_LENGTH,
    .block_size = MEM_BLOCK_SIZE,
    .block_count = MEM_BLOCK_COUNT,
    .cache_size = MEM_FS_BUFFER_LENGTH,
    .lookahead_size = MEM_BLOCK_COUNT / 16,
    .block_cycles = 500,

    .read_buffer= lfs_read_buff,
    .prog_buffer=lfs_write_buff,
    .lookahead_buffer=	lfs_head_buff,



    // Optional upper limit on files in bytes. No downside for larger files
    // but must be <= LFS_FILE_MAX. Defaults to LFS_FILE_MAX when zero. Stored
    // in superblock and must be respected by other littlefs drivers.
    .file_max=1024*1024,



};




//pram in :
// return counts of fs free space current
int fs_space_reamin(void)
{
    return fs_info.fs_space_remain;
}
// sub 4k byte every time save 4k data to fs
void fs_space_flush(void)
{
    fs_info.fs_space_remain -= 4;
    fs_info.fs_space_used += 4;
}

char fs_file_status_check(void)
{
    return fs_info.fs_file_status;
}
int fs_file_open(char *path)
{
    int  err=1;
    if (fs_info.fs_status == 0)
        return 0;
    if( fs_info.fs_file_status==0)
    {
        err= lfs_file_open(&lfs, &file, path, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
        if(err==0)
        {
            fs_info.fs_file_status=1;
        }
    }
    return err;
}
int fs_current_file_close(void)
{
    int  err=0;
    if (fs_info.fs_status == 0)
        return 0;
    if(fs_info.fs_file_status==1)
    {
        err= lfs_file_close(&lfs, &file);
        if(err==0)
        {
            fs_info.fs_file_status=0;
        }
    }
    return err;
}

int fs_current_file_seek(int off )
{
    if (fs_info.fs_status == 0)
        return 0;
    int  pos= lfs_file_seek(&lfs, &file,off,LFS_SEEK_SET);
    return pos;
}





int fs_current_file_write(void *buffer, int size)
{
    if (lfs_file_write(&lfs, &file, buffer, size))
        return size;
}



int fs_current_file_read(void *buffer, int size)
{
    return lfs_file_read(&lfs, &file, buffer, size);
}

int fs_current_file_sync(void)
{   int  code;
    code=lfs_file_sync(&lfs, &file);
    return code;
}
int fs_current_file_size(void)
{
    return lfs_file_size(&lfs, &file);
}

int open_root_dir(void)
{
    int  code = lfs_dir_open(&lfs, &dir, "/"); // scan root dir
    code = lfs_dir_rewind(&lfs, &dir);
    file_info.index=0;
    return code;
}
int fs_file_delete(char *path)
{
    int err=lfs_remove(&lfs, path);
    if (err==0)
        fs_info.fs_file_counts -= 1;
    return err;
}
int close_root_dir(void)
{
    file_info.index=0xFF;
    return lfs_dir_close(&lfs, &dir);
}

int file_info_scan_next(void)
{
    int    code = lfs_dir_read(&lfs, &dir, &file_info_temp);
    if(code>LFS_ERR_OK)
    {
        if (file_info_temp.type == LFS_TYPE_REG)
        {
            if (file_info_temp.size == 0)
            {
                lfs_remove(&lfs, file_info_temp.name);
                file_info.name=NULL;
                return 2;
            }
            else
            {
                // code = lfs_dir_close(&lfs, &dir);
                file_info.name=file_info_temp.name;
                file_info.size=file_info_temp.size;
                file_info.index++;
                return 1;
            }
        }
        else if (file_info_temp.type == LFS_TYPE_DIR)
        {
            file_info.name=NULL;
            return 3;
        }
        else
        {
            file_info.name=NULL;
            return 4;
        }
    }
    else
    {
        return code;
    }
}

char fs_all_file_scan(void)
{
    char temp = 0;
    struct lfs_info file_info_temp;
    int  code = lfs_dir_open(&lfs, &dir, "/"); // scan root dir
    code = lfs_dir_rewind(&lfs, &dir);
    do
    {
        code = lfs_dir_read(&lfs, &dir, &file_info_temp);
        if (file_info_temp.type == LFS_TYPE_REG)
        {
            if (file_info_temp.size == 0)
                lfs_remove(&lfs, file_info_temp.name);
            else
                temp++;
        }
        else if (file_info_temp.type == LFS_TYPE_DIR)
        {
        }
    }
    while (LFS_ERR_OK != code);
    code = lfs_dir_close(&lfs, &dir);
    return temp;
}

void fs_all_file_delete(void)
{
    int code = lfs_dir_open(&lfs, &dir, "/");
    if( !fs_info.fs_file_counts)
        return ;
    code = fs_current_file_close();
    code = lfs_dir_rewind(&lfs, &dir);
    do
    {
        code = lfs_dir_read(&lfs, &dir, &file_info_temp);
        if (file_info.type == LFS_TYPE_REG)
        {
            lfs_remove(&lfs, file_info.name);
        }
    }
    while (LFS_ERR_OK != code);
    fs_info.fs_file_counts = 0;
    fs_info.fs_space_used   = lfs_fs_size(&lfs) * MEM_BLOCK_SIZE / 1024;
    fs_info.fs_space_remain = MEM_BLOCK_COUNT * MEM_BLOCK_SIZE / 1024 - fs_info.fs_space_used;
}

int file_exname_check(char *file_name,char *exname)
{
    int index=0;
    int file_len=strlen(file_name);
    int exname_len=strlen(exname);
    for(int i=file_len; i>0; i--)
    {
        if(file_name[i]==0x2E)
        {
            index=i+1;
            break;
        }
    }
    if(memcmp(&file_name[index],exname,exname_len)==0)
        return 1;
    else return 0;
}

static char fs_init(void)
{
    if (fs_info.fs_status) //fs was mounted
        return 0;
    // mount the filesystem
    uint32_t err = lfs_mount(&lfs, &cfg);
//    // reformat if we can't mount the filesystem
//    // this should only happen on the first boot
    if (err)
    {
        err=lfs_format(&lfs, &cfg);
        err=lfs_mount(&lfs, &cfg);
    }
    fs_info.fs_file_counts = fs_all_file_scan();
    fs_info.fs_status = 1;
    fs_info.fs_space_used   = lfs_fs_size(&lfs) * MEM_BLOCK_SIZE / 1024;
    fs_info.fs_space_remain = MEM_BLOCK_COUNT * MEM_BLOCK_SIZE / 1024 - fs_info.fs_space_used;
    //lfs_error_l code = lfs_dir_open(&lfs, &dir, "/");
    return 0;
}
static void fs_uninit(void)
{
    if (!fs_info.fs_status) //fs was unmounted
        return;
    fs_current_file_close();
    lfs_unmount(&lfs);
    fs_info.fs_status = 0;
}




int fs_file_create(char *path)
{
    return fs_file_open(path);
}
void app_storage_enable(void)
{
    fs_init();
}
void app_storage_disable(void)
{
    fs_uninit();
}

void app_storage_init(void)
{
    bsp_W25Q256_init();
    app_storage_enable();
}

