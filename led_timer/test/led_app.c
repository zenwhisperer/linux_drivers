#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>


int main(int argc, char** argv)
{
    char *filename = argv[1];
    int ret = 0;
    int fd = open(filename, 0, O_RDWR);
    if (fd < 0) {
        printf("open file failed\r\n");
        return ret;
    }

    char *writebuf;
    writebuf = argv[2];
    printf("data to write: %s\r\n", writebuf);
    ret = write(fd, writebuf, 1);
    if (ret < 0) {
        printf("write file failed\r\n");
        return ret;
    }

    ret = close(fd);
    if (ret < 0) {
        printf("close file failed\r\n");
        return ret;
    }


    return 0;
}