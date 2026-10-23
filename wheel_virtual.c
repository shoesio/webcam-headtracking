// Separate analogue pad: mirror the existing Steam pad, replace only steering.
#include <linux/uinput.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>

static void emit(int fd, int type, int code, int value) {
    struct input_event e = {.type=type, .code=code, .value=value};
    if (write(fd, &e, sizeof(e)) != sizeof(e)) { perror("input write"); exit(2); }
}
int main(int argc, char **argv) {
    int fd=open("/dev/uinput",O_WRONLY|O_NONBLOCK);
    if(fd<0){perror("uinput");return 1;}
    ioctl(fd,UI_SET_EVBIT,EV_KEY);ioctl(fd,UI_SET_EVBIT,EV_ABS);
    int keys[]={BTN_SOUTH,BTN_EAST,BTN_NORTH,BTN_WEST,BTN_TL,BTN_TR,
                BTN_SELECT,BTN_START,BTN_MODE,BTN_THUMBL,BTN_THUMBR};
    for(unsigned i=0;i<sizeof(keys)/sizeof(*keys);i++)ioctl(fd,UI_SET_KEYBIT,keys[i]);
    struct uinput_user_dev dev={0};
    snprintf(dev.name,sizeof(dev.name),"MKVR Gesture Wheel");
    dev.id.bustype=BUS_USB;dev.id.vendor=0x045e;dev.id.product=0x028e;dev.id.version=1;
    int axes[]={ABS_X,ABS_Y,ABS_RX,ABS_RY,ABS_Z,ABS_RZ,ABS_HAT0X,ABS_HAT0Y};
    for(unsigned i=0;i<sizeof(axes)/sizeof(*axes);i++){
        int a=axes[i];ioctl(fd,UI_SET_ABSBIT,a);
        dev.absmin[a]=(a==ABS_Z||a==ABS_RZ)?0:(a==ABS_HAT0X||a==ABS_HAT0Y)?-1:-32768;
        dev.absmax[a]=(a==ABS_Z||a==ABS_RZ)?255:(a==ABS_HAT0X||a==ABS_HAT0Y)?1:32767;
    }
    if(write(fd,&dev,sizeof(dev))!=sizeof(dev)||ioctl(fd,UI_DEV_CREATE)<0){perror("create");return 1;}
    char name[128]={0};ioctl(fd,UI_GET_SYSNAME(sizeof(name)),name);
    printf("READY %s\n",name);fflush(stdout);
    int source=-1;
    if(argc>1 && strcmp(argv[1],"--probe")){
        source=open(argv[1],O_RDONLY|O_NONBLOCK);
        if(source<0){perror("source pad");ioctl(fd,UI_DEV_DESTROY);return 1;}
        // Seed held controls, including acceleration, before processing changes.
        for(unsigned i=0;i<sizeof(axes)/sizeof(*axes);i++){
            struct input_absinfo a;
            if(ioctl(source,EVIOCGABS(axes[i]),&a)==0)emit(fd,EV_ABS,axes[i],a.value);
        }
        unsigned long bits[(KEY_MAX+8*sizeof(long))/(8*sizeof(long))];memset(bits,0,sizeof(bits));
        if(ioctl(source,EVIOCGKEY(sizeof(bits)),bits)>=0)
            for(unsigned i=0;i<sizeof(keys)/sizeof(*keys);i++)
                emit(fd,EV_KEY,keys[i],(bits[keys[i]/(8*sizeof(long))]>>(keys[i]%(8*sizeof(long))))&1);
        emit(fd,EV_SYN,SYN_REPORT,0);
    }
    int enabled=0,x=0;char buffer[256];size_t used=0;
    struct pollfd fds[2]={{.fd=0,.events=POLLIN},{.fd=source,.events=POLLIN}};
    while(poll(fds,2,100)>=0){
        if(fds[0].revents&(POLLHUP|POLLERR))break;
        if(fds[0].revents&POLLIN){
            char c;ssize_t n=read(0,&c,1);if(n<=0)break;
            if(c=='\n'){
                buffer[used]=0;int en,val;
                if(sscanf(buffer,"%d %d",&en,&val)==2){enabled=!!en;x=val;
                    if(x>32767)x=32767;if(x< -32768)x=-32768;
                    if(enabled)emit(fd,EV_ABS,ABS_X,x);
                    else if(source>=0){struct input_absinfo a;if(ioctl(source,EVIOCGABS(ABS_X),&a)==0)emit(fd,EV_ABS,ABS_X,a.value);}
                    emit(fd,EV_SYN,SYN_REPORT,0);
                }used=0;
            }else if(used<sizeof(buffer)-1)buffer[used++]=c;
        }
        if(source>=0 && fds[1].revents&(POLLHUP|POLLERR))break;
        if(source>=0 && fds[1].revents&POLLIN){
            struct input_event e;
            while(read(source,&e,sizeof(e))==sizeof(e)){
                if(e.type==EV_ABS && e.code==ABS_X && enabled)e.value=x;
                if(e.type==EV_KEY||e.type==EV_ABS||e.type==EV_SYN)emit(fd,e.type,e.code,e.value);
            }
        }
    }
    ioctl(fd,UI_DEV_DESTROY);close(fd);if(source>=0)close(source);return 0;
}
