FROM gcc:11.3.0
MAINTAINER trimule

RUN  sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list \
     && sed -i s@/deb.debian.org/@/mirrors.aliyun.com/@g /etc/apt/sources.list \
     &&  apt-get clean && apt-get update  --fix-missing -o Acquire::http::No-Cache=True && apt-get install supervisor -y  \
     && apt-get install vim -y \
     && /bin/cp /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
     && echo 'Asia/Shanghai' >/etc/timezone  


RUN echo '[program:Dialogmanager] \n\
          command=sh run_dialogmanager.sh  \n\
          user=root   \n\
          autostart=true \n\
          autorestart=true  \n\
          directory=/home/Dialogmanager '>> /etc/supervisor/supervisord.conf   
