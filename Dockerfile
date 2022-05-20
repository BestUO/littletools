FROM gcc:11.3.0
MAINTAINER Parsifal

RUN apt-get update && apt-get install supervisor -y  \
    &&  apt-get install vim -y && mkdir /home/TRIMULE


RUN echo '[program:TRIMULE] \n\
         command=nohup ./trimule > /dev/null 2>&1 &  \n\
         user=root   \n\
         autostart=true \n\
         autorestart=true  \n\
        directory=/home/TRIMULE '>> /etc/supervisor/supervisord.conf   

