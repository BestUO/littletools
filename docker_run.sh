docker run -itd   -p 9987:9987    -v /home/TRIMULE:/home/TRIMULE   --name trimule trimule:v1.0-bate  sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash '  
