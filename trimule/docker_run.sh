docker run -itd -p 9987:9987 -v $(pwd):/home/Trimule --name trimule --privileged trimule sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash '  
