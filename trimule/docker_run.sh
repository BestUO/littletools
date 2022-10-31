docker run -itd -p 9987:9987 -v $(pwd):/home/Trimule --name newtrimule --privileged newtrimule sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash '  
