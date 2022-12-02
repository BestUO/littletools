docker run -itd -p 9988:9988 -v $(pwd):/home/Dialogmanager --name dialogmanager --privileged dialogmanager sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash '  
