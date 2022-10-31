
docker run -itd -p 9987:9987 -v $(pwd):/home/Trimule --name trimule trimule:v1.0 sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash' 