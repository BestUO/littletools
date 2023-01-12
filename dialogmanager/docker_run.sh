DialogManagerPath="/home/dialogmanager"
DataPathFrom="/var/java/fs/upload/"
DataPathTO=$DialogManagerPath/data
docker run -itd -p 9988:9988 -v $(pwd):$DialogManagerPath -v $DataPathFrom:$DataPathTO --name dialogmanager --privileged dialogmanager sh -c  'supervisord -c /etc/supervisor/supervisord.conf && /bin/bash '  