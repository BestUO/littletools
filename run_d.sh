tar -zxvf Trimule.tar.gz
rm -rf trimule
cd Trimule
docker stop trimule
cp -f  trimule ../ 
docker start trimule
