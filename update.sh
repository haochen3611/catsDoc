#!/bin/bash
# Updates the whole sphinx project after modifications.

cd /home/hao/Documents/autoCarProject2018/docs/

cd ./rst/

rm -rf *

cp ../conf.py .

mkdir _static

cd ..

sphinx-apidoc -f -o ./rst/ ./scripts/

cp index.rst ./rst/

# rm -rf ./html/*

sphinx-build -b html ./rst/ ./html/

cp index.rst ./html/

sudo rm -rf /var/www/html/test/*

sudo cp -r ./html/* /var/www/html/test/

sudo chown -R www-data:www-data /var/www/
