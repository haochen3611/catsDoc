#!/bin/bash
# Updates the whole sphinx project after modifications.

cd /home/hao/Documents/autoCarProject2018/docs/

cd ./rst/

cp ../conf.py .

cp ../index.rst .

cd ..

sphinx-apidoc -f -o ./rst/ ./scripts/

cp index.rst ./rst/

# rm -rf ./html/*

sphinx-build -b html ./rst/ ./html/

sudo rm -rf /var/www/html/test/*

sudo cp -r ./html/* /var/www/html/test/

sudo chown -R www-data:www-data /var/www/
