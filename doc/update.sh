#!/bin/bash
# Updates the whole sphinx project after modifications.

cd /home/hao/Documents/autoCarProject2018/docs/doc/

sphinx-apidoc -f -o ./rst/ ./scripts/

sphinx-build -b html ./rst/ ./html/

sudo rm -rf /var/www/html/test/*

sudo cp -r ./_build/html/* /var/www/html/test/

sudo chown -R www-data:www-data /var/www/
