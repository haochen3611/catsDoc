#!/bin/bash
# Updates the whole sphinx project after modifications.

cd /home/hao/Documents/autoCarProject2018/docs/doc/

#sphinx-apidoc -f -o ./rst/ ./scripts/

sphinx-apidoc -f -o . ./scripts/

make html

#sphinx-build -b html . ./_build/html/

sudo rm -rf /var/www/html/test/*

sudo cp -r ./_build/html/* /var/www/html/test/

sudo chown -R www-data:www-data /var/www/
