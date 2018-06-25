#!/bin/bash
# Updates the whole sphinx project after modifications.

cd /home/hao/Documents/autoCarProject2018/docs/doc/

#sphinx-apidoc -f -o ./rst/ ./scripts/

sphinx-apidoc -f -o . ./scripts/

make html

#cp index.rst ./rst/

#cp conf.py ./rst/

#cp core.rst ./rst/

#cp modules.rst ./rst/

#cp ./modified_rst/* ./rst/

#sphinx-build -b html ./rst/ ./_build/html/

sudo rm -rf /var/www/html/test/*

sudo cp -r ./_build/html/* /var/www/html/test/

sudo chown -R www-data:www-data /var/www/

cp -r ./_build/html/* /home/hao/Documents/autoCarProject2018/html/
