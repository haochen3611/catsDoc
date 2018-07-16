Install CATS on Ubuntu
======================

These instructions will get you a copy of the project up and running on
your local machine for development and testing purposes. See deployment
for notes on how to deploy the project on a live system.

Prerequisites
~~~~~~~~~~~~~

You will need a computer or virtual machine with Linux operation system
installed. Right now the installer only supports Debian, Ubuntu, and
Linux Mint. Make sure you have properly installed **Python 3.5+**.

You can check your Python version by typing the following command in the
terminal.

::

   python -V

If you do not have **Python 3.5+** in your system, then type the
following command in the terminal.

::

   sudo apt-get install python3

Installing
~~~~~~~~~~

-  **First**, download the project repo from `citrus-r-overtake.zip <https://github.com/haochen3611/CATS.git>`_.
   Unzip it into a fold of your choice. Change direction into the
   ``server`` folder in the termianl by using the following command.

   ::

      cd /your/direction/citrus-r-overtake/server

-  **Second**, initial the installer by executing the ``install.sh``.
   Type the following command.

   ::

      sudo bash install.sh

   You will be asked for password to run the command. Make sure you have
   root access

-  **Third**, you can start the program by running the
   ``simulation.py`` in the ``core`` folder. A simple way to do this is
   to enter the following commands in the terminal.

   ::

      cd core
      python3 simulation.py

   Alternatively, you can run ``simulation.py`` in an IDE of your choice
   by loading the project and executing the ``simulation.py``.
