# ch34x_linux
USB to serial driver for USB to serial chip ch340, ch341, etc.

1. BUILDING
 $ sudo make
 
2. LOAD
 $ sudo make load
 or you can use
 $ sudo insmod ch34x.ko
 
 3. UNLOAD
 $ sudo make unload
 or you can use
 $ sudo rmmod ch34x.ko
 
4. AUTOLOAD SINCE BOOT
  $ sudo make install
  
5. CANCEL AUTOLOAD SINCE BOOT
  $ sudo make uninstall
  
Note
  If you wanna look up more details, please open debug switch(DEBUG && VERBOSE_DEBUG) in ch34x.c
  Any question, you can send feedback to mail: tech@wch.cn
