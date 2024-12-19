ch341 драйвер последовательного порта Linux

Описание

Драйвер последовательного порта USB для микросхем USB-UART ch340, ch341 и т. д. ch341 поддерживает несколько режимов работы, этот драйвер поддерживает только режим последовательного порта.

Фактически драйвер последовательного порта ch341 был встроен в ядро ​​Linux mainline версии 2.6.24. Расположение: drivers/usb/serial/ch341.c, жаль, что встроенный драйвер не может быть обновлен и не может поддерживать все функции чипа. Мы предлагаем нашим клиентам использовать этот драйвер.

Открыть «Терминал»
Перейти в каталог «драйвер»
Скомпилируйте драйвер с помощью «make», в случае успеха вы увидите модуль «ch341.ko»
1) Введите «sudo make load» или «sudo insmod ch341.ko» для динамической загрузки драйвера.
Введите «sudo make unload» или «sudo rmmod ch341.ko», чтобы выгрузить драйвер.
Введите «sudo make install», чтобы драйвер работал постоянно.
Введите «sudo make uninstall», чтобы удалить драйвер.
Вы можете перейти по ссылке ниже, чтобы получить приложение uart, вы можете использовать gcc или выполнить кросс-компиляцию с помощью cross-gcc https://github.com/WCHSoftGroup/tty_uart
Перед тем, как драйвер заработает, необходимо убедиться, что USB-устройство подключено и работает правильно. Для подтверждения этого можно использовать команду оболочки «lsusb» или «dmesg». USB-VID этих устройств — [1a86]. Все идентификаторы можно просмотреть в таблице идентификаторов, которая определена в «ch341.c».

Если устройство работает нормально, драйвер создаст tty-устройства с именами «ttyCH341USBx» в каталоге /dev.



1. Open "Terminal"
2. Switch to "driver" directory
3. Compile the driver using "make", you will see the module "ch341.ko" if successful
4. Type "sudo make load" or "sudo insmod ch341.ko" to load the driver dynamically
5. Type "sudo make unload" or "sudo rmmod ch341.ko" to unload the driver
6. Type "sudo make install" to make the driver work permanently
7. Type "sudo make uninstall" to remove the driver
8. You can refer to the link below to acquire uart application, you can use gcc or Cross-compile with cross-gcc
