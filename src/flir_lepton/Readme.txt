flir_lepton package
-------------------

Librerias adicionales requeridas:

-OpenCV (https://opencv.org/): Para estructuras de imagen
-zmq (http://zeromq.org/): Para manejo de sockets tcp

Nodos
-----

thermal_image_publisher:

Es el nodo que publica las imagenes.
Este nodo es un cliente que se comunica con el raspberry pi, el cual envia las imagenes a través del ip fijo tcp://192.168.1.9:5555 y que se encuentra como argumento en la linea 46 de este codigo fuente.

thermal_image_listener:

Es un nodo de ejemplo que escucha a las imagenes publicadas por thermal_image_publisher.
La principal característica es que normaliza la imagen de modo que al momento de mostrarse pueda verse como una camara termal.

Driver en Raspberry pi
----------------------

El driver en el raspberry pi esta basado en el driver leptonic (https://github.com/themainframe/leptonic); el cual, adicionalmente de sus las instrucciones dadas por el autor, fue configurado con los siguientes parametros para su mejor funcionamiento:

1. Se modificó la variable spidev.bufsiz del archivo /boot/cmdline.txt la cual define el tamaño del buffer para el puerto spi, finalmente quedó asi:

   spidev.bufsiz=9840

2. Se reservó los núcleos 2 y 3 del raspberry para uso exclusivo del driver, modificando la variable isolcpus del archivo /boot/cmdline.txt, quedando de la siguiente forma:

   isolcpus=2,3

3. Para correr el programa se creó un script bash ejecutable donde se le indica en que nucleos debe correr el programa con el comando taskset -c <<numero_de_nucleo>>. El ejecutable es flir_lepton_driver y se encuentra /home/pi.

4. Finalmente, el programa del raspberry pi corre automaticamente al bootear ya que se agrego una línea al archivo /etc/rc.local en donde se corre el script bash flir_lepton_driver.

5. Adicionalmente, tambien se puede observar la camara termal desde el navegador accediento a la direccion http://192.168.1.9:3000/
